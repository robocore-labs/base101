#!/usr/bin/env python3
"""URDF (xacro output) -> MJCF, fully automatic. See mujoco.md.

    xacro .../mod101.xacro simulator:=mujoco | ros2 run mod101_mujoco urdf_to_mjcf - -o mod101.xml
    ros2 run mod101_mujoco urdf_to_mjcf robot.urdf --freejoint base_link --floor -o robot.xml

Also a module: mod101_mujoco/launch/mujoco.launch.py imports build() and
merge_ros2_control() to generate the model at launch time.

The xacro stays the single robot description; this never writes anything back
into it and its output is a build artifact, not something to commit or edit.

What MuJoCo's own URDF importer needs help with (all found by spiking on real
base101/mod101 output, see mujoco.md):
  - non-positive principal inertias (CAD exporters round tiny parts to 0) are
    rejected *during parsing*, before any compiler bounding runs -> clamp first
  - `file://` / `package://` mesh URIs aren't understood -> resolve to paths
  - fusestatic/discardvisual/strippath defaults would drop fixed frames (the
    ones sensors attach to), drop every visual mesh, and flatten mesh paths
What URDF doesn't carry at all, derived here from what it does carry:
  - actuators, from each joint's <ros2_control> command_interface
    (position -> position servo, velocity -> velocity servo)
  - joint couplings, from URDF <mimic> (the importer drops them, leaving
    e.g. the parallel gripper's second jaw and PincOpen's linkage limp)
  - cameras, from the Gazebo <sensor type="camera"> blocks the URDF already
    has for Gazebo (same frame, FOV, resolution) — not a second definition
  - convex decomposition of concave collision meshes (CoACD, cached): MuJoCo
    collides every mesh as its convex hull, which e.g. fills the jaw's own
    grasping gap. Gazebo collides the triangle mesh itself, so URDF authors
    never had to care.
Per-simulator tuning (gains, armature, friction) is the one non-derivable
piece: `--overrides` YAML, same role Gazebo's .gazebo/controllers files play.
"""
import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
import xml.etree.ElementTree as ET

import mujoco

EPS_INERTIA = 1e-9

# Deliberately generic defaults — anything robot-specific belongs in --overrides.
DEFAULTS = {
    'position': {'kp': 20.0, 'kv': 1.0},
    'velocity': {'kv': 5.0},
    # Some rotor inertia on every actuated joint. With 0, a servo on a light
    # part (PincOpen's 1 g coupler, inertia 1e-7) is stiffer than the 2 ms
    # step can integrate and the tool explodes. 1e-3 keeps kp <= ~200 stable.
    'armature': 0.001,
    'joints': {},        # per-joint: {name: {kp, kv, armature, damping, frictionloss}}
}


def resolve_mesh_uri(uri: str) -> str:
    if uri.startswith('file://'):
        return uri[len('file://'):]
    if uri.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        pkg, rest = uri[len('package://'):].split('/', 1)
        return f'{get_package_share_directory(pkg)}/{rest}'
    return uri


def preprocess(urdf_text: str):
    """Fix what the importer can't take; collect what it'll drop (ros2_control)."""
    root = ET.fromstring(urdf_text)

    clamped = []
    for link in root.iter('link'):
        inertia = link.find('inertial/inertia')
        if inertia is None:
            continue
        for k in ('ixx', 'iyy', 'izz'):
            if float(inertia.get(k, 0)) <= 0:
                inertia.set(k, repr(EPS_INERTIA))
                clamped.append(f"{link.get('name')}.{k}")

    for mesh in root.iter('mesh'):
        mesh.set('filename', resolve_mesh_uri(mesh.get('filename', '')))

    cameras = []
    for gz in root.iter('gazebo'):
        for sensor in gz.findall('sensor'):
            if sensor.get('type') not in ('camera', 'rgbd_camera', 'depth_camera'):
                continue
            num = lambda path, default: float(sensor.findtext(path, default))
            cameras.append({
                'name': sensor.get('name'),
                'link': gz.get('reference'),                  # looks along +X (Gazebo)
                'optical_frame': sensor.findtext('gz_frame_id'),  # REP-103, if declared
                'topic': sensor.findtext('topic'),
                'hfov': num('camera/horizontal_fov', 1.047),
                'width': int(num('camera/image/width', 640)),
                'height': int(num('camera/image/height', 480)),
                'rate': num('update_rate', 30),
            })

    # follower = offset + multiplier * leader, per URDF <mimic>
    mimics = [(joint.get('name'), m.get('joint'), float(m.get('multiplier', 1)), float(m.get('offset', 0)))
              for joint in root.iter('joint') if (m := joint.find('mimic')) is not None]

    interfaces = {}  # joint -> 'position' | 'velocity'
    for rc in root.findall('ros2_control'):
        for joint in rc.findall('joint'):
            cmds = [c.get('name') for c in joint.findall('command_interface')]
            if 'position' in cmds:
                interfaces[joint.get('name')] = 'position'
            elif 'velocity' in cmds:
                interfaces[joint.get('name')] = 'velocity'
        root.remove(rc)  # consumed here; MuJoCo has no use for it

    mj = ET.SubElement(root, 'mujoco')
    ET.SubElement(mj, 'compiler', {
        'fusestatic': 'false',     # keep imu_link/lidar_frame/camera frames
        'discardvisual': 'false',  # keep the STL visuals
        'strippath': 'false',      # use the resolved absolute mesh paths
    })
    return ET.tostring(root, encoding='unicode'), interfaces, clamped, cameras, mimics


def merge_ros2_control(urdf_text: str, name: str, hardware_params: dict) -> str:
    """Fold every <ros2_control> system into one, and set hardware params on it.

    mujoco_ros2_control's MujocoSystemInterface runs a whole physics world per
    instance, so the arm's system and the tool's system (separate blocks, which
    is what gz_ros2_control and the real bridges want) would become two
    unrelated simulations. Joints/sensors/gpios move into the first block; all
    blocks must already name the same plugin (xacro simulator:=mujoco).
    """
    root = ET.fromstring(urdf_text)
    systems = root.findall('ros2_control')
    if not systems:
        raise ValueError('no <ros2_control> blocks in the URDF')
    plugins = {rc.findtext('hardware/plugin', '').strip() for rc in systems}
    if len(plugins) != 1:
        raise ValueError(f'<ros2_control> blocks use different plugins {sorted(plugins)}; '
                         f'expected all MuJoCo (xacro simulator:=mujoco)')
    merged = systems[0]
    merged.set('name', name)
    for rc in systems[1:]:
        for child in rc:
            if child.tag != 'hardware':
                merged.append(child)
        root.remove(rc)
    hardware = merged.find('hardware')
    for key, value in hardware_params.items():
        for old in [p for p in hardware.findall('param') if p.get('name') == key]:
            hardware.remove(old)
        ET.SubElement(hardware, 'param', {'name': key}).text = str(value)
    return ET.tostring(root, encoding='unicode')


def add_camera(spec, cam):
    """A MuJoCo camera looks along its -Z with +Y up; REP-103 optical frames
    look along +Z with +Y down (a 180 deg turn about X), and Gazebo sensors
    look along their link's +X with +Z up."""
    fovy = math.degrees(2 * math.atan(math.tan(cam['hfov'] / 2) * cam['height'] / cam['width']))
    body = spec.body(cam['optical_frame']) if cam['optical_frame'] else None
    if body is not None:
        quat = [0, 1, 0, 0]
    else:
        body = spec.body(cam['link'])
        if body is None:
            return False
        quat = [0.5, 0.5, -0.5, -0.5]  # cam -Z = link +X, cam +Y = link +Z
    body.add_camera(name=cam['name'], quat=quat, fovy=fovy,
                    resolution=[cam['width'], cam['height']])
    return True


DECOMPOSE = {'threshold': 0.05, 'max_convex_hull': 16}
CONVEX_ENOUGH = 0.85   # mesh volume / hull volume at or above this: leave as is


def _cache_dir():
    base = os.environ.get('XDG_CACHE_HOME', os.path.expanduser('~/.cache'))
    return os.path.join(base, 'mod101_mujoco', 'coacd')


def convex_parts(path):
    """Convex pieces of a mesh file (in the file's own units), or [] if it is
    already close enough to convex. Cached by file content + parameters, so
    only the first launch after a mesh changes pays for CoACD (seconds per
    mesh)."""
    key = hashlib.sha1(open(path, 'rb').read() + json.dumps(
        [DECOMPOSE, CONVEX_ENOUGH], sort_keys=True).encode()).hexdigest()[:16]
    out = os.path.join(_cache_dir(), key)
    index = os.path.join(out, 'parts.json')
    if os.path.exists(index):
        return [os.path.join(out, f) for f in json.load(open(index))]

    import coacd
    import trimesh
    mesh = trimesh.load(path, force='mesh')
    files = []
    if not (mesh.is_watertight and mesh.volume / mesh.convex_hull.volume >= CONVEX_ENOUGH):
        coacd.set_log_level('error')
        parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), **DECOMPOSE)
        os.makedirs(out, exist_ok=True)
        for i, (verts, faces) in enumerate(parts):
            files.append(f'part{i}.stl')
            trimesh.Trimesh(verts, faces).export(os.path.join(out, files[-1]))
    os.makedirs(out, exist_ok=True)
    json.dump(files, open(index, 'w'))
    return [os.path.join(out, f) for f in files]


def decompose_collision(spec):
    """Replace each concave collision mesh geom by its convex parts."""
    done = {}
    for geom in list(spec.geoms):
        if geom.type != mujoco.mjtGeom.mjGEOM_MESH or (geom.contype == 0 and geom.conaffinity == 0):
            continue
        mesh = spec.mesh(geom.meshname)
        parts = convex_parts(mesh.file)
        if not parts:
            continue
        for i, part in enumerate(parts):
            name = f'{mesh.name}__cvx{i}'
            if spec.mesh(name) is None:
                spec.add_mesh(name=name, file=part, scale=mesh.scale)
            geom.parent.add_geom(
                type=mujoco.mjtGeom.mjGEOM_MESH, meshname=name, pos=geom.pos, quat=geom.quat,
                contype=geom.contype, conaffinity=geom.conaffinity, condim=geom.condim,
                friction=geom.friction, group=geom.group, rgba=geom.rgba)
        done[mesh.name] = len(parts)
        spec.delete(geom)
    return done


def srdf_excludes(srdf_text: str):
    """SRDF <disable_collisions> pairs — the collision matrix MoveIt already uses."""
    root = ET.fromstring(srdf_text)
    return [(dc.get('link1'), dc.get('link2')) for dc in root.iter('disable_collisions')]


def build(urdf_text: str, overrides: dict, freejoint: str | None, floor: bool,
          srdf_text: str | None = None, decompose: bool = True):
    cfg = {**DEFAULTS, **overrides}
    fixed_urdf, interfaces, clamped, cameras, mimics = preprocess(urdf_text)

    with tempfile.NamedTemporaryFile('w', suffix='.urdf') as f:
        f.write(fixed_urdf)
        f.flush()
        spec = mujoco.MjSpec.from_file(f.name)

    # MuJoCo only auto-excludes direct parent/child contacts. fusestatic=false
    # (needed to keep sensor frames) makes physically-mated parts like
    # servo_base_1 vs shoulder_rotation_link_1 non-parent pairs that grind
    # against each other forever — and fight the joints. The SRDF already
    # lists exactly which pairs never meaningfully collide.
    excluded, unknown = 0, []
    body_names = {b.name for b in spec.bodies}
    for l1, l2 in (srdf_excludes(srdf_text) if srdf_text else []):
        if l1 in body_names and l2 in body_names:
            spec.add_exclude(bodyname1=l1, bodyname2=l2)
            excluded += 1
        else:
            unknown.append((l1, l2))

    # The importer keeps both URDF geometries: <visual> STLs in group 1
    # (contype 0) and <collision> shapes in group 0 — for mod101 simple boxes,
    # which MuJoCo draws by default (groups 0-2), hiding the meshes inside
    # them. Gazebo hides collision geometry; do the same with the usual MJCF
    # convention: visuals group 2, collision group 3 (hidden in the viewer and
    # in camera renders, toggle with '3' in the viewer to inspect).
    for geom in spec.geoms:
        if geom.contype == 0 and geom.conaffinity == 0:
            geom.group = 2
        else:
            geom.group = 3
    decomposed = decompose_collision(spec) if decompose else {}

    for follower, leader, multiplier, offset in mimics:
        # MuJoCo joint equality: q_follower = polycoef[0] + polycoef[1] * q_leader + ...
        spec.add_equality(type=mujoco.mjtEq.mjEQ_JOINT, objtype=mujoco.mjtObj.mjOBJ_JOINT,
                          name1=follower, name2=leader,
                          data=[offset, multiplier] + [0.0] * 9)

    if freejoint:
        spec.body(freejoint).add_freejoint()
    if floor:
        spec.worldbody.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE,
                                size=[10, 10, 0.1], rgba=[0.3, 0.3, 0.3, 1])
        spec.worldbody.add_light(pos=[0, 0, 3], dir=[0, 0, -1])

    cameras = [c for c in cameras if add_camera(spec, c)]

    missing = []
    for jname, kind in interfaces.items():
        joint = spec.joint(jname)
        if joint is None:
            missing.append(jname)
            continue
        jcfg = cfg['joints'].get(jname, {})
        joint.armature = jcfg.get('armature', cfg['armature'])
        if 'damping' in jcfg:
            joint.damping = jcfg['damping']
        if 'frictionloss' in jcfg:
            joint.frictionloss = jcfg['frictionloss']

        act = spec.add_actuator(name=jname, target=jname,
                                trntype=mujoco.mjtTrn.mjTRN_JOINT)
        if kind == 'position':
            act.set_to_position(kp=jcfg.get('kp', cfg['position']['kp']),
                                kv=jcfg.get('kv', cfg['position']['kv']))
            if joint.range[0] < joint.range[1]:
                act.ctrlrange = joint.range
                act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
        else:
            act.set_to_velocity(kv=jcfg.get('kv', cfg['velocity']['kv']))

    model = spec.compile()
    return spec, model, {'clamped': clamped, 'actuated': interfaces, 'missing': missing,
                         'excluded': excluded, 'unknown_pairs': unknown,
                         'cameras': cameras, 'decomposed': decomposed,
                         'mimics': [f for f, *_ in mimics]}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('urdf', help="URDF file, or '-' for stdin")
    ap.add_argument('-o', '--output', help='MJCF path (default: stdout)')
    ap.add_argument('--overrides', help='YAML: per-sim tuning (gains, armature, ...)')
    ap.add_argument('--freejoint', metavar='BODY',
                    help='give this body a free joint (mobile base); default: welded')
    ap.add_argument('--floor', action='store_true', help='add a ground plane + light')
    ap.add_argument('--srdf', help='SRDF whose <disable_collisions> become contact excludes')
    ap.add_argument('--no-decompose', action='store_true',
                    help='collide concave meshes as their convex hulls (skip CoACD)')
    a = ap.parse_args()

    urdf_text = sys.stdin.read() if a.urdf == '-' else open(a.urdf).read()
    overrides = {}
    if a.overrides:
        import yaml
        overrides = yaml.safe_load(open(a.overrides)) or {}
    srdf_text = open(a.srdf).read() if a.srdf else None

    spec, model, report = build(urdf_text, overrides, a.freejoint, a.floor, srdf_text,
                                not a.no_decompose)
    xml = spec.to_xml()
    if a.output:
        open(a.output, 'w').write(xml)
    else:
        sys.stdout.write(xml)

    print(f'[urdf_to_mjcf] {model.nbody} bodies, {model.njnt} joints, '
          f'{model.nu} actuators, {model.ncam} cameras, {model.nmesh} meshes; '
          f'clamped {len(report["clamped"])} inertia terms; '
          f'{report["excluded"]} contact excludes from SRDF; '
          f'{len(report["decomposed"])} concave meshes decomposed', file=sys.stderr)
    if report['unknown_pairs']:
        print(f'[urdf_to_mjcf] WARNING: {len(report["unknown_pairs"])} SRDF pairs name '
              f'links not in the model (stale SRDF?): {report["unknown_pairs"][:5]}',
              file=sys.stderr)
    if report['missing']:
        print(f'[urdf_to_mjcf] WARNING: ros2_control joints not in model: '
              f'{report["missing"]}', file=sys.stderr)


if __name__ == '__main__':
    main()
