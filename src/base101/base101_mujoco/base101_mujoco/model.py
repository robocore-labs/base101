"""Generate MuJoCo assets from the canonical xacro; never hand-maintain MJCF."""
from pathlib import Path
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
import mujoco
import xacro
from ament_index_python.packages import get_package_share_directory
from .converter import build, resolve_mesh_uri


def prepare(runtime, arm=False):
    runtime=Path(runtime);runtime.mkdir(parents=True,exist_ok=True)
    share=Path(get_package_share_directory('base101_description'))
    description=xacro.process_file(str(share/'urdf/base101.xacro'),mappings={
        'simulator':'mujoco','arm':str(arm).lower(),'camera':'realsense'}).toxml()
    root=ET.fromstring(description)
    joints=[]
    systems=root.findall('ros2_control')
    for system in systems:
        for joint in system.findall('joint'):
            name=joint.get('name');source=root.find(f"joint[@name='{name}']")
            interface=joint.find('command_interface')
            if interface is None:continue
            kind=interface.get('name')
            limit=source.find('limit') if source is not None else None
            lo=-30. if kind=='velocity' else float(limit.get('lower',-math.pi))
            hi=30. if kind=='velocity' else float(limit.get('upper',math.pi))
            for child in list(joint):
                if child.tag in ('command_interface','state_interface'):joint.remove(child)
            command=ET.SubElement(joint,'command_interface',name=kind)
            ET.SubElement(command,'param',name='min').text=str(lo)
            ET.SubElement(command,'param',name='max').text=str(hi)
            for state in ('position','velocity'):ET.SubElement(joint,'state_interface',name=state)
            joints.append((name,kind,joint))
        root.remove(system)
    system=ET.SubElement(root,'ros2_control',name='Base101Simulation',type='system')
    hardware=ET.SubElement(system,'hardware')
    ET.SubElement(hardware,'plugin').text='base101_sim_control/SimulatorSystem'
    ET.SubElement(hardware,'param',name='state_file').text=str(runtime/'control.bin')
    for _,_,joint in joints:system.append(joint)
    # MuJoCo keys imported mesh assets by basename, even with strippath=false.
    # Different packages both own base_link.stl: namespace every resolved asset.
    assets=runtime/'assets';assets.mkdir(exist_ok=True)
    for mesh in root.iter('mesh'):
        source=Path(resolve_mesh_uri(mesh.get('filename')))
        alias=assets/(source.stem+'_'+hashlib.sha256(str(source).encode()).hexdigest()[:12]+source.suffix)
        if not alias.exists():shutil.copy2(source,alias)
        mesh.set('filename',str(alias))
    description=ET.tostring(root,encoding='unicode')
    srdf=ET.Element('robot',name='base101')
    if arm:
        tool='jaws' if root.find("link[@name='arm_jaws_root']") is not None else 'parallel' if root.find("link[@name='arm_parallel_root']") is not None else 'none'
        configs=Path(get_package_share_directory('base101_mujoco'))/'config'
        for label in ('base101','mod101'):
            source=ET.parse(configs/f'{label}_{tool}_collisions.xml').getroot()
            for pair in source.iter('disable_collisions'):
                ET.SubElement(srdf,'disable_collisions',link1=pair.get('link1').replace('${prefix}','arm_'),link2=pair.get('link2').replace('${prefix}','arm_'))
    spec,_,report=build(description,{},'base_link',True,srdf_text=ET.tostring(srdf,encoding='unicode'),decompose=False)
    # Fixed-connected chassis pieces are a rigid assembly, not self obstacles.
    # Exclude only pairs in the same fixed-joint component; movable collisions remain.
    parent={link.get('name'):link.get('name') for link in root.findall('link')}
    def find(n):
        while parent[n]!=n:n=parent[n]
        return n
    for joint in root.findall('joint'):
        if joint.get('type')=='fixed':parent[find(joint.find('child').get('link'))]=find(joint.find('parent').get('link'))
    names={b.name for b in spec.bodies}
    existing={(e.bodyname1,e.bodyname2) for e in spec.excludes}
    links=list(parent)
    for i,a in enumerate(links):
        for b in links[i+1:]:
            if a in names and b in names and find(a)==find(b) and (a,b) not in existing and (b,a) not in existing:
                spec.add_exclude(bodyname1=a,bodyname2=b)
    # A bounded room with asymmetric landmarks makes camera/lidar validation useful.
    for name,pos,size,color in [
        ('north',[0,3,.5],[3,.06,.5],[.65,.7,.8,1]),
        ('south',[0,-3,.5],[3,.06,.5],[.7,.6,.5,1]),
        ('east',[3,0,.5],[.06,3,.5],[.6,.7,.6,1]),
        ('west',[-3,0,.5],[.06,3,.5],[.6,.6,.7,1]),
        ('crate',[1.1,.5,.2],[.2,.3,.2],[.75,.2,.1,1]),
        ('pillar',[-.8,1,.4],[.15,.15,.4],[.1,.4,.8,1])]:
        spec.worldbody.add_geom(name=name,type=mujoco.mjtGeom.mjGEOM_BOX,pos=pos,size=size,rgba=color)
    spec.option.timestep=.001
    spec.option.integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model=spec.compile()
    mujoco.mj_saveModel(model,str(runtime/'scene.mjb'))
    (runtime/'robot.urdf').write_text(description)
    (runtime/'scene.xml').write_text(spec.to_xml())
    digest=hashlib.sha256((runtime/'scene.mjb').read_bytes()).hexdigest()
    metadata={'joints':[(n,k) for n,k,_ in joints],'cameras':report['cameras'],'digest':digest}
    (runtime/'model.json').write_text(json.dumps(metadata))
    return model,metadata,description
