#!/usr/bin/env python3
"""Stage the hardware-only Balena project and invoke the installed Balena CLI."""
import argparse
import ipaddress
import os
from pathlib import Path
import shlex
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / '.runtime/balena'
# Explicit allowlist: never upload native installs, simulation worlds, .env or .git.
DIRECTORIES = ('src/base101', 'src/robocore_viz', 'dependencies/mod101/src', 'dependencies/rplidar_ros')
FILES = ('docker/Dockerfile.ringfour-daemon', 'docker/Dockerfile.ringfour-runner', 'profiles/base101_hw.yaml', 'docker/Dockerfile.agent', 'docker/agent-profile.yaml', 'balena.yml', 'docker/Dockerfile.hardware', 'docker/Dockerfile.zenoh',
         'docker/entrypoint.hardware.sh', 'docker/start.hardware.sh',
         'docker/start.zenoh.sh', 'docker/serial-path.sh', 'docker/zenoh-session.json5',
         'docker/hardware-dashboard.yaml')



def prepare():
    for relative in (*DIRECTORIES, *FILES):
        if not (ROOT / relative).exists():
            raise SystemExit(f'Missing {relative}; run git submodule update --init --recursive')
    STAGE.mkdir(parents=True, exist_ok=True)
    for relative in DIRECTORIES:
        target = STAGE / relative
        # Only replace this script's known, disposable staged package directory.
        if target.is_symlink():
            raise SystemExit(f'Refusing staging symlink: {target}')
        if target.exists():
            shutil.rmtree(target)
        shutil.copytree(ROOT / relative, target,
                        ignore=shutil.ignore_patterns('.git', '.env', '.env.*', '__pycache__', '*.pyc', '*.egg-info'))
    for relative in FILES:
        target = STAGE / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / relative, target)
    shutil.copy2(ROOT / 'docker-compose.balena.yml', STAGE / 'docker-compose.yml')
    (STAGE / '.dockerignore').write_text('**/.git\n**/__pycache__\n**/*.pyc\n')
    return STAGE


def local_target(target):
    if target.endswith('.local'):
        return True
    try:
        ipaddress.ip_address(target)
        return True
    except ValueError:
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=[
        'prepare', 'build', 'deploy', 'push', 'push-fleet', 'devices', 'logs', 'ssh'])
    parser.add_argument('target', nargs='?')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--docker', default=os.environ.get('BALENA_DOCKER_SOCKET'),
                        help='Docker socket; otherwise use the active Docker context')
    args, extra = parser.parse_known_args()
    building = args.action in ('build', 'deploy', 'push', 'push-fleet')
    if (building or args.action in ('logs', 'ssh')) and not args.target:
        parser.error('a fleet slug (build/deploy) or device UUID (logs/ssh) is required')
    if args.target and (args.target.startswith('-') or local_target(args.target)):
        parser.error('normal mode requires a fleet slug or device UUID, not a LAN IP/.local address')
    if args.action == 'prepare':
        if extra:
            parser.error('prepare accepts no additional arguments')
        print(prepare())
        return 0
    if building:
        source = STAGE if args.dry_run else prepare()
        docker_socket = args.docker
        if not docker_socket:
            docker_socket = subprocess.check_output([
                'docker', 'context', 'inspect', '--format', '{{.Endpoints.docker.Host}}'
            ], text=True).strip()
            if not docker_socket.startswith('unix://'):
                parser.error('use --docker /path/to/socket for this Docker context')
            docker_socket = docker_socket[len('unix://'):]
        if args.action == 'build':
            command = ['balena', 'build', str(source), '--fleet', args.target]
        else:
            # deploy --build uses the workstation Docker daemon, then uploads a
            # release to the fleet registry. Never builds on the robot.
            command = ['balena', 'deploy', args.target, '--build', '--source', str(source)]
        command += ['--docker', docker_socket]
    elif args.action == 'devices':
        command = ['balena', 'device', 'list']
        if args.target:
            command += ['--fleet', args.target]
    elif args.action == 'logs':
        command = ['balena', 'device', 'logs', args.target, '--tail']
    else:
        command = ['balena', 'device', 'ssh', args.target]
    command += extra
    print(shlex.join(command), flush=True)
    if args.dry_run:
        return 0
    if not shutil.which('balena'):
        parser.error('Balena CLI is required: https://docs.balena.io/reference/balena-cli/latest/')
    return subprocess.call(command, cwd=ROOT)


if __name__ == '__main__':
    raise SystemExit(main())
