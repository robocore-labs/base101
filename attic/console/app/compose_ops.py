"""Wraps the `docker compose` CLI against COMPOSE_FILE.

Deliberately shells out to `docker compose` rather than reimplementing
container bookkeeping with the Docker SDK: compose already knows how to
recreate a container correctly from the file (volumes, env, networks), which
is exactly what "pull a new image and apply it" needs, and it keeps this
image's dependency footprint to just the docker/compose CLI binaries.
"""

import asyncio
import json
import os
from collections.abc import AsyncIterator

COMPOSE_FILE = os.environ["COMPOSE_FILE"]
# Compose derives a project name from the compose file's parent directory
# when none is given. Running this app natively (uvicorn on the host, no
# container) that resolves correctly on its own, since COMPOSE_FILE is the
# same path whatever process originally launched the real stack used. It
# only breaks when this app itself runs in a container with the compose
# file bind-mounted at a different path (e.g. /deploy) than its real host
# path — then the default resolves to the wrong project and finds zero
# containers. COMPOSE_PROJECT_NAME is the escape hatch for that case; find
# the right value with `docker compose ls` or `docker inspect <container>
# --format '{{index .Config.Labels "com.docker.compose.project"}}'`.
COMPOSE_PROJECT_NAME = os.environ.get("COMPOSE_PROJECT_NAME")

_ACTIONS = ("start", "stop", "restart")


def _compose_args(*args: str) -> list[str]:
    base = ["docker", "compose", "-f", COMPOSE_FILE]
    if COMPOSE_PROJECT_NAME:
        base += ["-p", COMPOSE_PROJECT_NAME]
    return [*base, *args]


async def _run(*args: str) -> tuple[int, str, str]:
    proc = await asyncio.create_subprocess_exec(
        *_compose_args(*args),
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    stdout, stderr = await proc.communicate()
    return proc.returncode, stdout.decode(errors="replace"), stderr.decode(errors="replace")


async def list_services() -> list[dict]:
    code, out, err = await _run("ps", "--all", "--format", "json")
    if code != 0:
        raise RuntimeError(err.strip() or "docker compose ps failed")
    stripped = out.strip()
    if not stripped:
        return []
    # Compose versions differ: some print one JSON array, others one JSON
    # object per line. Handle both rather than pinning a version.
    if stripped.startswith("["):
        return json.loads(stripped)
    return [json.loads(line) for line in stripped.splitlines() if line.strip()]


async def control(service: str, action: str) -> None:
    if action not in _ACTIONS:
        raise ValueError(f"unsupported action {action!r}")
    code, _out, err = await _run(action, service)
    if code != 0:
        raise RuntimeError(err.strip() or f"{action} {service} failed")


async def update(service: str) -> None:
    """Pull the service's image and recreate the container from it."""
    code, _out, err = await _run("pull", service)
    if code != 0:
        raise RuntimeError(err.strip() or f"pull {service} failed")
    code, _out, err = await _run("up", "-d", service)
    if code != 0:
        raise RuntimeError(err.strip() or f"up -d {service} failed")


async def stream_logs(service: str, tail: int = 200) -> AsyncIterator[str]:
    proc = await asyncio.create_subprocess_exec(
        *_compose_args("logs", "-f", "--tail", str(tail), "--no-color", service),
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )
    assert proc.stdout is not None
    try:
        while True:
            line = await proc.stdout.readline()
            if not line:
                break
            yield line.decode(errors="replace").rstrip("\n")
    finally:
        if proc.returncode is None:
            proc.terminate()
