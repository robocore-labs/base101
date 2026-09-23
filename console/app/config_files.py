"""Sandboxed read/write access to *.yaml/*.yml files under CONFIG_ROOT.

This is the "hot reload" half of the console: stacks bind-mount their config
directories from the host, so editing a file here and restarting the owning
container (via compose_ops) is the whole reload path — no image rebuild.
"""

import os
from pathlib import Path

import yaml

CONFIG_ROOT = Path(os.environ["CONFIG_ROOT"]).resolve()
ALLOWED_SUFFIXES = {".yaml", ".yml"}


class ConfigPathError(ValueError):
    pass


def _resolve(rel_path: str) -> Path:
    candidate = (CONFIG_ROOT / rel_path).resolve()
    if not candidate.is_relative_to(CONFIG_ROOT):
        raise ConfigPathError(f"path escapes CONFIG_ROOT: {rel_path!r}")
    if candidate.suffix not in ALLOWED_SUFFIXES:
        raise ConfigPathError(f"not a yaml file: {rel_path!r}")
    return candidate


def list_tree() -> list[str]:
    return sorted(
        str(p.relative_to(CONFIG_ROOT))
        for p in CONFIG_ROOT.rglob("*")
        if p.is_file() and p.suffix in ALLOWED_SUFFIXES
    )


def read_file(rel_path: str) -> str:
    path = _resolve(rel_path)
    if not path.is_file():
        raise FileNotFoundError(rel_path)
    return path.read_text()


def write_file(rel_path: str, content: str) -> None:
    path = _resolve(rel_path)
    if not path.is_file():
        raise FileNotFoundError(rel_path)
    yaml.safe_load(content)  # raises yaml.YAMLError on invalid syntax
    path.write_text(content)
