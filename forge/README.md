# forge/

`base101.yaml` — the [forge](https://github.com/robocore-labs/forge — or
wherever `~/Work/forge` is checked out) deployment config: hosts, Docker
components, the zenoh router. Moved here from the repo root on 2026-09-27,
grouped rather than sitting loose at top level.

**Deprecated as the primary hardware deployment path.** As of this session,
the robot runs `pixi` directly (see `PIXI.md`) instead of forge-built
per-component Docker containers — `pixi run build` is fast enough on-device
that forge's whole reason to exist (cross-compile once, distribute images)
stopped paying for its complexity. This file is kept as a working reference,
not because it's still the recommended path.

If you do still use it, the config file moved but forge's own path
resolution didn't change: component `source:` paths resolve against
whatever `-p`/`--project-root` you pass, not against the config file's own
location. From the repo root:

```bash
forge -p . -f forge/base101.yaml stage
forge -p . -f forge/base101.yaml launch
```

See `HARDWARE.md` for what actually runs the robot now.
