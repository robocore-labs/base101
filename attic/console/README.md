# base101 console

A minimal ops dashboard for a running base101 deployment — complementary to
forge, not a replacement for it. Forge builds images and stages/launches the
workspace (the hot dev loop); this operates whatever forge already put in
place. Two things it does:

1. **Container control** for whatever `docker compose` file you point it at:
   list services + status, start/stop/restart, pull-and-recreate ("update"),
   and stream logs.
2. **Config hot-reload**: a raw text editor, sandboxed to one directory
   (`CONFIG_ROOT`), for the `*.yaml` files stacks bind-mount from the host
   (`ekf.hw.yaml`, `twist_mux.yaml`, `lidar_filters.yaml`, ...). Because
   those are bind-mounted rather than baked into images, editing one and
   restarting the one owning container *is* the hot reload — no rebuild, no
   forge cycle.

**No authentication, no HTTPS.** This mounts the Docker socket, which is
root-equivalent access to the host, and lets anyone who can reach the page
rewrite config files — same trust model as rosboard (`:8888`) and
`base101_teleop` (`:8700`) already running unauthenticated on this robot.
**Do not expose this beyond the robot's LAN.**

## Running

The frontend is served by the same FastAPI app as the API (via `StaticFiles`
in `app/main.py`) — there's no separate static server and no build step.
Don't open `static/index.html` directly as a `file://` URL: its JS calls
`/api/...` as a same-origin path, which only resolves once something is
actually serving the app over HTTP.

### Locally, no Docker (fastest for frontend work)

```bash
cd console
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
COMPOSE_FILE=/absolute/path/to/some-docker-compose.yaml \
CONFIG_ROOT=/absolute/path/to/some/config/dir \
  .venv/bin/uvicorn app.main:app --reload --host 127.0.0.1 --port 8090
```

Open `http://127.0.0.1:8090`. This talks to your machine's own `docker`
CLI directly (no socket mount, no bind-mount path remapping), so
`COMPOSE_PROJECT_NAME` isn't needed here — compose resolves the project the
same way it would from a normal shell, since `COMPOSE_FILE` is the file's
real path. `--reload` picks up backend edits; static file edits (`app.js`,
`style.css`) just need a browser refresh.

### As a container (how it runs on the robot)

```bash
cd console
docker compose up --build
```

Open `http://<host>:8090`.

Environment variables (export before `docker compose up`, or put in a
`.env` file next to `docker-compose.yaml`) — these only apply to the
containerized path above, not the local/no-Docker one:

| Variable | Default | Meaning |
|---|---|---|
| `DEPLOY_ROOT` | `..` (repo root) | Host directory bind-mounted into the console at `/deploy`. Point this at wherever the real compose file + staged config live. |
| `CONSOLE_COMPOSE_FILE` | `/deploy/docker-compose.robot.yaml` | Path (inside the container, under `/deploy`) to the compose file to manage. |
| `CONSOLE_COMPOSE_PROJECT` | *(optional; required in practice for this path)* | The existing compose project name for the stack you're managing. **Must match exactly** — see the callout below. |
| `CONSOLE_CONFIG_ROOT` | `/deploy` | Path (inside the container) the config editor is sandboxed to. |

### The project-name gotcha (containerized path only)

`docker compose` derives a project name from the compose file's parent
directory when none is given. This container sees the compose file at
`/deploy/...` (the bind-mount path), which is *not* the path whoever
originally ran `docker compose up` used on the real host — so the default
name won't match, and every command will act like the stack has zero
containers. You must pass the **actual** project name explicitly via
`CONSOLE_COMPOSE_PROJECT`. Find it with either:

```bash
docker compose ls                                                   # on the host
docker inspect <a running container> \
  --format '{{index .Config.Labels "com.docker.compose.project"}}'
```

### On the real robot

Once forge has staged a deployment (`/home/cdr/robocore-deploy/...` per
`hardware.yaml`), point this at it:

```bash
DEPLOY_ROOT=/home/cdr/robocore-deploy \
CONSOLE_COMPOSE_FILE=/deploy/docker-compose.robot.yaml \
docker compose -f console/docker-compose.yaml up -d --build
```

**This hasn't been validated against a live robot** — only smoke-tested
locally against a throwaway compose file (below). Confirm the actual
compose file path and layout forge leaves under `robocore-deploy` before
relying on it, and adjust `CONSOLE_COMPOSE_FILE`/`CONSOLE_CONFIG_ROOT` if it
doesn't match.

## Local smoke test (no robot needed)

```bash
cd console
cat > dev-stack.yaml <<'EOF'
services:
  web:
    image: nginx:alpine
    ports: ["8091:80"]
  worker:
    image: busybox
    command: sleep infinity
EOF
docker compose -f dev-stack.yaml up -d

DEPLOY_ROOT=. CONSOLE_COMPOSE_FILE=/deploy/dev-stack.yaml CONSOLE_CONFIG_ROOT=/deploy \
CONSOLE_COMPOSE_PROJECT=console \
  docker compose up --build
```

Open `http://localhost:8090` — `web`/`worker` should list, and
start/stop/restart/update/logs should all work against them. `dev-stack.yaml`
is scratch, not meant to be committed.

## API

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/services` | `docker compose ps --all --format json` |
| POST | `/api/services/{name}/{start\|stop\|restart\|update}` | Control one service (`update` = pull + `up -d`) |
| GET | `/api/services/{name}/logs` | SSE stream of `docker compose logs -f` |
| GET | `/api/config/tree` | List `*.yaml`/`*.yml` files under `CONFIG_ROOT` |
| GET | `/api/config/file?path=...` | Read one file |
| PUT | `/api/config/file` | Write one file (rejected if not valid YAML) |

## Not in v1

Structured per-stack config forms (only a raw file editor exists today),
authentication, and any automatic file→service mapping — after saving a
config you pick which service to restart yourself from the services panel.
