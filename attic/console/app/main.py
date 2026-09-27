import json
import re
from pathlib import Path

import yaml
from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from . import compose_ops, config_files

app = FastAPI(title="base101 console")

STATIC_DIR = Path(__file__).resolve().parent.parent / "static"
_SERVICE_NAME_RE = re.compile(r"[A-Za-z0-9_.-]+")


def _validate_service_name(name: str) -> None:
    if not _SERVICE_NAME_RE.fullmatch(name):
        raise HTTPException(status_code=400, detail="invalid service name")


@app.get("/api/services")
async def get_services():
    try:
        return await compose_ops.list_services()
    except RuntimeError as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc


@app.post("/api/services/{name}/{action}")
async def control_service(name: str, action: str):
    _validate_service_name(name)
    if action not in {"start", "stop", "restart", "update"}:
        raise HTTPException(status_code=404, detail="unknown action")
    try:
        if action == "update":
            await compose_ops.update(name)
        else:
            await compose_ops.control(name, action)
    except RuntimeError as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    return {"ok": True}


@app.get("/api/services/{name}/logs")
async def service_logs(name: str):
    _validate_service_name(name)

    async def event_stream():
        async for line in compose_ops.stream_logs(name):
            yield f"data: {json.dumps(line)}\n\n"

    return StreamingResponse(
        event_stream(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


@app.get("/api/config/tree")
async def config_tree():
    return config_files.list_tree()


@app.get("/api/config/file")
async def config_read(path: str):
    try:
        return {"path": path, "content": config_files.read_file(path)}
    except config_files.ConfigPathError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


class ConfigWrite(BaseModel):
    path: str
    content: str


@app.put("/api/config/file")
async def config_write(body: ConfigWrite):
    try:
        config_files.write_file(body.path, body.content)
    except config_files.ConfigPathError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except yaml.YAMLError as exc:
        raise HTTPException(status_code=400, detail=f"invalid YAML: {exc}") from exc
    return {"ok": True}


# Mounted last so it acts as a fallback behind the /api routes above.
app.mount("/", StaticFiles(directory=STATIC_DIR, html=True), name="static")
