"""FastAPI backend for the calibration UI.

Runs the existing camera_lidar_calibration C++ binary as a subprocess and
exposes the parsed result over HTTP. Serves a single static HTML page at /.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import yaml
from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

PROJECT_ROOT = Path(__file__).resolve().parent.parent
BUILD_DIR = PROJECT_ROOT / "build"
BINARY = BUILD_DIR / "camera_lidar_calibration"
RESULT_FILE = PROJECT_ROOT / "results" / "calibration.yaml"
STATIC_DIR = Path(__file__).resolve().parent / "static"

app = FastAPI(title="Camera-LiDAR Calibration UI")


def _read_result() -> dict:
    if not RESULT_FILE.exists():
        raise HTTPException(status_code=404, detail=f"No result yet at {RESULT_FILE}")
    with RESULT_FILE.open() as fh:
        return yaml.safe_load(fh)


@app.post("/api/calibrate")
def run_calibration() -> JSONResponse:
    if not BINARY.exists():
        raise HTTPException(
            status_code=500,
            detail=f"Binary missing at {BINARY}. Build with: cmake --build build",
        )
    proc = subprocess.run(
        [str(BINARY)],
        cwd=str(BUILD_DIR),
        capture_output=True,
        text=True,
        timeout=120,
    )
    if proc.returncode != 0:
        raise HTTPException(
            status_code=500,
            detail={"stdout": proc.stdout, "stderr": proc.stderr},
        )
    return JSONResponse(
        {
            "stdout": proc.stdout,
            "stderr": proc.stderr,
            "result": _read_result(),
        }
    )


@app.get("/api/result")
def get_result() -> dict:
    return _read_result()


app.mount("/", StaticFiles(directory=str(STATIC_DIR), html=True), name="static")
