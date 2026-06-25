"""FastAPI backend for the calibration UI.

Drives the existing camera_lidar_calibration C++ binary as a subprocess and
exposes parsed data over HTTP. Frontend is plain HTML/JS in ui/static/.
"""

from __future__ import annotations

import csv
import re
import shutil
import subprocess
from pathlib import Path
from typing import List, Optional

import yaml
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

PROJECT_ROOT = Path(__file__).resolve().parent.parent
BUILD_DIR = PROJECT_ROOT / "build"
BINARY = BUILD_DIR / "camera_lidar_calibration"
PARAMS_FILE = PROJECT_ROOT / "config" / "params.yaml"
RESULT_FILE = PROJECT_ROOT / "results" / "calibration.yaml"
DATA_DIR = PROJECT_ROOT / "data"
UPLOADS_DIR = DATA_DIR / "uploads"
STATIC_DIR = Path(__file__).resolve().parent / "static"

LINES_PER_POSE = 19  # ACFR/MATLAB poses.csv layout
MM_TO_M = 1e-3

app = FastAPI(title="Camera-LiDAR Calibration UI")


# --- helpers --------------------------------------------------------------

def _list_datasets() -> List[dict]:
    """Any folder under data/ that contains a poses.csv counts as a dataset.

    Scans direct children of data/ and also data/uploads/* so uploaded
    datasets show up alongside hand-placed ones.
    """
    if not DATA_DIR.exists():
        return []
    out = []
    candidates = list(DATA_DIR.iterdir())
    if UPLOADS_DIR.exists():
        candidates += list(UPLOADS_DIR.iterdir())
    for sub in sorted(candidates, key=lambda p: str(p)):
        if not sub.is_dir():
            continue
        poses = sub / "poses.csv"
        if not poses.exists():
            continue
        # Name is relative to data/ so uploads keep their "uploads/<name>" prefix.
        rel_name = str(sub.relative_to(DATA_DIR)).replace("\\", "/")
        out.append({
            "name": rel_name,
            "poses_csv": str(poses.relative_to(PROJECT_ROOT)),
            "num_poses": _count_poses(poses),
        })
    return out


def _count_poses(poses_csv: Path) -> int:
    try:
        with poses_csv.open() as fh:
            n = sum(1 for ln in fh if ln.strip())
        return n // LINES_PER_POSE
    except OSError:
        return 0


def _parse_triple(line: str) -> List[float]:
    parts = [p.strip() for p in line.split(",")]
    return [float(parts[0]), float(parts[1]), float(parts[2])]


def _load_poses(poses_csv: Path) -> dict:
    """Parses ACFR poses.csv into a structured list per sensor. mm -> m."""
    if not poses_csv.exists():
        raise HTTPException(status_code=404, detail=f"poses.csv not found: {poses_csv}")
    lines = [ln.strip() for ln in poses_csv.read_text().splitlines() if ln.strip()]
    if len(lines) % LINES_PER_POSE != 0:
        raise HTTPException(
            status_code=400,
            detail=f"poses.csv line count {len(lines)} is not a multiple of {LINES_PER_POSE}",
        )
    poses = []
    for p in range(len(lines) // LINES_PER_POSE):
        base = p * LINES_PER_POSE
        cam_centroid = [v * MM_TO_M for v in _parse_triple(lines[base + 0])]
        cam_normal   = _parse_triple(lines[base + 1])
        cam_corners  = [[v * MM_TO_M for v in _parse_triple(lines[base + 2 + i])] for i in range(4)]
        lid_centroid = [v * MM_TO_M for v in _parse_triple(lines[base + 6])]
        lid_normal   = _parse_triple(lines[base + 7])
        lid_corners  = [[v * MM_TO_M for v in _parse_triple(lines[base + 8 + i])] for i in range(4)]
        poses.append({
            "index": p,
            "camera": {"centroid": cam_centroid, "normal": cam_normal, "corners": cam_corners},
            "lidar":  {"centroid": lid_centroid, "normal": lid_normal, "corners": lid_corners},
        })
    return {"num_poses": len(poses), "poses": poses}


def _read_result() -> dict:
    if not RESULT_FILE.exists():
        raise HTTPException(status_code=404, detail=f"No result yet at {RESULT_FILE}")
    with RESULT_FILE.open() as fh:
        return yaml.safe_load(fh)


_POSES_LINE_RE = re.compile(r"^(\s*)poses_csv\s*:.*$", re.MULTILINE)


def _set_poses_csv_in_params(poses_csv_rel: str) -> None:
    """Rewrite only data.poses_csv in params.yaml, preserving formatting.

    A targeted text edit rather than a YAML round-trip — keeps the existing
    comments, blank lines, and quoting style intact.
    """
    text = PARAMS_FILE.read_text()
    replacement_value = f'"{poses_csv_rel}"'
    if _POSES_LINE_RE.search(text):
        new_text = _POSES_LINE_RE.sub(
            lambda m: f"{m.group(1)}poses_csv: {replacement_value}", text, count=1
        )
    else:
        # Insert after the line that says "data:" using its indentation + 2.
        data_re = re.compile(r"^(\s*)data\s*:\s*$", re.MULTILINE)
        m = data_re.search(text)
        if not m:
            raise HTTPException(status_code=500, detail="params.yaml has no 'data:' section")
        indent = m.group(1) + "  "
        insert = f"\n{indent}poses_csv: {replacement_value}"
        new_text = text[: m.end()] + insert + text[m.end():]
    PARAMS_FILE.write_text(new_text)


# --- API ------------------------------------------------------------------

class CalibrateRequest(BaseModel):
    dataset: Optional[str] = None  # subdir name under data/


@app.get("/api/datasets")
def list_datasets() -> dict:
    return {"datasets": _list_datasets()}


@app.get("/api/poses")
def get_poses(dataset: str) -> dict:
    poses_csv = DATA_DIR / dataset / "poses.csv"
    return _load_poses(poses_csv)


@app.post("/api/calibrate")
def run_calibration(req: CalibrateRequest) -> JSONResponse:
    if not BINARY.exists():
        raise HTTPException(
            status_code=500,
            detail=f"Binary missing at {BINARY}. Build with: cmake --build build",
        )
    if req.dataset:
        poses_csv = DATA_DIR / req.dataset / "poses.csv"
        if not poses_csv.exists():
            raise HTTPException(status_code=404, detail=f"Dataset has no poses.csv: {req.dataset}")
        _set_poses_csv_in_params(str(poses_csv.relative_to(PROJECT_ROOT)))

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
    return JSONResponse({"stdout": proc.stdout, "stderr": proc.stderr, "result": _read_result()})


@app.get("/api/result")
def get_result() -> dict:
    return _read_result()


@app.post("/api/upload")
async def upload_dataset(file: UploadFile = File(...)) -> dict:
    """Accept a poses.csv file and save it under data/uploads/<name>/poses.csv."""
    if not file.filename:
        raise HTTPException(status_code=400, detail="No filename")
    # Derive a safe folder name from the upload filename (sans extension).
    stem = Path(file.filename).stem or "upload"
    safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in stem)
    dest_dir = UPLOADS_DIR / safe
    dest_dir.mkdir(parents=True, exist_ok=True)
    dest = dest_dir / "poses.csv"
    with dest.open("wb") as fh:
        shutil.copyfileobj(file.file, fh)

    # Validate it parses
    try:
        info = _load_poses(dest)
    except HTTPException:
        dest.unlink(missing_ok=True)
        raise

    return {
        "name": f"uploads/{safe}",
        "poses_csv": str(dest.relative_to(PROJECT_ROOT)),
        "num_poses": info["num_poses"],
    }


app.mount("/", StaticFiles(directory=str(STATIC_DIR), html=True), name="static")
