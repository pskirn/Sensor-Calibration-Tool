# Calibration UI

Minimal web UI for the camera-LiDAR calibration pipeline. FastAPI backend
serves a single static HTML page that runs the C++ binary and visualises
the result.

## Setup (once)

```bash
cd ui
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

Make sure the C++ binary is built first:

```bash
cmake --build build -j$(nproc)
```

Then start the server from the **project root** (not from `ui/`):

```bash
uvicorn ui.app:app --reload --port 8000
```

Open <http://localhost:8000> in a browser. Click **Run Calibration**.

## How it works

- Frontend (`static/index.html` + `static/main.js`) is plain HTML + JS. No
  build step. Chart.js is loaded from a CDN.
- Backend (`app.py`) shells out to `build/camera_lidar_calibration`, reads
  the YAML it writes to `results/calibration.yaml`, and returns it as
  JSON.
- The C++ pipeline still reads its inputs from `config/params.yaml` as
  usual; the UI doesn't change the calibration logic, it just drives it.
