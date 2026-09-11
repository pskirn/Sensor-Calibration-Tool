# UI internals

User-facing docs live in the [project README](../README.md). This file covers how
the code here is put together.

Run the server from the **project root**, not from `ui/`:

```bash
ui/.venv/bin/python -m uvicorn ui.app:app --reload --port 8000
```

## Layout

```
app.py            FastAPI: dataset listing, upload, solving, /api/health
live_api.py       FastAPI: live capture endpoints (/api/live/*)
sensors/          vendor-neutral input layer — see sensors/base.py for rationale
live/             detection, plane fitting, quality metrics, capture session
static/           frontend; plain ES modules, no build step
```

## Design notes

**The solver is never reimplemented.** Both the offline and live paths end at the
same `build/camera_lidar_calibration` binary, driven through the same
`run_calibration()` function. Live capture's only job is to produce a `poses.csv`
in the 19-line layout `src/poses_csv_loader.cpp` reads. Any change to the maths
happens in C++ and both paths inherit it.

**Sensor adapters translate transport, not semantics.** Everything upstream of
`sensors/base.py` deals in `CloudFrame` (an `(N,3)` numpy array) and `ImageFrame`
(a BGR array). Adding a sensor means adding an adapter, never touching detection
or solving.

**Detection runs in Python, not through the C++ binary.** Preview frames need
interactive rates, and shelling out per frame would not give that. The Python
detectors in `live/` mirror what `src/*_detector.cpp` does; they feed capture and
preview, while the final solve stays in C++.

**One session at a time.** `live_api` holds a single module-level `LiveSession`.
The tool drives physical devices, and two sessions competing for one camera fail
in confusing ways, so the constraint is explicit — starting a second returns 409.

## Frontend

ES modules loaded directly by the browser; three.js and Chart.js come from a CDN
via an import map.

```
app.js       shell: mode switching, env banner, fetch helpers, toasts
live.js      live capture: source picker, streaming, capture loop, meters
offline.js   dataset mode: pick/upload, 3D inspect, solve
results.js   result rendering shared by both modes
viewer.js    PlaneViewer (poses as quads) and CloudViewer (live cloud + ROI)
```

The camera preview is MJPEG (`/api/live/preview.mjpg`) rather than WebSockets:
an `<img src>` renders it with no client code, and frames are already
JPEG-encoded server-side for the overlay.

## Tests

```bash
ui/.venv/bin/python tools/validate_live_pipeline.py --poses 12
```

Runs the simulated rig through detection, capture, CSV export and the C++ solver,
then checks the recovered extrinsic against the known ground truth. Exits
non-zero on regression, so it works as a CI gate.
