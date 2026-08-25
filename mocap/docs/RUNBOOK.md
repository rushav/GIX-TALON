# Mocap intrinsics runbook

Every command needed to calibrate one OptiTrack Prime 13W, in order. Written to
be followed start to finish without prior knowledge of this system.

For camera serials, model specs and network layout see
[../../docs/hardware/cameras.md](../../docs/hardware/cameras.md).
For what was measured on this hardware and why the settings are what they are,
see [FINDINGS.md](FINDINGS.md).

All paths below are relative to the repo root unless stated otherwise.

---

## 0. One-time setup

### 0.1 System packages

```sh
sudo apt install build-essential cmake qt6-base-dev libopencv-dev \
                 libyaml-cpp-dev python3.12-venv tcpdump
```

- `qt6-base-dev` — the GUI. Qt5 will not satisfy the build.
- `libopencv-dev` — OpenCV 4.6, used by the C++ side for live detection.
- `libyaml-cpp-dev` — config parsing.
- `python3.12-venv` — **not installed by default on Ubuntu 24.04**, and
  `python3 -m venv` fails with a confusing error without it.
- `tcpdump` — for the network check in section 4.

### 0.2 Python venv

Ubuntu 24.04 marks the system Python as externally managed (PEP 668), so
`pip install` outside a venv is refused. A venv is required, not preferred.

It also solves a second problem: `opencv-contrib-python` pulls in numpy 2.x,
and a system-wide numpy 2.x shadows the numpy 1.x that apt's `python3-matplotlib`
and `python3-scipy` were built against, breaking both. Inside a venv the two
never meet.

```sh
python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install "opencv-contrib-python>=5.0" numpy
```

Verify — this must print 5.x and `True`:

```sh
.venv/bin/python -c "import cv2; print(cv2.__version__, hasattr(cv2.aruco,'CharucoDetector'))"
```

The solve needs OpenCV 5.x specifically: OpenCV 5 removed
`aruco.calibrateCameraCharuco` and `aruco.interpolateCornersCharuco`, and the
replacement `CharucoDetector` + `matchImagePoints` path is what
`solve_intrinsics.py` uses. 5.x cannot be loaded into the capture tool's
process, which is already linked against the system OpenCV 4.6 — that is why
the solve runs out of process.

### 0.3 OptiTrack Camera SDK

Expected at `~/optitrack/CameraSDK`. Override with
`-DCAMERA_SDK_PATH=/path/to/CameraSDK` or the `CAMERA_SDK_PATH` environment
variable.

---

## 1. Build

```sh
cd mocap/src/intrinsics_capture
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

## 2. setcap — after **every** build

The camera network needs raw socket access.

```sh
sudo setcap cap_net_raw,cap_net_admin+eip ./build/intrinsics_capture
```

or equivalently:

```sh
cmake --build build --target setcap
```

**Linking clears file capabilities.** Any rebuild that relinks the binary drops
them and you must set them again. The build prints a reminder. To check:

```sh
getcap ./build/intrinsics_capture      # empty output means none are set
```

Do **not** run the tool under `sudo`. It breaks desktop audio for the session
and writes root-owned PNGs into `mocap/sessions/`, which then have to be
chowned before the solve can read them.

---

## 3. Check the camera network

```sh
ip -brief link show enp6s0                    # want: UP ... LOWER_UP
sudo tcpdump -i enp6s0 -n port 13013          # want: packets from 169.254.x.x
```

`NO-CARRIER` means nothing is plugged in or the cameras are unpowered. Silence
on 13013 with the link up means the cameras are not broadcasting — power-cycle
the PoE switch.

Then ask the tool what it can see:

```sh
./build/intrinsics_capture --list
```

Enumeration is staggered and takes 4–8 seconds cold. A serial listed twice is
normal and is collapsed automatically.

---

## 4. Run the tool

```sh
cd mocap/src/intrinsics_capture
./build/intrinsics_capture                  # first camera with status: active
./build/intrinsics_capture --serial 33661   # a specific camera
```

`--serial` avoids editing `mocap/config/cameras.yaml` to switch cameras. The
serial must be listed there and not marked `status: excluded`.

The tool opens with the verified working settings from
`mocap/config/intrinsics_capture.yaml`:

| Setting | Value |
|---------|-------|
| IR bandpass filter | **IN** (checked) |
| IR intensity | 15 |
| Imager gain | 7 |
| Exposure | 8133 µs |
| Frame rate | 30 Hz |

You should see 27/27 markers and 40/40 corners on the board at ~1–1.5 m.

The live view delivers 6–8 frames per second. That is normal — see
[FINDINGS.md](FINDINGS.md#3-grayscale-delivery-is-bandwidth-bound-6-8-fps).

---

## 5. Capture procedure

Board at **1–1.5 m**. Further than that and the board fills too little of the
frame, marker count starts fluctuating, and corners cluster mid-frame — which
is the narrow-data failure this whole procedure exists to avoid.

Press **Space** (or click *Capture frame*) for each view. Watch two things as
you go:

**The Coverage tab.** Every cell of the 5×6 grid must end up non-zero. Red
cells are regions where no board corner has ever landed; the distortion model
will extrapolate across them and **RMS will not warn you**. Work the board into
the corners and edges of the frame deliberately — that is what fills the outer
cells.

**The tilt histogram, same tab.** At least 30% of captures should exceed 20° of
tilt. All-fronto-parallel views leave focal length and distance mutually
ambiguous; the symptom afterwards is fx drifting away from fy.

A workable pattern, ~30–40 captures:

1. Nine positions — centre, four edges, four corners — roughly fronto-parallel.
2. The same nine again, tilted 20–40° about the vertical axis.
3. The same nine again, tilted 20–40° about the horizontal axis.
4. A few at varied roll and distance to fill whatever cells are still thin.

Check the Captures tab as you go. Any capture can be deleted there (select,
then *Delete selected capture* or the Delete key); deleting also removes its
contribution from the coverage grid and the tilt histogram.

Frames land in `mocap/sessions/intrinsics_<serial>_<timestamp>/` with a
`session.json` recording camera settings per frame. Timestamps are UTC.

---

## 6. Solve

Press **Process — solve intrinsics** on the Solve tab. It shells out to the
venv and writes `intrinsics.json` into the session folder.

The same thing from a terminal:

```sh
.venv/bin/python mocap/python/solve_intrinsics.py \
    mocap/sessions/intrinsics_33661_<timestamp>
```

Useful flags: `--exclude frame_0007.png frame_0012.png` to drop views without
deleting them, `--min-corners N` to change the per-frame corner floor.

### Reading the result

The panel colours values against the ranges the previous rig settled into.
**These are context, not pass/fail** — a value outside them is worth a look,
not a failure.

| Quantity | Previous rig |
|----------|--------------|
| RMS | 0.32–0.39 px |
| fx, fy | 778–786 px, agreeing within 1% |
| cx | 627–637 |
| cy | 516–521 |

Then check, in this order:

1. **Coverage warning.** If the panel says cells were empty, the result is
   under-determined at the frame edges regardless of how good RMS looks.
   Recapture, filling those cells.
2. **fx vs fy.** More than ~1% apart points at insufficient tilt variety.
3. **Per-view error, worst first.** The three worst views are highlighted in
   the Captures tab. Delete a genuinely bad one and solve again.

Distortion is the 8-coefficient rational model. Its coefficients are a ratio of
two polynomials and are **not** individually interpretable — large values with
k1≈k4 and k2≈k5 are normal. Judge the model by undistortion behaviour, not by
the numbers.

---

## 7. Troubleshooting

| Symptom | Cause |
|---------|-------|
| No cameras found | setcap missing after a relink; or link down — section 3 |
| Hangs on start right after a previous run | SDK sockets not yet released; wait a couple of seconds |
| Live view very dark | IR intensity or exposure reset; check the Camera tab readbacks |
| Exposure slider will not go higher | The ceiling tracks the frame period. Lower the frame rate and the bound moves — the tool re-reads it automatically |
| Markers detected but 0 corners | Board partly out of frame, or too far — get closer than 1.5 m |
| Solve button disabled | Fewer than 3 captures |
| "no interpreter at .venv/bin/python" | Section 0.2 |
| Solve reports 0 corners on every frame | `legacy_pattern` is false in `mocap/config/charuco_board.yaml` — see FINDINGS |
