# intrinsics_capture

Qt6 tool for capturing ChArUco calibration images from a single OptiTrack
Prime 13W over the camera network.

Acquisition, capture guidance, and the calibration solve.

Team documentation lives in [../../docs/RUNBOOK.md](../../docs/RUNBOOK.md)
(every command, in order) and [../../docs/FINDINGS.md](../../docs/FINDINGS.md)
(what was measured on this hardware and why the settings are what they are).
This file covers the tool's own build and internals.

## What it does

- Connects to one camera, selected by serial from `mocap/config/cameras.yaml`.
- Puts it in grayscale video mode (not object mode) so the printed board is
  actually visible.
- Live view at native 1280x1024, scaled to fit the window.
- Runs `cv::aruco::detectMarkers` on every displayed frame and overlays the
  detections with a live `n / 27` count.
- Sliders for exposure, imager gain, IR LED intensity and frame rate, each
  showing the value the camera reports back next to the one you asked for.
- A toggle for the 850 nm IR bandpass filter.
- Capture writes the current full-resolution frame as PNG into
  `mocap/sessions/intrinsics_<serial>_<timestamp>/`, alongside a `session.json`.
- **Coverage grid** — a live 5x6 map of accepted ChArUco corners per frame
  region, so gaps get filled during capture instead of discovered afterwards.
- **Tilt histogram** — board pose per capture, flagging an all-fronto-parallel
  set before it produces an ambiguous solve.
- **Capture list** — thumbnails with corner counts; deleting a capture unwinds
  it from the coverage grid and the tilt statistics too.
- **Process** — runs the calibration solve and displays fx/fy, cx/cy, RMS and
  per-view error with the worst views highlighted.

## Dependencies

| Component | Version used | Package |
|-----------|--------------|---------|
| Qt6       | 6.4.2        | `qt6-base-dev` |
| OpenCV    | 4.6.0        | `libopencv-dev` (aruco included) |
| yaml-cpp  | 0.8.0        | `libyaml-cpp-dev` |
| OptiTrack Camera SDK | 3.5.0 Beta1 | `~/optitrack/CameraSDK` |
| CMake     | 3.16+        | `cmake` |
| OpenCV (solve) | 5.x     | `opencv-contrib-python` in `.venv` |

Qt5 will not satisfy the build — `find_package(Qt6 REQUIRED)` is explicit.

The solve runs out of process in the repo venv. It needs OpenCV 5.x, which
removed `aruco.calibrateCameraCharuco` and `aruco.interpolateCornersCharuco` in
favour of `CharucoDetector` + `matchImagePoints`, and 5.x cannot be loaded into
a process already linked against the system OpenCV 4.6. The two talk JSON.

## Build

```sh
cd mocap/src/intrinsics_capture
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

The SDK is located at `$HOME/optitrack/CameraSDK` by default. Override with
either:

```sh
cmake -S . -B build -DCAMERA_SDK_PATH=/path/to/CameraSDK
# or
export CAMERA_SDK_PATH=/path/to/CameraSDK
```

## setcap — required after every build

The camera network needs raw socket access. Grant it with file capabilities,
**not** sudo:

```sh
sudo setcap cap_net_raw,cap_net_admin+eip ./build/intrinsics_capture
```

Or use the convenience target, which does the same thing:

```sh
cmake --build build --target setcap
```

**Linking clears file capabilities.** Every rebuild that relinks the binary
drops them and you have to set them again — the build prints a reminder. Check
the current state with:

```sh
getcap ./build/intrinsics_capture     # empty output means none set
```

Do not run this under `sudo`. It breaks desktop audio for the session and
writes root-owned PNGs and session folders into `mocap/sessions/`, which you
then have to chown before stage 2 can read them.

## Running

```sh
./build/intrinsics_capture                 # first camera with status: active
./build/intrinsics_capture --serial 33661  # a specific camera, no config edit needed
./build/intrinsics_capture --list          # enumerate what the SDK sees, then exit
./build/intrinsics_capture --repo-root /path/to/GIX-TALON
```

The solver can also be run by hand:

```sh
.venv/bin/python mocap/python/solve_intrinsics.py <session_dir> \
    [--exclude frame_0007.png] [--min-corners N] [--no-rational]
```

The repo root is found by searching upward for `mocap/config/cameras.yaml`,
starting from the working directory and then from the binary's location, so the
tool runs from anywhere in the tree.

`--serial` must name a camera listed in `cameras.yaml`. A serial marked
`status: excluded` is refused, with the reason from the inventory.

Press **Space** or click **Capture frame** to save a frame.

## Configuration

Nothing is hardcoded — no serials, no paths, no board geometry.

| File | Holds |
|------|-------|
| `mocap/config/cameras.yaml` | camera inventory, serials, native resolution |
| `mocap/config/charuco_board.yaml` | dictionary, square counts, physical sizes |
| `mocap/config/intrinsics_capture.yaml` | session root, camera defaults, discovery timing, coverage grid, pose thresholds, solver paths, reference bounds |

Board defaults are `DICT_5X5_1000`, 9x6 squares, 90 mm squares, 67 mm markers,
27 markers, 40 ChArUco corners.

The reference bounds under `reference:` are context for reading a solve result,
never pass/fail. They colour the numbers in the Solve tab and nothing else.

## Output

```
mocap/sessions/intrinsics_33661_20260824_231405/
├── frame_0001.png      # 1280x1024 8-bit, no overlay drawn on it
├── frame_0002.png
├── session.json        # capture manifest, rewritten after every capture/delete
└── intrinsics.json     # written by the solve
```

The session folder is created on the first capture, so quitting without
capturing leaves nothing behind. `session.json` is rewritten after every
capture, so an interrupted session still has a manifest matching the PNGs on
disk. All timestamps, including the one in the folder name, are UTC.

It records the camera serial and revision, SDK / OpenCV / Qt versions, the board
geometry, and per frame: requested *and* actual exposure, imager gain, IR
intensity, frame rate, IR filter state, the camera's frame id and timestamp, and
the marker ids that were detected.

It also records the coverage grid and per-frame ChArUco corner count and tilt,
so a solve result can be read next to the sampling that produced it.

The PNG is the exact image the detection count was computed from, so a frame and
its recorded marker count never disagree.

`intrinsics.json` carries fx/fy/cx/cy, all eight rational distortion
coefficients, overall RMS, per-view reprojection error, and provenance: OpenCV
version, timestamp, source session, board geometry and the camera settings each
frame was taken with.

Capture filenames are never reused. Deleting `frame_0003.png` leaves a gap
rather than renumbering, so a filename in `intrinsics.json` always refers to the
same image.

## SDK behaviours handled

These are all verified against a Prime 13W (revision 32) on this hardware, not
guesses. Each is commented at the point it is dealt with.

- **Staggered discovery.** Waiting for the first device returns almost
  immediately while a cold network takes 4-8 s to enumerate fully.
  `discover()` polls `GetCameraList()` and only accepts the result once the
  count has held steady for `discovery.settle_ms`, sleeping on the listener's
  condition variable so a late arrival cuts the wait short.
- **Duplicate list entries.** `CameraList` reported the same serial twice on
  this camera. Entries are collapsed by serial, preferring one that has reached
  `Initialized`.
- **`SetVideoType` is asynchronous.** `VideoType()` reports the new mode at
  once, but queued frames still carry the old one and `GrayscaleData()` on an
  object-mode frame returns null. The mode is set before `Start()` and
  reasserted after it; every frame's `FrameType()` is checked before its data is
  touched, and a long run of wrong-mode frames triggers another reassert.
  In practice one object-mode frame arrives and is dropped.
- **The frame queue must be drained.** `NextFrame()` returns the *oldest* queued
  frame, so one call per iteration falls further behind every pass. The capture
  loop drains to the end and keeps only the newest.
- **The exposure range moves with the frame rate.** The SDK re-reports
  `MinimumExposureValue()`/`MaximumExposureValue()` after every
  `SetFrameRate()`, and the maximum is `floor(1e6/fps) - 200` microseconds.
  The slider bounds are re-read on every rate change and after the startup
  defaults are applied; leaving them stale silently caps how bright the image
  can get. The panel still shows the value read back from the camera next to
  the request. Measured: 240 Hz -> 3966, 120 Hz -> 8133, 60 Hz -> 16466,
  30 Hz -> 33133.
- **Listener callbacks run on the SDK's discovery thread.** `CameraInitialized`
  and friends only tick a counter under our own mutex. They never call back into
  the SDK — re-entering the library from inside its own callback can deadlock.
  Every control setter records the request under the lock, releases it, and only
  then issues the SDK call.

## Module layout

| File | Role |
|------|------|
| `camera_source.*` | SDK lifecycle, discovery, capture thread, control readback |
| `board_detector.*` | marker detection plus ChArUco corner interpolation |
| `coverage_grid.*` | corners per frame region, add and unwind |
| `pose_estimator.*` | per-capture tilt via `solvePnP`, histogram, threshold |
| `capture_session.*` | capture records, PNG and manifest I/O, delete |
| `solver_runner.*` | `QProcess` wrapper around the venv solver, JSON parsing |
| `analysis_widgets.*` | coverage grid and tilt histogram painters |
| `capture_list.*` | thumbnail list with delete |
| `main_window.*` | layout and wiring |

## Notes from this hardware

Full detail with measurements in
[../../docs/FINDINGS.md](../../docs/FINDINGS.md). In brief:

- **The IR bandpass filter stays IN**, with the IR ring doing the illuminating
  (`ir_intensity: 15`, `imager_gain: 7`, `exposure: 8133`, 30 Hz). That gives
  27/27 markers and 40/40 corners at ~1.5 m, and means calibration happens in
  the same optical configuration the cameras track in — no filter-removal focal
  shift to account for. The filter is only workable together with the ring:
  filter in with the ring off gives a frame averaging 6/255 and zero markers.
- **Imager gain is free at this exposure.** Static-board corner jitter over 20
  frames: 0.0561 px median at gain 0, 0.0540 px at gain 7, 40/40 corners
  detected in every frame both ways.
- **Grayscale frames arrive at roughly 8-15 fps, not 120.** The imager really is
  running at 120 Hz — `ActualFrameRate()` confirms it — but full-resolution
  grayscale is delivered at about 10 MB/s, roughly 8 frames per second of
  1280x1024. `SetGrayscaleDecimation()` had no effect on this camera. This is a
  property of grayscale mode on the Prime 13W, not of this tool, and it is
  irrelevant for calibration capture, where the board is held still. The status
  line shows the imager rate and the delivered rate separately so the two are
  not confused.
- **Give the SDK a moment between runs.** Starting a second instance
  immediately after killing the first occasionally hangs in
  `CameraLibraryStartup()` while the previous process's sockets are still
  bound. A couple of seconds between runs avoids it.
- **Exposure readback echoes the request**, so treat the amber "clamped"
  indication as a hint rather than proof. The slider is bounded by the SDK's
  reported range for the current frame rate, and image brightness is the
  reliable check.
- **Working distance is 1-1.5 m.** Further out the board fills too little of
  the frame, marker count fluctuates, and corners cluster mid-frame.

## Why coverage is shown so prominently

RMS only measures error where corners were observed, so a set with whole frame
regions empty can report a healthy RMS while the distortion model extrapolates
across the gaps.

Measured on synthetic captures with known ground truth, solved with this repo's
own solver:

| Set | Empty cells | RMS | fx (truth 780) | Undistortion error, outer 20% |
|-----|------------:|----:|---------------:|------------------------------:|
| centre-clustered | 18 of 30 | 0.262 px | 781.3 | 55.96 px |
| edge-covered | 0 of 30 | 0.223 px | 780.0 | 26.82 px |

RMS does not separate them. Undistortion accuracy differs 10x overall. That gap
is what the coverage grid exists to close, and why the empty-cell count is
repeated next to the intrinsics in the Solve tab.
