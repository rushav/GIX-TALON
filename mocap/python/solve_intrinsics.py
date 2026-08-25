#!/usr/bin/env python3
"""
Solve camera intrinsics from a ChArUco capture session.

Reads a session folder produced by mocap/src/intrinsics_capture (PNGs plus
session.json), runs the ChArUco calibration, and writes intrinsics.json beside
the images. Prints the same JSON to stdout so the Qt tool can read it back.

This lives in Python because it needs opencv-contrib-python 5.x: OpenCV 5
removed aruco.calibrateCameraCharuco and aruco.interpolateCornersCharuco in
favour of CharucoDetector + board.matchImagePoints, and 5.x cannot coexist with
the system OpenCV 4.6 the C++ side links against.

Usage:
    solve_intrinsics.py <session_dir> [--output PATH] [--min-corners N]
                        [--no-rational] [--exclude frame_0003.png ...]
"""
import argparse
import datetime
import json
import os
import platform
import sys

import cv2
import numpy as np


def log(msg):
    """Progress goes to stderr; stdout carries only the result JSON."""
    print(msg, file=sys.stderr, flush=True)


def build_board(geom):
    """Rebuild the physical board from the geometry recorded in session.json."""
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, geom["dictionary"]))
    board = cv2.aruco.CharucoBoard(
        (geom["squares_x"], geom["squares_y"]),
        float(geom["square_length_mm"]),
        float(geom["marker_length_mm"]),
        dictionary,
    )
    # Required even though the board was generated with calib.io's "ChArUco
    # Legacy" checkbox UNCHECKED. OpenCV changed its internal chessboard origin
    # convention; without this detectBoard returns zero corners on boards that
    # detectMarkers reads happily (26 of 27 markers found, 0 corners matched).
    if geom.get("legacy_pattern", True):
        board.setLegacyPattern(True)
    return board


def collect_observations(session_dir, manifest, board, min_corners, exclude):
    """Detect ChArUco corners in every frame. Returns (kept, skipped)."""
    detector = cv2.aruco.CharucoDetector(board)
    kept, skipped = [], []

    for entry in manifest.get("frames", []):
        name = entry["file"]
        path = os.path.join(session_dir, name)

        if name in exclude:
            skipped.append({"file": name, "reason": "excluded on the command line"})
            continue
        if not os.path.exists(path):
            skipped.append({"file": name, "reason": "file missing"})
            continue

        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            skipped.append({"file": name, "reason": "unreadable image"})
            continue

        corners, ids, _, _ = detector.detectBoard(image)
        count = 0 if ids is None else len(ids)
        if count < min_corners:
            skipped.append({"file": name,
                            "reason": f"only {count} corners, need {min_corners}"})
            continue

        # matchImagePoints pairs each detected corner with its 3D board
        # coordinate; it replaces interpolateCornersCharuco from OpenCV 4.x.
        object_points, image_points = board.matchImagePoints(corners, ids)
        if object_points is None or len(object_points) < min_corners:
            skipped.append({"file": name, "reason": "corner/board match failed"})
            continue

        kept.append({
            "file": name,
            "entry": entry,
            "corners": int(count),
            "object_points": object_points,
            "image_points": image_points,
            "size": (image.shape[1], image.shape[0]),
        })
        log(f"  {name}: {count} corners")

    return kept, skipped


def per_view_errors(kept, K, dist, rvecs, tvecs):
    """RMS reprojection error for each view, in pixels."""
    errors = []
    for obs, rvec, tvec in zip(kept, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obs["object_points"], rvec, tvec, K, dist)
        diff = projected.reshape(-1, 2) - obs["image_points"].reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))))
    return errors


def carry_settings(entry):
    """Per-frame camera settings, carried through for provenance."""
    keys = ("exposure_requested", "exposure_actual",
            "imager_gain_requested", "imager_gain_actual",
            "ir_intensity_requested", "ir_intensity_actual",
            "frame_rate_requested", "frame_rate_actual", "imager_frame_rate",
            "ir_filter_requested", "ir_filter_actual",
            "camera_frame_id", "camera_timestamp", "captured_utc")
    return {k: entry[k] for k in keys if k in entry}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir")
    ap.add_argument("--output", default=None,
                    help="default: <session_dir>/intrinsics.json")
    ap.add_argument("--min-corners", type=int, default=6,
                    help="skip a frame with fewer matched corners (default 6)")
    ap.add_argument("--no-rational", action="store_true",
                    help="use the 5-parameter model instead of the 8-parameter one")
    ap.add_argument("--exclude", nargs="*", default=[],
                    help="frame filenames to leave out of the solve")
    args = ap.parse_args()

    session_dir = os.path.abspath(args.session_dir)
    manifest_path = os.path.join(session_dir, "session.json")
    if not os.path.exists(manifest_path):
        log(f"error: no session.json in {session_dir}")
        return 2
    with open(manifest_path) as fh:
        manifest = json.load(fh)

    geom = manifest["board"]
    board = build_board(geom)
    log(f"board: {geom['dictionary']} {geom['squares_x']}x{geom['squares_y']} "
        f"{geom['square_length_mm']}mm/{geom['marker_length_mm']}mm")

    kept, skipped = collect_observations(
        session_dir, manifest, board, args.min_corners, set(args.exclude))
    for s in skipped:
        log(f"  skipped {s['file']}: {s['reason']}")

    if len(kept) < 3:
        log(f"error: only {len(kept)} usable views, need at least 3")
        return 3

    image_size = kept[0]["size"]

    # The 8-parameter rational model. The 5-parameter Brown-Conrady model
    # cannot represent the radial falloff of this 82 degree lens near the frame
    # edge - it fits the centre and extrapolates the corners.
    flags = 0 if args.no_rational else cv2.CALIB_RATIONAL_MODEL

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        [o["object_points"] for o in kept],
        [o["image_points"] for o in kept],
        image_size, None, None, flags=flags)

    # calibrateCamera returns a 14-slot vector (rational + thin-prism + tilted).
    # The rational model only uses the first 8; the rest come back zero. Report
    # those 8 as the coefficients and keep the raw vector for exactness.
    dist_raw = np.asarray(dist).reshape(-1)
    dist = dist_raw[:8] if not args.no_rational else dist_raw[:5]
    errors = per_view_errors(kept, K, dist, rvecs, tvecs)

    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    names = ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"]

    result = {
        "tool": "solve_intrinsics",
        "solved_utc": datetime.datetime.now(datetime.timezone.utc)
                              .strftime("%Y-%m-%dT%H:%M:%SZ"),
        "source_session": session_dir,
        "software": {
            "opencv_version": cv2.__version__,
            "numpy_version": np.__version__,
            "python_version": platform.python_version(),
        },
        "board": geom,
        "solver": {
            "legacy_pattern": bool(geom.get("legacy_pattern", True)),
            "distortion_model": "rational" if not args.no_rational else "brown-conrady",
            "flags": ["CALIB_RATIONAL_MODEL"] if not args.no_rational else [],
            "min_corners": args.min_corners,
            "views_used": len(kept),
            "views_skipped": len(skipped),
        },
        "image_size": list(image_size),
        "camera_matrix": [[float(v) for v in row] for row in K],
        "intrinsics": {
            "fx": fx, "fy": fy, "cx": cx, "cy": cy,
            # Focal drift is the tell for an all-fronto-parallel capture set:
            # with no tilt, focal length and distance trade off freely.
            "fx_fy_percent_diff": abs(fx - fy) / ((fx + fy) / 2.0) * 100.0,
        },
        "distortion": {
            "model": "rational" if not args.no_rational else "brown-conrady",
            "coefficients": [float(v) for v in dist],
            "coefficients_raw": [float(v) for v in dist_raw],
            **{n: float(dist[i]) for i, n in enumerate(names) if i < len(dist)},
        },
        "rms": float(rms),
        "views": [
            {
                "file": o["file"],
                "corners": o["corners"],
                "reprojection_error_px": e,
                "settings": carry_settings(o["entry"]),
            }
            for o, e in zip(kept, errors)
        ],
        "skipped": skipped,
    }

    output = args.output or os.path.join(session_dir, "intrinsics.json")
    with open(output, "w") as fh:
        json.dump(result, fh, indent=2)
        fh.write("\n")
    log(f"wrote {output}")

    json.dump(result, sys.stdout, indent=2)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
