# mocap/python/corner_jitter.py
"""
Measures how much detected ChArUco corners wobble between frames of a
static board. Lower is better. Gain amplifies signal and sensor noise
alike, so this tells us whether the extra brightness costs precision.

Usage: python corner_jitter.py <session_dir> [<session_dir> ...]
"""
import sys, glob, os
import cv2
import numpy as np

SQUARES_X, SQUARES_Y = 9, 6
SQUARE_MM, MARKER_MM = 90.0, 67.0

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_MM, MARKER_MM, dictionary)
board.setLegacyPattern(True)          # see finding 4.7 - without this, zero corners
detector = cv2.aruco.CharucoDetector(board)


def corners_by_id(path):
    """Return {corner_id: (x, y)} for one image."""
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    corners, ids, _, _ = detector.detectBoard(img)
    if ids is None:
        return {}
    return {int(i): np.asarray(c, dtype=float).reshape(-1)[:2]
            for i, c in zip(ids.flatten(), corners)}


def analyze(session):
    files = sorted(glob.glob(os.path.join(session, "*.png")))
    if not files:
        print(f"{session}: no PNGs")
        return

    observations = [corners_by_id(f) for f in files]
    counts = [len(o) for o in observations]

    # Only corners seen in EVERY frame are comparable - a corner that
    # drops in and out would otherwise contaminate the spread.
    common = set(observations[0])
    for o in observations[1:]:
        common &= set(o)

    if len(common) < 4:
        print(f"{session}: only {len(common)} corners in all frames, not enough")
        return

    # Per corner: std dev of its position across frames, in pixels.
    stds = []
    for cid in sorted(common):
        pts = np.array([o[cid] for o in observations])
        stds.append(np.hypot(pts[:, 0].std(), pts[:, 1].std()))
    stds = np.array(stds)

    print(f"\n{os.path.basename(session)}")
    print(f"  frames            {len(files)}")
    print(f"  corners detected  min {min(counts)}, max {max(counts)}, of 40")
    print(f"  corners compared  {len(common)}")
    print(f"  jitter mean       {stds.mean():.4f} px")
    print(f"  jitter median     {np.median(stds):.4f} px")
    print(f"  jitter worst      {stds.max():.4f} px")


if __name__ == "__main__":
    for s in sys.argv[1:]:
        analyze(s)