"""Display one saved image briefly in a process isolated from the ROS node."""

import argparse
import os
import sys
import time


def show_image(
    image_path: str,
    duration_sec: float,
    window_name: str,
    qt_platform: str = "xcb",
) -> int:
    """Open a saved image, then close the window after ``duration_sec``."""
    if duration_sec <= 0.0:
        return 0

    if qt_platform:
        os.environ["QT_QPA_PLATFORM"] = qt_platform

    import cv2

    image = cv2.imread(image_path)
    if image is None:
        return 1

    window_created = False
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        window_created = True
        cv2.imshow(window_name, image)
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            key_code = cv2.waitKey(25) & 0xFF
            if key_code in (27, ord("q")):
                break
        return 0
    except cv2.error:
        return 1
    finally:
        if window_created:
            try:
                cv2.destroyWindow(window_name)
            except cv2.error:
                pass


def main(argv=None) -> int:
    """Parse command-line arguments and run the short-lived image viewer."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image_path")
    parser.add_argument("--duration", type=float, default=2.5)
    parser.add_argument("--window-name", default="LLM Camera")
    parser.add_argument("--qt-platform", default="xcb")
    args = parser.parse_args(argv)
    return show_image(
        args.image_path,
        args.duration,
        args.window_name,
        args.qt_platform,
    )


if __name__ == "__main__":
    sys.exit(main())

