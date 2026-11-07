"""Record the visible MuJoCo window to an animated GIF on Windows."""

import argparse
import ctypes
from ctypes import wintypes
from pathlib import Path
import time

from PIL import ImageGrab


def find_window():
    """Find the visible MuJoCo window.

    Returns:
        Windows window handle for MuJoCo, or zero if no window is visible.
    """
    user = ctypes.windll.user32
    matches = []

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    def collect(hwnd, _parameter):
        """Collect a matching visible window.

        Args:
            hwnd: Enumerated Windows window handle.
            _parameter: Unused enumeration argument.

        Returns:
            True to continue enumeration.
        """
        title = ctypes.create_unicode_buffer(512)
        user.GetWindowTextW(hwnd, title, len(title))
        if user.IsWindowVisible(hwnd) and title.value.startswith("MuJoCo : arachne"):
            matches.append(hwnd)
        return True

    user.EnumWindows(collect, 0)
    return matches[0] if matches else 0


def record(output, stop_file, duration, fps):
    """Capture the visible window until requested to stop and write a GIF.

    Args:
        output: Path of the output GIF.
        stop_file: File whose appearance ends capture.
        duration: Maximum capture time in seconds.
        fps: Requested capture frequency.

    Returns:
        Number of captured frames.
    """
    user = ctypes.windll.user32
    user.SetProcessDPIAware()
    hwnd = find_window()
    if not hwnd:
        raise RuntimeError("MuJoCo window not found")
    user.ShowWindow(hwnd, 9)
    user.SetForegroundWindow(hwnd)
    time.sleep(1)
    rectangle = wintypes.RECT()
    user.GetWindowRect(hwnd, ctypes.byref(rectangle))
    box = (rectangle.left, rectangle.top, rectangle.right, rectangle.bottom)
    frames = []
    start = time.monotonic()
    while time.monotonic() - start < duration and not stop_file.exists():
        tick = time.monotonic()
        frame = ImageGrab.grab(bbox=box, all_screens=True).convert("RGB")
        frame.thumbnail((720, 540))
        frames.append(frame.convert("P", palette=1, colors=128))
        time.sleep(max(0.0, 1.0 / fps - (time.monotonic() - tick)))
    if not frames:
        raise RuntimeError("no frames captured")
    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(output, save_all=True, append_images=frames[1:],
                   duration=round(1000 / fps), loop=0, optimize=False)
    return len(frames)


def main():
    """Read recording options and print the output path and frame count.

    Returns:
        None.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("stop_file", type=Path)
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--fps", type=float, default=3.0)
    args = parser.parse_args()
    print(f"Saved {record(args.output, args.stop_file, args.duration, args.fps)} frames to {args.output}")


if __name__ == "__main__":
    main()
