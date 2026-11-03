"""Save frames from Gazebo camera topics as PNG files.

``frame_to_png`` is pure (bytes in, file out) and unit-tested. ``capture`` and ``capture_all`` subscribe to the
camera topics through gz-transport's Python bindings and need a running Gazebo world generated with cameras
(``sitl/generate_gazebo_world.py --camera``). The bindings are imported lazily so the rest of the project does
not depend on them.
"""
from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Dict, Optional

_FORMATS = {"R8G8B8": ("RGB", 3), "RGB_INT8": ("RGB", 3), "B8G8R8": ("BGR", 3), "L8": ("L", 1), "L_INT8": ("L", 1)}


def frame_to_png(width: int, height: int, pixel_format: str, data: bytes, path: Path) -> Path:
    """Write raw camera pixels to ``path``. Raises ValueError for an unsupported format or a short buffer."""
    from PIL import Image
    if pixel_format not in _FORMATS:
        raise ValueError(f"unsupported camera pixel format {pixel_format!r}")
    mode, channels = _FORMATS[pixel_format]
    if width <= 0 or height <= 0 or len(data) < width * height * channels:
        raise ValueError("camera frame is empty or shorter than its declared size")
    raw = bytes(data[: width * height * channels])
    image = Image.frombytes(mode if mode != "BGR" else "RGB", (width, height), raw,
                            *(("raw", "BGR") if mode == "BGR" else ()))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)
    return path


def capture_all(topics: Dict[int, str], directory: Path, label: str, timeout_s: float = 10.0) -> Dict[int, Optional[Path]]:
    """Save one frame from every topic (``{drone_number: topic}``). Returns the saved path per drone, None on timeout."""
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node

    node = Node()
    frames: Dict[int, object] = {}
    lock = threading.Lock()

    def make_callback(number: int):
        def callback(message):
            with lock:
                frames.setdefault(number, message)
        return callback

    callbacks = {number: make_callback(number) for number in topics}      # keep references alive
    for number, topic in topics.items():
        node.subscribe(Image, topic, callbacks[number])
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        with lock:
            if len(frames) == len(topics):
                break
        time.sleep(0.05)
    for topic in topics.values():
        node.unsubscribe(topic)
    saved: Dict[int, Optional[Path]] = {}
    for number in topics:
        message = frames.get(number)
        if message is None:
            saved[number] = None
            continue
        pixel_format = {1: "L8", 3: "R8G8B8", 8: "B8G8R8"}.get(int(message.pixel_format_type), "R8G8B8")
        saved[number] = frame_to_png(message.width, message.height, pixel_format, message.data,
                                     Path(directory) / f"{label}_D{number}.png")
    return saved
