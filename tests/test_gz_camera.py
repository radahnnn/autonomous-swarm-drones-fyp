import pytest
from PIL import Image

from sitl.gz_camera import frame_to_png


def test_rgb_frame_round_trips(tmp_path):
    width, height = 4, 2
    data = bytes([255, 0, 0] * 4 + [0, 0, 255] * 4)
    path = frame_to_png(width, height, "R8G8B8", data, tmp_path / "a" / "x.png")
    image = Image.open(path)
    assert image.size == (4, 2) and image.getpixel((0, 0)) == (255, 0, 0) and image.getpixel((0, 1)) == (0, 0, 255)


def test_bgr_is_converted(tmp_path):
    path = frame_to_png(1, 1, "B8G8R8", bytes([0, 0, 255]), tmp_path / "b.png")      # BGR red
    assert Image.open(path).getpixel((0, 0)) == (255, 0, 0)


@pytest.mark.parametrize("fmt, w, h, data", [("YUV", 2, 2, b"x" * 12), ("R8G8B8", 0, 2, b""), ("R8G8B8", 2, 2, b"short")])
def test_bad_frames_are_rejected(tmp_path, fmt, w, h, data):
    with pytest.raises(ValueError):
        frame_to_png(w, h, fmt, data, tmp_path / "c.png")
    assert not (tmp_path / "c.png").exists()
