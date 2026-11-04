"""The solver IPC carries full image bytes without JSON/base64 expansion."""
import io
import struct

import pytest

pytest.importorskip('cv2')
from space_arm_platform.vision_worker import read_pairs


def test_binary_pairs_preserve_pixels_bytes_and_do_not_mix_frames():
    encoded = b''.join(struct.pack('!II', len(l), len(r))+l+r
                       for l, r in [(b'left\x00\n', b'right\xff'), (b'', b'')])
    assert list(read_pairs(io.BytesIO(encoded), True)) == [[b'left\x00\n', b'right\xff'], [b'', b'']]


@pytest.mark.parametrize('data', [b'bad', struct.pack('!II', 3, 4)+b'left', struct.pack('!II', 100_000_000, 0)])
def test_binary_truncation_and_oversized_images_fail_explicitly(data):
    with pytest.raises(ValueError):
        list(read_pairs(io.BytesIO(data), True))
