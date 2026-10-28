from __future__ import annotations

import logging
import threading
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from embodied_nexus_camera.adapters.gemini335.capture import Gemini335Capture
from embodied_nexus_camera.config import load_config


class GeminiAdapterTests(unittest.TestCase):
    """Verify device capture delivers frames through the common callback."""

    def test_color_capture_delivers_frame_and_releases_device(self):
        """A decoded color frame is passed with metadata and the device closes.

        Returns:
            None.
        """
        config = load_config(None)
        stop = threading.Event()
        frame = np.zeros((2, 2, 3), dtype=np.uint8)
        received = []

        def accept(image, stamp, frame_id, source):
            """Record a capture and end the loop.

            Args:
                image: Decoded image array.
                stamp: Host timestamp.
                frame_id: Configured image frame.
                source: Timestamp source label.

            Returns:
                None.
            """
            received.append((image, stamp, frame_id, source))
            stop.set()

        capture = Gemini335Capture(config, stop, MagicMock(), accept, MagicMock(), logging.getLogger(__name__))
        device = MagicMock()
        device.isOpened.return_value = True
        device.read.return_value = (True, frame)
        device.get.return_value = 2
        with patch.object(capture, "_apply_v4l2_controls"), \
                patch("embodied_nexus_camera.adapters.gemini335.capture.cv2.VideoCapture", return_value=device):
            capture.capture_color()
        self.assertEqual(len(received), 1)
        self.assertIs(received[0][0], frame)
        self.assertEqual(received[0][2], config["capture"]["color_frame_id"])
        self.assertEqual(received[0][3], "host_receive")
        device.release.assert_called_once()
