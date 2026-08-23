# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — VLA client backend B: REMOTE over HTTP (opt-in, self-hosted).

Same `act(image, instruction, state)` signature as LocalSmolVLA, but the model
runs on a server we stand up (the 4090 box on the LAN, or a cloud GPU). There is
NO public OpenVLA/SmolVLA API — "remote" always means infrastructure we host
(see LOCAL_VS_REMOTE_VLA.md). The server (server.py) already returns the canonical
contract, so this client just POSTs the inputs and rebuilds an Action.

Used for the advanced local-vs-remote benchmark in Lab 2.2. The default path
never touches it.
"""

from __future__ import annotations

import base64

import numpy as np

from .base import Action


class RemoteVLA:
    """HTTP backend. Implements the VLAClient protocol."""

    def __init__(self, url: str, timeout_s: float = 2.0):
        if not url:
            raise ValueError(
                'RemoteVLA needs a URL. Set VLA_REMOTE_URL, e.g. http://gpu-box.lan:8000'
            )
        # Lazy import so the package loads without `requests` on a minimal box.
        try:
            import requests
        except ImportError as e:
            raise ImportError('RemoteVLA needs `requests` (pip install requests)') from e
        self._requests = requests
        self.url = url.rstrip('/')
        self.timeout_s = timeout_s

    def act(self, image, instruction: str, state=None) -> Action:
        """POST {image, instruction, state} to the server, return its Action."""
        img = np.asarray(image, dtype=np.uint8)
        # Send the image as raw base64 + shape so the server reconstructs it
        # without guessing — simple and dependency-free on both ends.
        payload = {
            'image_b64': base64.b64encode(img.tobytes()).decode('ascii'),
            'image_shape': list(img.shape),
            'instruction': instruction,
            'state': None if state is None else [float(v) for v in np.asarray(state).reshape(-1)],
        }
        r = self._requests.post(f'{self.url}/act', json=payload, timeout=self.timeout_s)
        r.raise_for_status()
        return Action.from_dict(r.json())   # server already normalized to canonical
