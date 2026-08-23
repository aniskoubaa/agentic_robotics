# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — VLA policy server: host a checkpoint behind POST /act.

This is the "remote" half of LOCAL_VS_REMOTE_VLA.md. Run it on the GPU box (LAN)
or a cloud GPU; the RemoteVLA client (remote_vla.py) talks to it. It loads a
LocalSmolVLA in-process and exposes ONE endpoint that already returns the
canonical Action contract, so every client gets the same action language no
matter which model is hosted.

    POST /act  {image_b64, image_shape, instruction, state?}  ->  {joints[6], gripper, base?}
    GET  /healthz                                             ->  {"ok": true, ...}

Run (on the server, in the lerobot env):
    pip install fastapi uvicorn          # plus lerobot[smolvla] for the policy
    VLA_LOCAL_CKPT=outputs/smolvla_A python -m vla_client.server --port 8000

All heavy imports are lazy/local so importing the package never requires FastAPI.
"""

from __future__ import annotations

import argparse
import base64
import os

import numpy as np


def build_app():
    """Construct the FastAPI app. Imported lazily so the package stays light."""
    from fastapi import FastAPI
    from pydantic import BaseModel

    from .local_smolvla import LocalSmolVLA

    class ActRequest(BaseModel):
        image_b64: str
        image_shape: list[int]
        instruction: str = ''
        state: list[float] | None = None

    app = FastAPI(title='RAISE 2026 VLA server')

    # Load once at startup — this is the model the whole class will hit.
    ckpt = os.getenv('VLA_LOCAL_CKPT', 'lerobot/smolvla_base')
    device = os.getenv('VLA_DEVICE', 'cuda')
    policy = LocalSmolVLA(ckpt=ckpt, device=device)

    @app.get('/healthz')
    def healthz():
        return {'ok': True, 'ckpt': ckpt, 'device': device}

    @app.post('/act')
    def act(req: ActRequest):
        # Reconstruct the image from raw bytes + shape (matches remote_vla.py).
        raw = base64.b64decode(req.image_b64)
        img = np.frombuffer(raw, dtype=np.uint8).reshape(req.image_shape)
        action = policy.act(img, req.instruction, req.state)
        return action.to_dict()   # canonical contract on the wire

    return app


def main():
    ap = argparse.ArgumentParser(description='RAISE 2026 VLA policy server')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=8000)
    args = ap.parse_args()

    import uvicorn
    uvicorn.run(build_app(), host=args.host, port=args.port)


if __name__ == '__main__':
    main()
