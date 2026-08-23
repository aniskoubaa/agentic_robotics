# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — VLA client factory: one switch picks the brain.

The Lab 2.2 executor calls make_vla_client(...) and gets back "something that
acts" — it never imports a concrete backend. Defaults make everything run FULLY
LOCAL with no server and no network: if VLA_BACKEND is unset you get in-process
SmolVLA. Set VLA_BACKEND=remote-* + VLA_REMOTE_URL to flip to a self-hosted
endpoint, and nothing else in the executor changes.

Config contract (see LOCAL_VS_REMOTE_VLA.md):
    VLA_BACKEND     local-smolvla (default) | remote-openvla | remote-pi0
    VLA_LOCAL_CKPT  a fine-tuned output_dir; if unset, the local REFERENCE
                    checkpoint (~/raise_checkpoints/smolvla_C_ref) is used
                    when it exists, else the PUBLIC Hugging Face copy
                    (scalexi/smolvla-raise2026-ripeness-ref) is downloaded
                    automatically — no key needed, cached after first use
    VLA_REMOTE_URL  required only for remote-*  (e.g. http://gpu-box.lan:8000)
    VLA_DEVICE      cuda if available else cpu
"""

from __future__ import annotations

import os
from pathlib import Path


def find_reference_ckpt():
    """The instructor-provided fine-tuned checkpoint, if installed.

    Every tool that loads a local model (vla_executor, vla_one_step, the
    evaluator) should behave the same when VLA_LOCAL_CKPT is unset — so the
    discovery lives HERE, in the factory, not in each script.
    """
    root = Path.home() / 'raise_checkpoints' / 'smolvla_C_ref' / 'checkpoints'
    if root.is_dir():
        steps = sorted(d for d in root.iterdir() if (d / 'pretrained_model').is_dir())
        if steps:                       # 'last' symlink sorts after the numbers
            return steps[-1] / 'pretrained_model'
    return None


# The same reference model, published publicly on Hugging Face (no key).
HF_REFERENCE = 'scalexi/smolvla-raise2026-ripeness-ref'


def _hf_cached(repo_id: str) -> bool:
    cache = Path.home() / '.cache' / 'huggingface' / 'hub' / (
        'models--' + repo_id.replace('/', '--'))
    return cache.is_dir()


def _default_local_ckpt() -> str:
    if os.getenv('VLA_LOCAL_CKPT'):
        return os.environ['VLA_LOCAL_CKPT']
    ref = find_reference_ckpt()
    if ref is not None:
        return str(ref)
    # No local install → use the public HF copy. from_pretrained downloads it
    # into the HF cache on first use (~900 MB), then it's local forever.
    return HF_REFERENCE


def _default_device() -> str:
    """cuda when torch sees a GPU, else cpu — without importing torch eagerly."""
    if os.getenv('VLA_DEVICE'):
        return os.environ['VLA_DEVICE']
    try:
        import torch
        return 'cuda' if torch.cuda.is_available() else 'cpu'
    except ImportError:
        return 'cpu'


def make_vla_client(backend: str | None = None):
    """Build the configured VLA backend.

    `backend` overrides VLA_BACKEND (which itself defaults to local-smolvla).
    Returns an object implementing the VLAClient protocol from base.py.
    """
    backend = (backend or os.getenv('VLA_BACKEND', 'local-smolvla')).lower()

    if backend in ('local-smolvla', 'local'):
        from .local_smolvla import LocalSmolVLA
        ckpt = _default_local_ckpt()
        if ckpt == HF_REFERENCE and not _hf_cached(HF_REFERENCE):
            print(f'no local checkpoint — downloading the reference model from '
                  f'Hugging Face ({HF_REFERENCE}, ~900 MB, one time; '
                  f'get_brain_d4 is the faster offline-safe alternative) ...')
        if ckpt == 'lerobot/smolvla_base':
            print('⚠ running the UN-fine-tuned base model — it will flail. '
                  '(Set VLA_LOCAL_CKPT to a fine-tuned checkpoint for a real run.)')
        return LocalSmolVLA(ckpt=ckpt, device=_default_device())

    if backend.startswith('remote'):
        # remote-openvla / remote-pi0 / remote-smolvla all speak the same HTTP
        # contract — the server decides which model it hosts.
        from .remote_vla import RemoteVLA
        return RemoteVLA(url=os.getenv('VLA_REMOTE_URL', ''))

    raise ValueError(
        f"Unknown VLA_BACKEND '{backend}'. "
        "Use local-smolvla (default) | remote-openvla | remote-pi0."
    )
