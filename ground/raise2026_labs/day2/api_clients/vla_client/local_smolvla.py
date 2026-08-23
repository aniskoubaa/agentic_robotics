# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — VLA client backend A: LOCAL SmolVLA (in-process, the default).

This is the brain the labs use unless you opt into a remote endpoint. It loads a
SmolVLA policy (the LeRobot-native 450M VLA) on the local GPU and runs inference
in-process — no server, no network. The checkpoint is either:
    - the instructor's PRE-FINE-TUNED reference checkpoint (default path), or
    - a team's OWN fine-tune from finetune_smolvla.py (advanced path),
selected with the VLA_LOCAL_CKPT env var (point it at the run's
`checkpoints/00XXXX/pretrained_model` directory).

Inference goes through LeRobot's OFFICIAL pipeline (`predict_action` + the
pre/post processor pipelines saved WITH the checkpoint). The preprocessor
tokenizes the instruction and normalizes images/state with the dataset stats;
the postprocessor un-normalizes the predicted action back to real joint radians.
Calling `policy.select_action` directly (our first attempt) fails — it expects
an already-tokenized, already-normalized batch. Found live, fixed here.

Heavy imports (torch, lerobot) are LAZY — they happen the first time you build a
LocalSmolVLA, not when this module is imported. That keeps `import vla_client`
working on a machine with no GPU/torch.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

from .base import Action, GRIPPER_OPEN, GRIPPER_CLOSED

# The VLM backbone SmolVLA is built on. lerobot instantiates it from the HF
# Hub even when the checkpoint is local — the hub round-trip is structural,
# not needed once the backbone sits in the local HF cache.
_BACKBONE_CACHE = (Path.home() / '.cache' / 'huggingface' / 'hub' /
                   'models--HuggingFaceTB--SmolVLM2-500M-Video-Instruct')


def _go_offline_if_possible(ckpt: str) -> None:
    """Skip ALL HF Hub traffic when everything needed is already on disk.

    With a LOCAL checkpoint and the backbone in the HF cache, model loading
    is fully self-contained — going offline removes the network dependency
    (lab wifi, rate limits, 'unauthenticated requests' warnings). If either
    piece is missing we stay online so the first download still works."""
    if os.getenv('HF_HUB_OFFLINE') is not None:
        return                                   # user already decided
    ckpt_local = Path(ckpt).expanduser().is_dir()
    if not ckpt_local and '/' in ckpt:
        # a hub repo id counts as "local" once its snapshot is in the cache
        ckpt_local = (Path.home() / '.cache' / 'huggingface' / 'hub' /
                      ('models--' + ckpt.replace('/', '--'))).is_dir()
    if ckpt_local and _BACKBONE_CACHE.is_dir():
        os.environ['HF_HUB_OFFLINE'] = '1'
        os.environ['TRANSFORMERS_OFFLINE'] = '1'


class LocalSmolVLA:
    """In-process SmolVLA backend. Implements the VLAClient protocol."""

    def __init__(self,
                 ckpt: str = 'lerobot/smolvla_base',
                 device: str = 'cuda',
                 instruction_default: str = ''):
        _go_offline_if_possible(ckpt)      # before transformers gets imported
        # ── Lazy, well-explained imports ────────────────────────────────────
        # Inside __init__ so a machine without torch can still import the
        # package and use the remote backend or test the Action contract.
        try:
            import torch
            from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
            from lerobot.policies.factory import make_pre_post_processors
            from lerobot.utils.control_utils import predict_action
        except ImportError as e:
            raise ImportError(
                "LocalSmolVLA needs torch + lerobot. Install into the lerobot venv:\n"
                '    ~/raise_venvs/lerobot/bin/python3 -m pip install "lerobot[smolvla]"\n'
                f"(original error: {e})"
            ) from e

        self._torch = torch
        self._predict_action = predict_action
        self.device = torch.device(device)
        self.instruction_default = instruction_default

        # The policy + the processor pipelines that were SAVED with it.
        # The preprocessor holds the tokenizer + the dataset-stats normalizers;
        # the postprocessor un-normalizes predicted actions -> real radians.
        self.policy = SmolVLAPolicy.from_pretrained(ckpt).to(self.device).eval()
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            self.policy.config, pretrained_path=ckpt)

        # Which image key(s) the policy was trained with. smolvla_base uses
        # observation.images.camera1/2/3; fine-tuning maps our wrist camera
        # onto camera1 via --rename_map (see finetune_smolvla.py). Feed our
        # single camera to the FIRST expected image feature.
        try:
            feats = self.policy.config.input_features
            self._image_keys = sorted(k for k in feats if 'image' in k)
        except AttributeError:
            self._image_keys = []
        if not self._image_keys:
            self._image_keys = ['observation.images.camera1']

    def reset(self):
        """Clear the policy's internal action-chunk queue.

        SmolVLA predicts CHUNKS of future actions and pops them from a queue
        across act() calls. Call reset() at the START of every episode/trial —
        otherwise stale chunks from the previous episode leak into the new one
        (found live: evaluation trial 1 succeeded, trials 2-4 timed out on
        leftover queue state).
        """
        self.policy.reset()

    def act(self, image, instruction: str, state=None) -> Action:
        """One forward pass: (image, instruction, state) -> canonical Action.

        `image`       : HxWx3 uint8 RGB (the 224x224 wrist frame).
        `instruction` : the typed natural-language command.
        `state`       : current joint vector (6 arm + gripper), radians.
        """
        instruction = instruction or self.instruction_default

        # Raw numpy obs dict — predict_action handles tensor conversion,
        # batching, normalization, tokenization, and un-normalization.
        obs = {self._image_keys[0]: np.asarray(image, dtype=np.uint8)}
        if state is not None:
            obs['observation.state'] = np.asarray(state, dtype=np.float32).reshape(-1)

        action = self._predict_action(
            obs, self.policy, self.device,
            self.preprocessor, self.postprocessor,
            use_amp=False, task=instruction, robot_type='ur5e',
        )
        return self._to_canonical(action)

    def _to_canonical(self, raw) -> Action:
        """Adapter: LeRobot action tensor -> our Action contract.

        Because we fine-tune on UR5e joint actions, the (un-normalized) output
        already lives in the UR5e command space: 6 joint targets + 1 gripper
        knuckle value in radians. Unpack + clamp for safety.
        """
        vec = raw.detach().float().cpu().numpy().reshape(-1)
        joints = [float(v) for v in vec[:6]]
        gripper = float(vec[6]) if vec.shape[0] > 6 else GRIPPER_OPEN
        # The dataset recorded the knuckle in radians (0..0.5) — no rescale.
        return Action(joints=joints, gripper=gripper).clamped()
