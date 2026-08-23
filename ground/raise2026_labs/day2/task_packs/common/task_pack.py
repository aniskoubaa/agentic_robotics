# Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>
"""
RAISE 2026 — task pack loader: the `--task A|C|B` switch.

A "task pack" is a self-contained Day-2 scenario: its instructions, its dataset
id, its demo recipe, its success rule, and its grading scenarios. The whole point
is ISOLATION — the record / fine-tune / execute code never changes between tasks;
you only swap which pack it loads. So choosing a task (or running several) is a
flag, not a code fork.

    from task_pack import load_task
    pack = load_task("A")                       # or "C", "B"
    repo = pack.dataset_repo_id(hf_user, team)  # where Lab 2.1 uploads / 2.3 trains
    for instr in pack.instructions: ...         # what the teleop demos must cover

Each pack lives in task_packs/task_<id>_*/ as two YAML files:
    pack.yaml       — metadata, instructions, demo recipe, success rule
    scenarios.yaml  — the graded scenarios (5 instructions x 3 start poses)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml


# task_packs/ root = the parent of this file's "common/" folder.
PACKS_ROOT = Path(__file__).resolve().parent.parent

# Friendly id -> pack directory name.
_PACK_DIRS = {
    'A': 'task_A_tabletop_pick',
    'C': 'task_C_ripeness_sort',
    'B': 'task_B_mobile_manip',
}


@dataclass
class TaskPack:
    """One loaded task pack. Everything the labs need to run a given task."""

    id: str
    name: str
    summary: str
    uses_base: bool                       # True only for Task B (Husky drive)
    recommended_episodes: int
    repo_slug: str                        # dataset name stem, e.g. "raise_tabletop_pick"
    instructions: list[str]               # language the demos must cover
    demo_recipe: list[str]                # teleop steps for one good episode
    success_rule: str                     # plain-English grading rule
    scenarios: list[dict] = field(default_factory=list)   # from scenarios.yaml

    def dataset_repo_id(self, hf_user: str, team_id: str | None = None) -> str:
        """The LeRobot/HF dataset id Lab 2.1 uploads to and fine-tuning reads."""
        stem = f'{hf_user}/{self.repo_slug}'
        return f'{stem}_{team_id}' if team_id else stem


def available_tasks() -> list[str]:
    """Task ids that have a pack directory on disk."""
    return [tid for tid, d in _PACK_DIRS.items() if (PACKS_ROOT / d).is_dir()]


def load_task(task_id: str) -> TaskPack:
    """Load a task pack by id ('A' | 'C' | 'B'). Raises if unknown/missing."""
    task_id = task_id.upper()
    if task_id not in _PACK_DIRS:
        raise ValueError(f"Unknown task '{task_id}'. Choose from {list(_PACK_DIRS)}.")

    pack_dir = PACKS_ROOT / _PACK_DIRS[task_id]
    if not pack_dir.is_dir():
        raise FileNotFoundError(f'Task pack directory missing: {pack_dir}')

    meta = yaml.safe_load((pack_dir / 'pack.yaml').read_text())
    scen_path = pack_dir / 'scenarios.yaml'
    scenarios = yaml.safe_load(scen_path.read_text())['scenarios'] if scen_path.exists() else []

    return TaskPack(
        id=task_id,
        name=meta['name'],
        summary=meta['summary'],
        uses_base=bool(meta.get('uses_base', False)),
        recommended_episodes=int(meta['recommended_episodes']),
        repo_slug=meta['repo_slug'],
        instructions=list(meta['instructions']),
        demo_recipe=list(meta['demo_recipe']),
        success_rule=meta['success_rule'],
        scenarios=scenarios,
    )


if __name__ == '__main__':
    # Tiny self-test: list packs and print a one-line summary of each.
    for tid in available_tasks():
        p = load_task(tid)
        print(f'[{p.id}] {p.name} — {p.recommended_episodes} eps, '
              f'{len(p.instructions)} instructions, base={p.uses_base}')
