# Lab 1.2 — `day1_02_agentic_inspector`

**Slot:** Day 1, 16:00 – 17:30 (90 min) · **Track:** both (advanced = multi-model compare)

## Goal

Turn the tool-use loop from Lab 1.1 into an **autonomous inspector**. You give
the agent a *mission* in plain English; it navigates and inspects on its own,
recovers from problems, and produces a **structured field report**.

## How this builds on Lab 1.1

Lab 1.1 ended with `10_orchestrate.py`: type one goal → the LLM chains a couple
of tool calls → it prints a sentence. This lab keeps that exact machinery (same
`std_srvs/Trigger` services, same `tool_to_service` map, same `.env` loader) and
adds the three things that make it *agentic*:

| | Lab 1.1 (`10_orchestrate.py`) | Lab 1.2 (`agent.py`) |
|---|---|---|
| Input | one goal | a whole mission |
| Output | a chat sentence | a structured report (JSON + table) |
| How the report is built | parsed from prose | the agent **calls** `record_finding(...)` per observation |
| Termination | LLM stops asking for tools | `finish_report` **or** `STEP_LIMIT` (30) **or** 5-min wall-clock |
| Failure handling | none | unreachable → record & move on; occluded → re-capture from a new pose |

The "report is built from tool calls, not prose" idea is the real lesson: a
production agent logs structured events, it doesn't hand you a paragraph.

## Reference loop (`starter/agent.py`)

```
mission + tool_schemas
   │
   ▼
LLM ──► tool_call? ──yes──► record_finding / finish_report (bookkeeping)
   │                    └─► else run ROS 2 service ──► observation ──┐
   no                                                                │
   ▼                                                  back to LLM ◄──┘
final field report          loop until finish_report OR step/time limit
```

## Files

| File | Status |
|------|--------|
| `README.md` (this file) | ✅ |
| `starter/agent.py` | ✅ shipped — runnable agent loop |
| `starter/mission_prompts.md` | ✅ shipped — mission library incl. graded scenarios |
| `solution/` | ☐ to author (private reference) |
| `evaluator/scenarios.yaml` + `evaluator/evaluate.py` | ☐ to author |
| `tests/` | ☐ to author |
| `instructor.md` | ☐ to author |

## Prerequisites

- `raise-sim` is running (Gazebo open, you see the agroforestry plot)
- The five tool servers from Lab 1.1 are running (the `agent_d2` alias starts
  them for you)
- `OPENAI_API_KEY` is set — in `raise_ros2_ws/src/.env` (gitignored) or the env

## How to run

**A. One alias (recommended)** — starts all five servers, then the agent:

```bash
agent_d2
```

**B. Manual** — five servers in the background, then the agent:

```bash
ros2 run raise2026_tools gripper_server &
ros2 run raise2026_tools move_to_pose_server &
ros2 run raise2026_tools navigation_server &
ros2 run raise2026_tools detector_server &
ros2 run raise2026_tools inspector_server &
ros2 run raise2026_labs agent.py          # press TAB after raise2026_labs
```

**C. Direct python3 while editing** (servers must already be up):

```bash
cd ~/Dev_WS/raise_summer_school/RAISE2026/raise_ros2_ws/src/raise2026_labs/day1_02_agentic_inspector/starter
python3 agent.py
```

Then type a mission at the `mission >` prompt — see `starter/mission_prompts.md`.
Each run prints a field report and saves it to
`/tmp/raise_field_report_<timestamp>.json`.

## Track differences

- **Beginner:** one provider (the shipped `agent.py` uses GPT-4o), fixed tool set
- **Advanced:** swap in Claude *and* a local Llama and compare; bonus for a
  custom MCP server exposing the same tools

## Grading

F1 over 3 scripted scenarios (see `starter/mission_prompts.md` for the prompts):

1. **Normal** — all plants reachable, lighting good
2. **Unreachable plant** — agent must report it, not loop forever
3. **Occluded camera** — agent must re-capture from a different pose

Step limit: 30 (enforced in `agent.py`). Wall-clock limit: 5 min per scenario
(enforced in `agent.py`). The `/tmp/raise_field_report_*.json` is the artifact
the evaluator scores.
