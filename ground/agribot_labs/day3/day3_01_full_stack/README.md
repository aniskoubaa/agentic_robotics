<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Guided Lab 3.1 — Assemble the Full Stack

**Slot:** Day 3, 10:30 – 12:30 (120 min) · **Track:** both

## The objective of this lab

> **The objective of this lab is to run ONE complete "plan → act → report"
> pipeline end-to-end** — the Day-1 LLM planner and the Day-2 VLA executor
> working as a single robot — so that every team enters the afternoon
> hackathon with a working baseline, not a blank page.

This is the "reference checkpoint" philosophy from Day 2 applied to
architecture: we ship a runnable skeleton; you make it *yours* in the hackathon.

## Two skeletons — you choose (same contract, same mission)

| | Skeleton A — plain-Python | Skeleton B — LangGraph |
|---|---|---|
| File | `starter/skeleton_agent.py` | `starter/skeleton_graph.py` |
| Built from | Day-1 `agent.py` tool-loop you already know | `StateGraph` nodes/edges + retry branch |
| New concepts | none — LLM plans with one new tool: `vla_pick` | graph state, conditional edges, tracing |
| Best for | beginners; fastest to modify | teams who think in flowcharts; advanced |

Both implement the same node contract the evaluator expects:

```
patrol → inspect(plant) → diagnose → [health_map]
      → goto(station) → vla_pick("pick the red tomato") → deliver(basket)
      → report()          # every node logs a structured event
```

The `vla_pick` node **is Day 2**: it calls `vla_execute` with the trained
SmolVLA checkpoint (grasp_server provides the grasp, `/grasp/state` the truth).

Those nodes **are the five challenge phases**, in order: `patrol` / `goto` =
Phase 1 (Navigation), `inspect` = Phase 2 (Detection), `diagnose` = Phase 3
(Reasoning), `vla_pick` / `deliver` = Phase 4 (Act), and wiring them into one
logged autonomous run = Phase 5 (All together). Finishing this guided lab already
puts **Phases 1–3 within reach** — the afternoon hardens Phase 4 and Phase 5. See
the phase rubric in [`../DAY3_CHALLENGE_DESIGN.md`](../DAY3_CHALLENGE_DESIGN.md).

## What you do in the 120 minutes

1. Launch the sim + grasp_server + tool servers (one alias, provided).
2. Run **your chosen skeleton** on the known-good scenario — watch it patrol,
   inspect, pick a red tomato, deliver, and print a field report (< 5 min run).
3. Read the skeleton top to bottom with the instructor — every node maps to
   something you built on Day 1 or Day 2.
4. Make one small modification (e.g. change the patrol order, add a log field)
   and re-run — proving to yourself you can change it safely.

## Deliverable

A green smoke-test: `evaluator/smoke_test.py` confirms your skeleton completes
the demo scenario. That's your ticket into the 14:00 mission drop.

## Files

| File | Status |
|------|--------|
| `README.md` (this file) | ✅ |
| `starter/skeleton_agent.py` (plain-Python) | ☐ to author |
| `starter/skeleton_graph.py` (LangGraph) | ☐ to author (after A works) |
| `starter/nodes/` (shared node implementations) | ☐ to author |
| `evaluator/smoke_test.py` | ☐ to author |
| `instructor.md` | ☐ to author |

Design context: [`../DAY3_CHALLENGE_DESIGN.md`](../DAY3_CHALLENGE_DESIGN.md).
