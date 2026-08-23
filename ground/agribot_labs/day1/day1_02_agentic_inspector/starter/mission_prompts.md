<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Mission prompts — Lab 1.2 agentic inspector

Paste any of these at the `mission >` prompt of `agent.py`. Unlike Lab 1.1
(one goal → a couple of calls), a *mission* is a whole job the agent runs on
its own until it calls `finish_report`.

## Warm-up (single plant)

- `inspect the tomato plant on your left at row 1 and tell me if it's healthy`
- `go to tomato row 2 and check the right-hand plant`

## Full sweep (the typical mission)

- `inspect both tomato rows, left and right, and file a report of any unhealthy plants`
- `do a complete health sweep of the greenhouse and report what you find`
- `check every tomato plant you can reach and rank them from healthiest to sickest`

## Scene / detection missions

- `drive around and tell me if there are any animals near the olive grove`
- `describe what you can see from tomato row 1, then inspect the left plant`

## The three graded scenarios (see ../README.md → Grading)

These map to the F1-scored scenarios. The *grader* sets the world up; your
prompt stays the same — the agent's **behaviour** is what's scored.

1. **Normal** — all plants reachable, lighting good.
   > `inspect both tomato rows and report each plant's health`
   - Pass: every plant gets one `record_finding` with `healthy`/`unhealthy`.

2. **Unreachable plant** — one waypoint/inspection errors out.
   > `inspect both tomato rows and report each plant's health`
   - Pass: the agent records that plant as `status: unreachable`, **does not
     loop**, and still finishes the rest of the mission.

3. **Occluded camera** — one view is blocked/blurry/empty.
   > `inspect the tomato plant on your left and tell me if it's healthy`
   - Pass: the agent changes viewpoint (a small `drive` or re-aim) and
     re-inspects **once** before reporting; if still bad it records
     `status: occluded` rather than inventing a verdict.

## What "done" looks like

After each mission the agent prints a **FIELD REPORT** table and saves it to
`/tmp/raise_field_report_<timestamp>.json`. That JSON is the deliverable the
evaluator scores — open it and check every reachable plant has exactly one
finding.
