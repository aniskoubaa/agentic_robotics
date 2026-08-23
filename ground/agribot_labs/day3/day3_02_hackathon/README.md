<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# 🏆 The Autonomous Greenhouse Challenge — `day3_02_hackathon`

**Slot:** Day 3, 14:00 – 17:00 build · 17:00 – 18:00 judging · **Teams of 3**

## Mission brief (given at 14:00)

> *"Your robot manages the greenhouse alone. In one autonomous run:*
> 1. *Patrol the rows and produce a HEALTH MAP of all 15 plants — which are
>    diseased or wilted?*
> 2. *At each of the 4 HARVEST STATIONS, pick ONLY the ripe red tomato — leave
>    the green ones — and deliver it to the collection basket.*
> 3. *File a structured FIELD REPORT (JSON + markdown).*
>
> *3 hours to build. 3 scored runs — best counts. Your pick skill must be the
> Day-2 VLA. At judging, one surprise scenario will be injected."*

This is Days 1+2 forced into one robot: the **LLM plans** (patrol, recovery,
reporting — Day 1), the **VLA acts** (every pick — Day 2), and your
architecture is what competes.

## Build it in five natural phases

The mission is the natural sequence of an autonomous robot — each phase is a skill
you already have, in build order. **Do them in order; each is demoable on its own.**

| Phase | You build | Tool you already have | From |
|---|---|---|---|
| **1. Navigation** | drive the rows, reach a station | `navigation_server` (`07_call_navigation.py`) | Day 1 |
| **2. Detection** | find & localize plants + tomatoes | `detector_server` / YOLOv8 (`08_call_detector.py`) | Day 1 |
| **3. Reasoning** | health + ripeness + *why* → health map | `inspector_server` / VLM (`09_call_inspector.py`) | Day 1 |
| **4. Act (VLA)** | pick the red tomato + deliver | `vla_execute` / SmolVLA (`vla_executor.py`) | Day 2 |
| **5. All together** | one autonomous run + report + surprise | the skeleton (`skeleton_agent.py` / `skeleton_graph.py`) | Day 3 |

## Rules

- **Autonomous:** one command starts the run; no human input after that.
- **VLA required:** picks must go through `vla_execute` (reference checkpoint
  provided; your own fine-tune allowed and admired). A declared scripted-pick
  fallback earns **−50%** of that station's points.
- **3 scored runs** against the live evaluator; the leaderboard shows your best.
- **Surprise at judging:** the final run injects one unseen scenario
  (examples: a station with only green tomatoes — the correct move is to
  *refuse*; a blocked row; a moved basket; a camera dropout).
- `grasp_server` runs during evaluation; `/grasp/state` is the pick truth.
  **Grabbing a green tomato costs −5 points each.** Safety counts.

## Scoring — five phases, 100 points (incremental)

Points are **cumulative**: complete a phase, bank it. You keep whatever you climb
to, and each phase maps to a certification tier.

| Phase | Focus | Pts | Running | Tier |
|---|---|---:|---:|---|
| **P1 Navigation** | autonomously reach waypoints / stations | 15 | 15 | |
| **P2 Detection** | find & localize plants + tomatoes (YOLO) | 15 | 30 | participation |
| **P3 Reasoning** | health map + ripeness + explanation (VLM) | 20 | 50 | **Bronze** |
| **P4 Act (VLA)** | pick the ripe red tomato + deliver | 25 | 75 | **Silver** |
| **P5 All together** | full autonomous run + report + surprise | 25 | 100 | **Gold** |

**Within a phase, credit is proportional** — F1 on the 15-plant health map; each
harvest station ≈ 10 pts (~6 correct VLA pick on `/grasp/state` + ~4 delivery), so
even one station moves your score. A **scripted** pick = −50% of that station; a
**green** attach = −5 (safety). ≤500 ms/action latency for full pick credit;
wall-clock is the tiebreaker.

P5 = full-loop linkage (~10) + a schema-valid field report (~5–10) + correct
surprise behavior (~5). **Code quality, logging honesty, and creativity** are
judged from your repo + 2-min video across P5 and the tiebreaker.

**Standalone capability demos count:** an unintegrated **Navigation**,
**Detection**, or **VLA** skill banks its phase (P1 / P2 / P4) even if you never
wire the full loop — so a team is never all-or-nothing.

## Certification tiers

- **Gold** ≥ 85 · **Silver** 65 – 84 · **Bronze** 40 – 64

## Submission package

GitHub repo (clone `submission_template/`) + **2-minute demo video** + the
field report from your best run.

## Files to author (instructor workplan — see the design doc)

- `evaluator/run_evaluation.py` — full mission auto-scorer, phase-by-phase
  (extends the Day-2 `evaluate.py` pattern: waypoints, detections, health-map F1,
  `/grasp/state` picks + deliveries, loop/surprise checks)
- `scenarios/` — `nominal.yaml`, `green_only_station.yaml`, `blocked_row.yaml`,
  `moved_basket.yaml`, `dead_camera.yaml`
- `challenge_world/` — station spawner + basket zone (built on `gz_utils` +
  `sim_poses`)
- `leaderboard/` — static HTML consuming the scored JSON (live during build)
- `submission_template/` · `judging_rubric.md` · `instructor.md`

Full rationale, schedule, risks: [`../DAY3_CHALLENGE_DESIGN.md`](../DAY3_CHALLENGE_DESIGN.md).
Student-facing brief, guide, and slides: `lectures/day3_challenge/`.
