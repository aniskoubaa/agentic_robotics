<!-- Author: Prof. Anis Koubaa <anis.koubaa@gmail.com> -->

# Day 3 — The Autonomous Greenhouse Challenge (design)

> **Day 3 in one sentence:** teams get a full day to build ONE autonomous robot
> that *plans like Day 1 and acts like Day 2* — then compete on a live-scored
> greenhouse mission with a surprise twist at judging.
>
> Locked decisions (2026-07-04): **two skeletons offered** (plain-Python agent
> loop AND LangGraph — students choose) · **VLA required for picks** (scripted
> pick = −50% fallback) · mechanics kept: **3 runs best-counts, surprise
> scenario, Gold/Silver/Bronze, repo + 2-min video + report submission**.
>
> Locked (2026-07-11): scoring is now a **5-phase incremental rubric** ---
> Navigation · Detection · Reasoning · Act (VLA) · All-together = 15·15·20·25·25 ---
> so partial work always scores and each standalone skill banks its phase. Mission
> and mechanics are unchanged; only the rubric was re-expressed as phases.

---

## 1. Why this mission (and why not the old one)

The original sketch asked robots to *"pick one diseased leaf per sick plant."*
**Leaves are not graspable in the sim** — nothing can pick foliage. Day 2's
verified manipulation layer picks **spawned tomatoes** via the `grasp_server`.
So the challenge is rebuilt from **capabilities we have live-verified**:

| Verified capability | From | Used in the mission as |
|---|---|---|
| Husky driving (`/cmd_vel`), navigation services | Day 1 | patrolling the rows |
| Plant inspection (YOLO `detector_server`, VLM `inspector_server`) | Day 1 | the health map — the world **already contains** healthy / diseased / wilted plant variants with known ground truth |
| LLM tool-calling agent loop | Day 1 | the planner that orchestrates everything |
| Graspable red/green tomatoes + deterministic `grasp_server` | Day 2 | the harvest stations |
| Trained SmolVLA policy (`vla_execute`) | Day 2 | **the only sanctioned pick skill** |
| `/grasp/state`, world ground truth, latency logs | Day 2 | clean auto-scoring signals |

## 2. The mission (handed to teams at 14:00)

> *"Your robot manages the greenhouse alone. In one autonomous run:*
> 1. *Patrol the rows and produce a HEALTH MAP of all 15 plants — which are
>    diseased or wilted?*
> 2. *At each of the 4 HARVEST STATIONS, pick ONLY the ripe red tomato — leave
>    the green ones — and deliver it to the collection basket.*
> 3. *File a structured FIELD REPORT (JSON + markdown) of what you found and did.*
>
> *You have 3 hours to build, 3 scored runs (best counts), and your pick skill
> must be the Day-2 VLA. At judging, one surprise scenario will be injected."*

**Harvest stations** are spawn points where the evaluator places tomatoes
(red + green mix) at the reachable grasp points — the same no-IK mechanics the
Day-2 dataset used, so the trained policy is in-distribution. Stations sit next
to specific plants, so the robot must *drive* between them (mobile manipulation
= the Task-B ambition, delivered via the challenge).

## 3. Day schedule

| Time | Block |
|---|---|
| 09:00–10:30 | **Lecture D3L1** — planner ↔ executor architectures: how an LLM brain and a VLA body compose; the two skeletons side by side |
| 10:30–12:30 | **Guided Lab 3.1** — run the shipped skeleton end-to-end ONCE on a known-good scenario (every team leaves with a working baseline — same "reference checkpoint" philosophy as Day 2) |
| 12:30–14:00 | lunch — instructors reset sim boxes, leaderboard up |
| 14:00 | **Mission drop** + repo template released |
| 14:00–17:00 | **Build** — 3 scored runs against the live evaluator, leaderboard updates in real time |
| 17:00–17:45 | **Judging** — final run with the surprise scenario + 2-min demo videos |
| 17:45–18:00 | Awards: Gold / Silver / Bronze certificates |

## 4. Two skeletons, one contract (students choose)

Both skeletons implement the SAME node contract, so the evaluator and the
mission don't care which one a team uses:

```
patrol → inspect(plant) → diagnose → [health_map]
      → goto(station) → vla_pick("pick the red tomato") → deliver(basket)
      → report()                    # every node logs a structured event
```

- **Skeleton A — plain-Python agent loop** (`starter/skeleton_agent.py`):
  Day-1's `agent.py` pattern extended — the LLM plans with tools
  `{navigate, inspect, vla_pick, deliver, record_finding, finish_report}`,
  where `vla_pick` wraps Day-2's `vla_execute`. No new framework; students
  already know every line. *Recommended for beginners.*
- **Skeleton B — LangGraph `StateGraph`** (`starter/skeleton_graph.py`):
  explicit nodes/edges, retry branch (`on_failure → retry ×2 → escalate`),
  optional LangSmith tracing. Industry-flavored, visualizable. *For teams who
  want the graph mental model — taught in the morning lecture.*

Beginner path = pick either skeleton and improve it. Advanced path = free
architecture (multi-agent, memory, retrieval…) as long as the run is
autonomous and the pick is the VLA.

## 5. The VLA rule (locked)

- The pick skill **must be** `vla_execute` (the Day-2 executor) driving the
  arm — the reference checkpoint is provided; teams MAY substitute their own
  fine-tune (and brag about it in the report).
- A scripted pick is allowed **only as a declared fallback at −50% of that
  station's points** — a team is never fully blocked by a flaky policy, but
  the leaderboard rewards the real thing.
- `grasp_server` runs during evaluation (it is the grasp physics), and
  `/grasp/state` is the pick ground truth: red attach = success, green attach
  = **negative points** (safety).

## 6. Scoring — five phases, 100 pts (incremental)

The rubric is a **phase ladder**: complete a phase, bank its points. Points are
cumulative, and each phase maps to a certification tier. The natural build order
is Navigation → Detection → Reasoning → Act → Integrate (Days 1→2→3).

| Phase | Focus | Pts | Running | Tier |
|---|---|---|---|---|
| **P1 Navigation** | autonomously reach waypoints / stations | 15 | 15 | |
| **P2 Detection** | find & localize plants + tomatoes (YOLO) | 15 | 30 | participation |
| **P3 Reasoning** | health map + ripeness + explanation (VLM) | 20 | 50 | **Bronze** |
| **P4 Act (VLA)** | pick the ripe red tomato + deliver | 25 | 75 | **Silver** |
| **P5 All together** | full autonomous run + field report + surprise | 25 | 100 | **Gold** |

Within a phase, credit is **proportional**: F1 on the 15-plant health map; each
harvest station ≈ 10 pts (~6 for a correct VLA pick on `/grasp/state` + ~4 for
delivery), so even one station scores. A **scripted** pick = −50% of that station;
a **green** attach = −5 (safety). ≤500 ms/action latency for full pick credit
(logged by the executor). Wall-clock is the tiebreaker.

P5 breaks down as full-loop linkage (~10) + a schema-valid field report (~5–10) +
correct surprise behavior (~5). **Code quality, logging honesty, and creativity**
are judged from the repo + 2-min video as a quality lens across P5 and the
tiebreaker. **Standalone capability demos count**: an unintegrated Navigation,
Detection, or VLA skill banks its phase (P1 / P2 / P4) even without the full loop.

**Certificates:** Gold ≥ 85 · Silver 65–84 · Bronze 40–64.

## 7. Surprise scenarios (one injected at judging, sealed until 17:00)

| Scenario | The correct behavior it tests |
|---|---|
| `green_only_station.yaml` — one station has ONLY green tomatoes | the robot must **refuse to pick** and note it in the report (language grounding + honesty) |
| `blocked_row.yaml` — an obstacle blocks one row | replan the patrol route, mark plants as unobserved rather than guessing |
| `moved_basket.yaml` — basket displaced 1 m | re-locate the delivery zone (perception, not hardcoded coords) |
| `dead_camera.yaml` — wrist camera stream drops for 30 s | detect the fault, wait/retry, report degraded confidence |

## 8. Submission package (unchanged)

GitHub repo URL (from the released `submission_template/`) + 2-minute demo
video + the field report from the best run. Judges review code and video while
the auto-scores are already on the leaderboard.

## 9. What must be built (workplan — order matters)

1. **`challenge_world` scenario layer** — station spawner (tomatoes at grasp
   points near 4 chosen plants), basket zone model + pose check, scenario YAMLs
   (nominal + 4 surprises). *Builds directly on `gz_utils` + `sim_poses`.*
2. **Mission evaluator** (`day3_02/evaluator/run_evaluation.py`) — runs a
   team's entry, scores the **automatable phases** (P1 waypoints reached, P2
   detections, P3 health-map F1, P4 `/grasp/state` picks + deliveries, and the
   P5 loop/surprise checks) from ground truth (`/grasp/state`, world poses,
   health-map F1 vs the world file), emits scored JSON. *Extends the Day-2
   `evaluate.py` pattern.*
3. **Skeleton A** (plain-Python) — Day-1 `agent.py` + `vla_pick` tool. Cheap.
4. **Skeleton B** (LangGraph) — same contract as A; adds the dependency to the
   install docs. Ship only after A works.
5. **Leaderboard** — static HTML polling the scored-JSON directory.
6. **`submission_template/`**, `judging_rubric.md`, instructor runbook.
7. **Lecture D3L1 deck** — last, from the working system (the Day-2 lesson).

**Pre-school gate (same discipline as Day 2):** instructors must complete one
full Gold-level run themselves with the reference checkpoint before the school —
that run's video doubles as the Lab-3.1 demo.

## 10. Risks & honest notes

- **VLA generalization risk:** the policy was trained at 2 grasp points from a
  stationary base. Stations reuse those exact relative geometries (drive → park
  at the trained offset → pick), keeping it in-distribution. If live testing
  shows drift after parking, add a base-alignment helper to the skeletons.
- **3 h is short:** the skeletons must genuinely run the whole mission
  out-of-the-box at ~Bronze level; teams then climb by improving nodes, not by
  plumbing.
- **20 students / shared GPU:** evaluation runs are serialized through the
  instructor box (the run queue is also what makes the leaderboard fair).
