#!/usr/bin/env python3
"""
RAISE 2026 — Lab 1.2 — the agentic inspector (autonomous mission → field report).
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

WHAT  In Lab 1.1 (10_orchestrate.py) you typed ONE goal and the LLM chained
      a couple of tool calls to answer it. Here we go one step further: you
      hand the agent a MISSION ("inspect every tomato plant and file a health
      report"), and it runs ON ITS OWN — navigating, inspecting, recovering
      from problems — until the mission is done, then it produces a STRUCTURED
      FIELD REPORT (not just a chat sentence).

╔═══════════════════════════════════════════════════════════════════════════╗
║ WHAT IS NEW vs LAB 1.1                                                     ║
║                                                                           ║
║ 1. The deliverable is DATA, not prose. The agent files each observation   ║
║    by CALLING A TOOL — record_finding(...). So the report is built from   ║
║    structured tool calls, exactly like a production agent would log to a  ║
║    database. We print it AND save it to /tmp/raise_field_report_*.json.   ║
║                                                                           ║
║ 2. The agent is AUTONOMOUS over a multi-step mission, not a single goal.  ║
║    We bound it with a STEP LIMIT and a WALL-CLOCK limit so a confused     ║
║    agent can never loop forever (this is graded — see README scenarios).  ║
║                                                                           ║
║ 3. It must RECOVER, not give up: if a plant is unreachable it records the ║
║    failure and moves on; if a view is occluded it re-aims / re-captures   ║
║    from a different pose before reporting.                                ║
║                                                                           ║
║ The loop (same machinery as 10_orchestrate.py):                           ║
║   1. send {system prompt, mission, tool list} to the LLM                  ║
║   2. LLM replies with tool_calls OR finishes by calling finish_report     ║
║   3. for each tool_call → run the ROS 2 service → feed the result back    ║
║   4. repeat until finish_report OR a limit is hit                         ║
╚═══════════════════════════════════════════════════════════════════════════╝

The tools the agent can use (each wraps a ROS 2 Trigger service from Lab 1.1):
    navigate(destination)      → /nav_to_<destination>
    drive(action, steps)       → /drive_forward | /drive_backward | /turn_*
    inspect_plant(side)        → /inspect_left | /inspect_right   (VLM verdict)
    describe_scene()           → /describe_scene                  (VLM scene)
    detect_objects(camera)     → /detect_ptz | /detect_wrist      (YOLO)
    record_finding(...)        → (no ROS call) append one row to the report
    finish_report(summary)     → (no ROS call) end the mission

Needs the five tool servers running + OPENAI_API_KEY (in src/.env or env).
The `agent_d2` alias starts the servers for you. Manual:

    ros2 run raise2026_tools gripper_server &
    ros2 run raise2026_tools move_to_pose_server &
    ros2 run raise2026_tools navigation_server &
    ros2 run raise2026_tools detector_server &
    ros2 run raise2026_tools inspector_server &
    ros2 run raise2026_labs agent.py              # or:  agent_d2

Try missions (see starter/mission_prompts.md for the graded set):
    "inspect both tomato rows, left and right, and tell me which plants look unhealthy"
    "do a full health sweep of the greenhouse and file a report"
"""

import json
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


# ── .env loader (same helper as 10_orchestrate / inspector_server) ─────────
# Looks for raise_ros2_ws/src/.env and copies any KEY=value into the
# environment so the OpenAI client can find OPENAI_API_KEY.
def load_dotenv_into_environ():
    """Populate os.environ from raise_ros2_ws/src/.env (keys not already set)."""
    candidates = []
    for prefix in os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            candidates.append(os.path.join(prefix, '..', 'src', '.env'))
    candidates.append(os.path.join(os.getcwd(), '.env'))
    for path in candidates:
        if not os.path.isfile(path):
            continue
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    k, v = line.split('=', 1)
                    os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))
        return os.path.abspath(path)
    return None


MODEL = 'gpt-4o'          # full model — plans a multi-step mission reliably
STEP_LIMIT = 30           # hard cap on tool calls (graded — see README)
WALLCLOCK_LIMIT_S = 300   # 5 minutes per mission (graded)

# The system prompt is the agent's "standing orders". Unlike Lab 1.1 it must
# (a) work through a whole mission, (b) file findings as it goes, and
# (c) recover from the three failure modes the lab grades on.
SYSTEM_PROMPT = (
    "You are an autonomous field-inspection agent for a greenhouse robot. You "
    "are given a MISSION and you must carry it out end to end using the tools, "
    "then call finish_report when done.\n\n"
    "World layout: the robot drives an east-west aisle. Two rows of tomato "
    "plants flank the aisle — one on the robot's LEFT, one on its RIGHT. "
    "tomato_row_1 and tomato_row_2 are two positions ALONG the aisle, not "
    "'left vs right'. From either waypoint you can inspect the LEFT or RIGHT "
    "plant without moving again.\n\n"
    "HOW TO WORK:\n"
    "1. Inspect a plant in 2 steps: navigate() once to a tomato_row waypoint, "
    "   then inspect_plant(side). To inspect the other side from the same spot, "
    "   just call inspect_plant with the other side — DO NOT navigate again.\n"
    "2. After EVERY inspection, immediately call record_finding(...) with what "
    "   you observed. The report is built ONLY from your record_finding calls, "
    "   so an unrecorded observation does not exist.\n"
    "3. When the whole mission is covered, call finish_report(summary) with a "
    "   2-4 sentence wrap-up. Do not keep exploring after that.\n\n"
    "RECOVERING FROM PROBLEMS (this is the point of the lab):\n"
    "A. UNREACHABLE: if a navigate() or inspect call returns an ERROR or says a "
    "   plant cannot be reached, DO NOT retry it more than once. record_finding "
    "   with status='unreachable' and the error text, then move on. Never loop.\n"
    "B. OCCLUDED / BAD VIEW: if an inspection says the view is blocked, blurry, "
    "   empty, or it cannot tell, do NOT report a verdict from it. Change the "
    "   viewpoint first — drive(forward/backward, 1) a little or re-aim with the "
    "   other side — then inspect_plant again ONCE. If it is still bad, "
    "   record_finding with status='occluded'.\n"
    "C. Never call the exact same tool with the exact same arguments twice in a "
    "   row expecting a different answer.\n\n"
    "Be efficient: do not call describe_scene or detect_objects unless the "
    "mission explicitly asks about the scene or about objects/animals."
)

# ── Tool schemas exposed to the LLM (OpenAI function-calling format) ───────
# Six of these wrap a ROS service; the last two (record_finding, finish_report)
# are "bookkeeping" tools the agent uses to build the report and to stop.
TOOLS = [
    {
        'type': 'function',
        'function': {
            'name': 'navigate',
            'description': 'Drive the robot base to a named waypoint.',
            'parameters': {
                'type': 'object',
                'properties': {
                    'destination': {
                        'type': 'string',
                        'enum': ['home', 'tomato_row_1', 'tomato_row_2', 'olive_grove'],
                    },
                },
                'required': ['destination'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'drive',
            'description': "Move the robot base RELATIVE to where it is now, in fixed steps. forward/backward = 1 metre per step; turn_left/turn_right = 90 deg per step. Use this to change viewpoint when a view is occluded, or for 'go back'/'turn around' instructions.",
            'parameters': {
                'type': 'object',
                'properties': {
                    'action': {
                        'type': 'string',
                        'enum': ['forward', 'backward', 'turn_left', 'turn_right'],
                    },
                    'steps': {
                        'type': 'integer',
                        'description': 'How many fixed steps (1 m or 90 deg). Default 1.',
                    },
                },
                'required': ['action'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'inspect_plant',
            'description': "Aim the PTZ camera at the LEFT or RIGHT side of the robot and ask a vision model for a plant-health verdict. Use AFTER navigate() to a tomato_row waypoint. To inspect the other side, call again with the other side — do NOT navigate again.",
            'parameters': {
                'type': 'object',
                'properties': {
                    'side': {
                        'type': 'string',
                        'enum': ['left', 'right'],
                        'description': "Which side of the robot to look at.",
                    },
                },
                'required': ['side'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'describe_scene',
            'description': 'Use the PTZ mast camera + a vision model to describe what is in front of the robot. Only when the mission asks what is around.',
            'parameters': {'type': 'object', 'properties': {}},
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'detect_objects',
            'description': 'Run YOLO object detection on a camera and return the top detections. Only when the mission asks about objects/animals.',
            'parameters': {
                'type': 'object',
                'properties': {
                    'camera': {'type': 'string', 'enum': ['wrist', 'ptz']},
                },
                'required': ['camera'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'record_finding',
            'description': "File ONE observation into the field report. Call this after each inspection. This does not move the robot — it just logs structured data.",
            'parameters': {
                'type': 'object',
                'properties': {
                    'location': {
                        'type': 'string',
                        'description': "Where it was observed, e.g. 'tomato_row_1 left'.",
                    },
                    'status': {
                        'type': 'string',
                        'enum': ['healthy', 'unhealthy', 'unreachable', 'occluded'],
                        'description': "healthy/unhealthy = a real verdict; unreachable/occluded = a recovery outcome.",
                    },
                    'detail': {
                        'type': 'string',
                        'description': "The verdict reason, or the error/occlusion text.",
                    },
                    'confidence': {
                        'type': 'number',
                        'description': "0.0-1.0 if the vision model gave one, else 0.",
                    },
                },
                'required': ['location', 'status', 'detail'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'finish_report',
            'description': "End the mission. Call this once everything is inspected and recorded.",
            'parameters': {
                'type': 'object',
                'properties': {
                    'summary': {
                        'type': 'string',
                        'description': "A 2-4 sentence wrap-up of the mission.",
                    },
                },
                'required': ['summary'],
            },
        },
    },
]


# ── Map a ROS-backed tool (name, args) → ROS service name ──────────────────
# record_finding / finish_report are NOT here — they are handled in the loop.
DRIVE_SERVICE = {
    'forward':    '/drive_forward',
    'backward':   '/drive_backward',
    'turn_left':  '/turn_left',
    'turn_right': '/turn_right',
}


def tool_to_service(name: str, args: dict) -> str:
    if name == 'navigate':       return f"/nav_to_{args['destination']}"
    if name == 'inspect_plant':  return f"/inspect_{args['side']}"   # /inspect_left | /inspect_right
    if name == 'describe_scene': return '/describe_scene'
    if name == 'detect_objects': return f"/detect_{args['camera']}"
    if name == 'drive':          return DRIVE_SERVICE[args['action']]
    raise ValueError(f'tool {name} has no backing service')


class InspectorAgent(Node):
    def __init__(self):
        super().__init__('agentic_inspector')
        # One service client per service name, created lazily and reused.
        # (Don't name this `self._clients` — rclpy.Node uses that internally.)
        self._client_cache = {}

    def call_service(self, service: str, timeout_s: float = 45.0) -> str:
        """Call a std_srvs/Trigger service by name; return its response message."""
        client = self._client_cache.get(service)
        if client is None:
            client = self.create_client(Trigger, service)
            self._client_cache[service] = client
        if not client.wait_for_service(timeout_sec=5.0):
            return f'ERROR: service {service} is not available (is its server running?)'
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            return f'ERROR: {service} timed out'
        resp = future.result()
        if resp is None:
            return f'ERROR: {service} returned nothing'
        return resp.message


def print_report(mission: str, findings: list, summary: str, stopped_reason: str) -> dict:
    """Pretty-print the field report and return it as a dict (for saving)."""
    report = {
        'mission': mission,
        'generated': datetime.now().isoformat(timespec='seconds'),
        'summary': summary,
        'stopped_reason': stopped_reason,
        'findings': findings,
    }
    print('\n╔═══ RAISE 2026 · FIELD REPORT ' + '═' * 32)
    print(f'║ mission : {mission}')
    print(f'║ summary : {summary or "(none — mission did not finish cleanly)"}')
    print(f'║ status  : {stopped_reason}')
    print('╠' + '═' * 60)
    if not findings:
        print('║ (no findings recorded)')
    for i, f in enumerate(findings, 1):
        conf = f.get('confidence', 0) or 0
        print(f"║ {i}. [{f.get('status', '?'):<11}] {f.get('location', '?')}")
        print(f"║      {f.get('detail', '')}  (conf {conf:.2f})")
    print('╚' + '═' * 60)
    return report


def run_mission(node: InspectorAgent, oai, mission: str) -> dict:
    """Run one full autonomous mission and return the field-report dict."""
    messages = [
        {'role': 'system', 'content': SYSTEM_PROMPT},
        {'role': 'user', 'content': f'MISSION: {mission}'},
    ]
    findings = []           # built up from record_finding tool calls
    summary = ''
    stopped_reason = 'completed'
    started = time.monotonic()

    for step in range(1, STEP_LIMIT + 1):
        # Wall-clock guard — a slow/looping agent must still terminate.
        if time.monotonic() - started > WALLCLOCK_LIMIT_S:
            stopped_reason = f'stopped: exceeded {WALLCLOCK_LIMIT_S}s wall-clock limit'
            break

        completion = oai.chat.completions.create(
            model=MODEL,
            messages=messages,
            tools=TOOLS,
        )
        msg = completion.choices[0].message
        messages.append(msg)   # keep the assistant turn so tool replies attach

        # If the model answered in plain text with no tool call, nudge it:
        # this agent is supposed to end via finish_report, not by chatting.
        if not msg.tool_calls:
            if msg.content:
                summary = msg.content
            stopped_reason = 'completed (model ended without finish_report)'
            break

        done = False
        for tc in msg.tool_calls:
            name = tc.function.name
            try:
                args = json.loads(tc.function.arguments or '{}')
            except json.JSONDecodeError:
                args = {}

            # ── Bookkeeping tools: no ROS call ──────────────────────────────
            if name == 'record_finding':
                findings.append({
                    'location': args.get('location', '?'),
                    'status': args.get('status', '?'),
                    'detail': args.get('detail', ''),
                    'confidence': float(args.get('confidence', 0) or 0),
                })
                print(f"  📝 step {step}: recorded [{args.get('status')}] {args.get('location')}")
                result = 'ok, finding recorded'
            elif name == 'finish_report':
                summary = args.get('summary', '')
                print(f'  ✅ step {step}: finish_report')
                result = 'ok, report finished'
                done = True
            # ── ROS-backed tools: call the matching Trigger service ─────────
            else:
                service = tool_to_service(name, args)
                arg_str = ', '.join(f'{k}={v}' for k, v in args.items())
                print(f'  ⚙ step {step}: {name}({arg_str})   → {service}')
                if name == 'drive':
                    # repeat the fixed-step service `steps` times (1..10)
                    steps = max(1, min(int(args.get('steps', 1) or 1), 10))
                    result = node.call_service(service)
                    for _ in range(steps - 1):
                        result = node.call_service(service)
                    if steps > 1:
                        result = f'{result} (x{steps} steps)'
                else:
                    result = node.call_service(service)
                print(f'      ↳ {result}')

            # Feed the tool result back to the LLM so it can decide next step.
            messages.append({
                'role': 'tool',
                'tool_call_id': tc.id,
                'content': result,
            })

        if done:
            break
    else:
        # for-loop fell through without `break` → we hit STEP_LIMIT
        stopped_reason = f'stopped: hit STEP_LIMIT ({STEP_LIMIT} tool calls)'

    return print_report(mission, findings, summary, stopped_reason)


def save_report(report: dict) -> str:
    """Write the report to /tmp and return the path."""
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = f'/tmp/raise_field_report_{stamp}.json'
    with open(path, 'w') as f:
        json.dump(report, f, indent=2)
    return path


def main():
    loaded = load_dotenv_into_environ()
    api_key = os.environ.get('OPENAI_API_KEY')
    if not api_key:
        print('✗ OPENAI_API_KEY not found (looked in env and src/.env).')
        sys.exit(1)
    from openai import OpenAI
    oai = OpenAI(api_key=api_key)

    rclpy.init()
    node = InspectorAgent()
    if loaded:
        print(f'  (loaded credentials from {loaded})')
    print(f'  (model = {MODEL}, step_limit = {STEP_LIMIT}, wall-clock = {WALLCLOCK_LIMIT_S}s)')

    BANNER = """
┌─── RAISE 2026 · Agentic Inspector (autonomous mission) ────┐
│  Give the robot a MISSION. It will inspect on its own and  │
│  file a structured field report.                           │
│  Examples:                                                 │
│    inspect both tomato rows and report unhealthy plants    │
│    do a full health sweep and file a report                │
│  Type q to quit.                                           │
└────────────────────────────────────────────────────────────┘"""

    try:
        while rclpy.ok():
            print(BANNER)
            try:
                mission = input('  mission > ').strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if mission.lower() in ('q', 'quit', 'exit'):
                break
            if not mission:
                continue
            report = run_mission(node, oai, mission)
            path = save_report(report)
            print(f'  💾 report saved to {path}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
