#!/usr/bin/env python3
"""
RAISE 2026 — Lab 1, Script 10 — the orchestrator (LLM tool-use loop).
Author: Prof. Anis Koubaa <anis.koubaa@gmail.com>

WHAT  The climax of Day-1 Lab-1. You type a GOAL in plain English; an
      LLM (GPT-4o) decides which of the robot's tools to call, in what
      order, calls them (they're the ROS 2 services from 05-09), reads
      the results, and keeps going until the goal is done — then writes
      a short summary.

╔═══════════════════════════════════════════════════════════════════════════╗
║ THIS IS THE WHOLE POINT OF LAB 1.                                        ║
║                                                                           ║
║ In 05-09 YOU pressed menu keys to call services. Here the LLM does it.    ║
║ Nothing about the services changed — they're the same Trigger calls.     ║
║ We just hand the LLM a TOOL LIST (names + descriptions + arguments) and   ║
║ let it choose. That's "LLM tool use" / "function calling", and it's the   ║
║ same machinery a production agent uses.                                   ║
║                                                                           ║
║ The loop:                                                                 ║
║   1. send {system prompt, user goal, tool list} to the LLM               ║
║   2. LLM replies with either tool_calls OR a final text answer            ║
║   3. for each tool_call → call the matching ROS service → feed the        ║
║      result back to the LLM                                              ║
║   4. repeat until the LLM stops asking for tools                          ║
║                                                                           ║
║ Tools available to the LLM (each wraps a ROS 2 service you already built):║
║   navigate(destination)     → /nav_to_<destination>                       ║
║   move_arm(pose)            → /move_to_<pose>                             ║
║   operate_gripper(action)   → /<action>_gripper                          ║
║   detect_objects(camera)    → /detect_<camera>                           ║
║   inspect_plant()           → /inspect_plant                             ║
║   describe_scene()          → /describe_scene                            ║
╚═══════════════════════════════════════════════════════════════════════════╝

Needs all five tool servers running + OPENAI_API_KEY (in src/.env or env).
The `10_d1` alias starts the servers for you. Manual:

    ros2 run raise2026_tools gripper_server &
    ros2 run raise2026_tools move_to_pose_server &
    ros2 run raise2026_tools navigation_server &
    ros2 run raise2026_tools detector_server &
    ros2 run raise2026_tools inspector_server &
    ros2 run raise2026_labs 10_orchestrate.py     # or:  10_d1

Try goals like:
    "drive to tomato row 1, aim the arm at the plant, and tell me if it's healthy"
    "what do you see around you right now?"
    "go to the olive grove and detect any animals"
"""

import json
import os
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


# ── .env loader (same helper as inspector_server) ──────────────────────────
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


MODEL = 'gpt-4o'               # full model — plans efficiently, doesn't wander
MAX_TURNS = 6                  # tight cap — most goals finish in 2-3 turns

SYSTEM_PROMPT = (
    "You are the planner for a field robot in a greenhouse. Use the tools "
    "to accomplish the user's goal in the FEWEST CALLS POSSIBLE — usually "
    "2 turns. Do not explore.\n\n"
    "World layout: the robot drives in an aisle. Both rows of tomato plants "
    "flank the aisle — one on the robot's LEFT, one on its RIGHT. The two "
    "tomato_row waypoints (tomato_row_1, tomato_row_2) are different positions "
    "ALONG the aisle (east-west), not 'left vs right'. From either waypoint "
    "you can inspect plants on the LEFT or the RIGHT without moving again.\n\n"
    "RULES (follow strictly):\n"
    "1. To inspect a tomato plant: navigate() once to a tomato_row waypoint, "
    "   then inspect_plant(side='left' or 'right'). That's 2 calls total.\n"
    "2. When the user wants the OTHER side from the same spot (\"now the other "
    "   side\", \"and the right one\", \"now on the right\"), call inspect_plant "
    "   with the new side. DO NOT navigate. Just 1 call total.\n"
    "3. 'first / closest / this' tomato plant → use tomato_row_1 (the first "
    "   waypoint the robot reaches from home).\n"
    "4. Do NOT call describe_scene unless the user asks 'what do you see' or "
    "   similar open-ended question.\n"
    "5. Do NOT call move_arm before inspect_plant — the PTZ auto-aims.\n"
    "6. Never navigate twice in a row, and never repeat the same call.\n"
    "7. RELATIVE movement ('go back', 'go forward', 'back 1 meter', 'turn "
    "   around', 'turn left/right', 'the tree behind'): use drive(action, "
    "   steps), NOT navigate(). forward/backward = 1 m per step, turn = 90° "
    "   per step. 'go back 1 meter' → drive(backward, 1). 'turn around' → "
    "   drive(turn_left, 2). Only use navigate() when the user names a place "
    "   (home, a tomato row, the olive grove).\n"
    "8. describe_scene/detect_objects: only when the user explicitly asks for "
    "   a scene description or object detection. detect_objects(camera): "
    "   'ptz' wide view, 'wrist' close-up arm cam.\n"
    "When the goal is achieved, STOP calling tools and write a concise "
    "(2-4 sentence) summary of what you did and found."
)

# ── Tool schemas exposed to the LLM (OpenAI function-calling format) ───────
# Each tool's name + description + argument enum is what the LLM "sees".
# The enums mirror exactly the services the servers advertise.
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
            'description': "Move the robot base RELATIVE to where it is now, in fixed steps. Use this for 'go back', 'move forward', 'go back 1 meter', 'turn around', 'turn left/right'. forward/backward = 1 metre per call; turn_left/turn_right = 90° per call. For larger amounts, call multiple times (e.g. 'back 3 m' → drive(backward) three times). Use navigate() instead when the user names a place.",
            'parameters': {
                'type': 'object',
                'properties': {
                    'action': {
                        'type': 'string',
                        'enum': ['forward', 'backward', 'turn_left', 'turn_right'],
                    },
                    'steps': {
                        'type': 'integer',
                        'description': 'How many fixed steps (1 m or 90°) to take. Default 1.',
                    },
                },
                'required': ['action'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'move_arm',
            'description': 'Move the UR5e arm to a named pose.',
            'parameters': {
                'type': 'object',
                'properties': {
                    'pose': {
                        'type': 'string',
                        'enum': ['home', 'above_plant', 'side_view', 'stow'],
                    },
                },
                'required': ['pose'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'operate_gripper',
            'description': 'Open, close, or rotate the gripper.',
            'parameters': {
                'type': 'object',
                'properties': {
                    'action': {'type': 'string', 'enum': ['open', 'close', 'rotate']},
                },
                'required': ['action'],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'detect_objects',
            'description': 'Run YOLO object detection on a camera and return the top detections.',
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
            'name': 'inspect_plant',
            'description': 'Aim the PTZ camera at the LEFT or RIGHT side of the robot from where it currently stands, and ask a vision model for a plant-health verdict (Verdict / Reason / Confidence). Use this AFTER navigate() to a tomato_row waypoint. To inspect the other side, just call again with the other side — DO NOT navigate again.',
            'parameters': {
                'type': 'object',
                'properties': {
                    'side': {
                        'type': 'string',
                        'enum': ['left', 'right'],
                        'description': 'Which side of the robot to look at. "left" → the row to the robot\'s left (north side of the +1.5 m aisle). "right" → the row to the robot\'s right (south side).',
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
            'description': 'Use the PTZ mast camera + a vision model to describe what is in front of the robot.',
            'parameters': {'type': 'object', 'properties': {}},
        },
    },
]

# ── Map (tool name, args) → ROS service name ───────────────────────────────
# Every tool is backed by a std_srvs/Trigger service we built in 05-09.
def tool_to_service(name: str, args: dict) -> str:
    if name == 'navigate':        return f"/nav_to_{args['destination']}"
    if name == 'move_arm':        return f"/move_to_{args['pose']}"
    if name == 'operate_gripper': return f"/{args['action']}_gripper"
    if name == 'detect_objects':  return f"/detect_{args['camera']}"
    if name == 'inspect_plant':   return f"/inspect_{args['side']}"   # /inspect_left or /inspect_right
    if name == 'describe_scene':  return '/describe_scene'
    if name == 'drive':           return DRIVE_SERVICE[args['action']]
    raise ValueError(f'unknown tool {name}')


# drive(action) → the relative-motion service name.
DRIVE_SERVICE = {
    'forward':    '/drive_forward',
    'backward':   '/drive_backward',
    'turn_left':  '/turn_left',
    'turn_right': '/turn_right',
}


class Orchestrator(Node):
    def __init__(self):
        super().__init__('orchestrator')
        # Cache one client per service name, created lazily on first use.
        # NB: don't call this `self._clients` — rclpy.Node already uses that
        # name internally for its own list of clients.
        self._client_cache = {}

    def call_service(self, service: str, timeout_s: float = 45.0) -> str:
        """Call a Trigger service by name; return its response message."""
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


def run_goal(node: Orchestrator, oai, goal: str) -> None:
    """Run one full agent loop for a single user goal."""
    messages = [
        {'role': 'system', 'content': SYSTEM_PROMPT},
        {'role': 'user', 'content': goal},
    ]

    for turn in range(1, MAX_TURNS + 1):
        completion = oai.chat.completions.create(
            model=MODEL,
            messages=messages,
            tools=TOOLS,
        )
        msg = completion.choices[0].message
        # Append the assistant turn verbatim (needed so tool replies attach).
        messages.append(msg)

        # No tool calls → the LLM is done; print its final answer.
        if not msg.tool_calls:
            print('\n┌─ agent summary ─────────────────────────────────────')
            for line in (msg.content or '').splitlines():
                print(f'│ {line}')
            print('└─────────────────────────────────────────────────────')
            return

        # Otherwise execute each requested tool and feed results back.
        for tc in msg.tool_calls:
            name = tc.function.name
            try:
                args = json.loads(tc.function.arguments or '{}')
            except json.JSONDecodeError:
                args = {}
            service = tool_to_service(name, args)
            arg_str = ', '.join(f'{k}={v}' for k, v in args.items())
            print(f'  ⚙ turn {turn}: {name}({arg_str})   → {service}')

            # drive(action, steps) repeats the fixed-step service `steps` times
            # so "go back 3 m" works without a custom-distance message.
            if name == 'drive':
                steps = max(1, min(int(args.get('steps', 1) or 1), 10))
                result = node.call_service(service)
                for _ in range(steps - 1):
                    result = node.call_service(service)
                if steps > 1:
                    result = f'{result} (×{steps} steps)'
            else:
                result = node.call_service(service)
            print(f'      ↳ {result}')

            messages.append({
                'role': 'tool',
                'tool_call_id': tc.id,
                'content': result,
            })

    print('  ⚠ hit MAX_TURNS without a final answer — stopping.')


def main():
    loaded = load_dotenv_into_environ()
    api_key = os.environ.get('OPENAI_API_KEY')
    if not api_key:
        print('✗ OPENAI_API_KEY not found (looked in env and src/.env).')
        sys.exit(1)
    from openai import OpenAI
    oai = OpenAI(api_key=api_key)

    rclpy.init()
    node = Orchestrator()
    if loaded:
        print(f'  (loaded credentials from {loaded})')
    print('  (model = ' + MODEL + ')')

    BANNER = """
┌─── RAISE 2026 · Orchestrator (LLM agent) ──────────────────┐
│  Type a goal in plain English. The LLM will chain tools.   │
│  Examples:                                                 │
│    inspect tomato row 1 and tell me if it is healthy       │
│    what do you see around you right now?                   │
│    go to the olive grove and detect any animals            │
│  Type q to quit.                                           │
└────────────────────────────────────────────────────────────┘"""

    try:
        while rclpy.ok():
            print(BANNER)
            try:
                goal = input('  goal > ').strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if goal.lower() in ('q', 'quit', 'exit'):
                break
            if not goal:
                continue
            run_goal(node, oai, goal)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
