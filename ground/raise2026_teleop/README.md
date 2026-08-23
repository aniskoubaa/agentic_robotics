# `raise2026_teleop`

Three minimal drivers for the RAISE 2026 Husky — keyboard, joystick, phone.
All three publish on the same `/cmd_vel` topic; only one needs to run at a time.

## Quick reference

| Mode | Command | Best for |
|---|---|---|
| Keyboard | `ros2 run raise2026_teleop teleop_keyboard` | Quick driving from any terminal |
| Joystick | `ros2 launch raise2026_teleop teleop_joy.launch.py` | Smooth analog control with a gamepad |
| Phone   | `ros2 run raise2026_teleop teleop_phone` | Demos — let an audience tap-drive |

## 1. Keyboard

```
ros2 run raise2026_teleop teleop_keyboard
```

Hold the key, robot moves. Tap `s` (or SPACE) to stop.

| Key(s)     | Action |
| ---------- | ------ |
| `w` or `↑` | forward |
| `a` or `←` | turn left |
| `d` or `→` | turn right |
| `x` or `↓` | backward |
| `s`        | **STOP** |
| `q` / `e`  | curve forward-left / forward-right |
| SPACE      | STOP |
| Ctrl-C     | exit |

Code: `raise2026_teleop/teleop_keyboard.py` — ~70 lines, no extra deps.

## 2. Joystick

Connect any USB or Bluetooth gamepad (Xbox-style assumed by default), then:

```bash
ros2 launch raise2026_teleop teleop_joy.launch.py
```

Default mapping (edit `config/teleop_joy.yaml` for other controllers):

| Control | Action |
|---|---|
| Left stick vertical | linear.x (forward / backward) |
| Right stick horizontal | angular.z (turn) |
| A button (hold) | **enable** — must hold to drive |
| B button (hold with A) | **turbo** — 3× speed |

Uses the stock `joy` + `teleop_twist_joy` packages — no custom code beyond the launch + YAML.

## 3. Phone (web teleop)

```bash
ros2 run raise2026_teleop teleop_phone
# then on your phone (same WiFi as the laptop):
#   http://<laptop-ip>:5000/
```

To find the laptop IP: `ip -4 addr show | grep inet | grep -v 127`.

You'll get a green page with arrow buttons. **Hold** to drive, **release** to stop.
Works over any modern mobile browser — no app to install.

Code:
- `raise2026_teleop/teleop_phone.py` — Flask + rclpy bridge (~80 lines)
- `templates/index.html` — touch UI

## See the camera feed

**Built-in tool (no code):**
```bash
ros2 run rqt_image_view rqt_image_view /wrist_camera/image_raw
```

**Our minimal node (~40 lines, shows FPS):**
```bash
ros2 run raise2026_teleop camera_view
# Q in the window to quit
```

**On the phone:** the camera feed is embedded directly above the buttons when you run `teleop_phone`.

## Topics published / subscribed (summary)

| Direction | Topic | Type | Source |
|---|---|---|---|
| pub | `/cmd_vel` | `geometry_msgs/Twist` | all three teleop modes |
| sub | `/joy` | `sensor_msgs/Joy` | (joystick mode reads this from `joy_node`) |
