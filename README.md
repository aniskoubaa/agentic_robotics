# Agentic Robotics

A ROS 2 learning environment with five simulated robot platforms:

- PX4 UAVs
- RaiseBot mobile manipulator
- Unitree Go2 quadruped
- UR5e robot arm
- TurtleBot 3 and TurtleBot 4

Each platform includes examples, keyboard control, demonstrations, and a
diagnostic command. The supported system is **Ubuntu 24.04 on an amd64 PC**.

## 1. Install everything

Open a terminal and run:

```bash
mkdir -p ~/ros2_ws/src
git clone https://github.com/aniskoubaa/agentic_robotics.git ~/ros2_ws/src/agentic_robotics
cd ~/ros2_ws/src/agentic_robotics
./install.sh
```

Enter your sudo password when requested. The rest is automatic. Installation
can take a long time because it downloads ROS 2, Gazebo, PX4, PyTorch, and the
AI libraries. Keep the terminal open until you see:

```text
Full installation complete.
```

The installer is safe to run again if it is interrupted. It keeps a log in:

```text
~/ros2_ws/install_logs/
```

To check an existing installation without downloading anything:

```bash
cd ~/ros2_ws/src/agentic_robotics
./install.sh --check
```

## 2. Start your first robot

Close the installation terminal and open a **new terminal**. Choose one robot:

| Robot | Command |
|---|---|
| UAV | `agr-sim` |
| RaiseBot | `agr-sim ground tools:=true` |
| Unitree Go2 | `agr-sim legged` |
| UR5e arm | `agr-sim arm tools:=true` |
| TurtleBot 3 | `agr-sim turtlebot` |
| TurtleBot 4 | `agr-sim turtlebot model:=tb4_lite` |

For example:

```bash
agr-sim ground tools:=true
```

Gazebo may need a minute to open on the first run.

## 3. Check and stop the robot

Open another terminal and run the diagnostic command for your robot:

| Robot | Diagnostic command |
|---|---|
| UAV | `ros2 run agr_uav_demos diagnose` |
| RaiseBot | `ros2 run raisebot_demos diagnose` |
| Unitree Go2 | `ros2 run agr_legged_demos diagnose` |
| UR5e arm | `ros2 run agr_arm_demos diagnose` |
| TurtleBot | `ros2 run agr_tb_demos diagnose` |

Stop any running simulation with:

```bash
agr-stop
```

## Useful commands

```bash
agr_help                                 # show all shortcuts
agr-build                                # rebuild the ROS workspace
agr-stop                                 # stop the simulation

# UAV
agr-sim                                  # x500 quadrotor
agr-sim world:=agr_city                  # fly in the city
agr-sim airframe:=rc_cessna              # fixed-wing aircraft
agr-sim uav --multi count:=3             # three UAVs

# RaiseBot
agr-sim ground tools:=true               # greenhouse and service tools
agr_drive                                # keyboard driving
agr_ground_demo                          # greenhouse demonstration

# Unitree Go2
agr-sim legged                           # inspection environment
agr_walk                                 # keyboard walking
agr_legged_demo                          # walking demonstration

# UR5e arm
agr-sim arm tools:=true                  # arm and service tools
agr_jog                                  # keyboard arm control
agr_arm_demo                             # pick-and-place demonstration

# TurtleBot
agr_tb3                                  # TurtleBot 3
agr_tb4                                  # TurtleBot 4
agr_tb_drive                             # keyboard driving
agr_tb_demo                              # TurtleBot demonstration
```

Add `headless:=true` to a simulation command to run without the Gazebo window:

```bash
agr-sim ground tools:=true headless:=true
```

## Learning examples

Each platform follows the same structure:

| Package type | Purpose |
|---|---|
| `*_teleop` | Control the robot manually |
| `*_examples` | Learn one ROS or robotics concept at a time |
| `*_demos` | Run diagnostics and complete demonstrations |
| `*_labs` | Follow guided student exercises |

Example programs are numbered in the recommended learning order:

```bash
ros2 run agr_uav_examples 01_read_telemetry
ros2 run raisebot_examples 01_drive
ros2 run agr_legged_examples 01_read_joints
ros2 run agr_arm_examples 01_read_joints
ros2 run agr_tb_examples 01_read_odometry
```

Use terminal tab completion after a package name to see its available programs.

## AI and VLA tools

The automatic installer includes:

- CPU PyTorch and torchvision
- Ultralytics YOLO
- OpenAI Python library
- LeRobot and SmolVLA
- the SmolVLM2 vision backbone

LeRobot is installed separately in `~/raise_venvs/lerobot` so it does not
conflict with ROS 2. The provided `agr_vla`, `agr_record`, and `agr_finetune`
commands select that environment automatically.

The complete SmolVLA policy is downloaded the first time it is used. Examples
that call an online AI service require your own API key. The default PyTorch
installation uses the CPU; CUDA and GPU drivers are not installed.

## If something fails

Run these commands first:

```bash
cd ~/ros2_ws/src/agentic_robotics
./install.sh --check
agr-stop
```

If installation stopped because of a network or package error, run it again:

```bash
./install.sh
```

For a simulation problem, start the robot and run its diagnostic command from
the table above. When asking for help, include the diagnostic output and the
latest file from `~/ros2_ws/install_logs/`.

If Anaconda is active, continue using `agr-build` and `agr-sim`. These commands
automatically use the correct system Python required by ROS 2.

## Repository structure

```text
agentic_robotics/
├── install.sh             complete machine installer
├── setup.sh               workspace-only setup
├── agr-sim                start any robot
├── agr-stop               stop simulations
├── agr-build              rebuild the workspace
├── common/                interfaces shared by all platforms
├── uav/                   PX4 UAV packages and labs
├── ground/                RaiseBot packages and labs
├── legged/                Unitree Go2 packages
├── arm/                   UR5e arm packages
└── turtlebot/             TurtleBot 3 and 4 packages
```

## Advanced installation options

Most users only need `./install.sh`.

```bash
./install.sh --check       # check the installation only
./install.sh --no-shell    # do not update ~/.bashrc
./setup.sh                 # rebuild and configure an existing environment
./setup.sh --no-deps       # rebuild without installing apt packages
```

The installer provides ROS 2 Jazzy, Gazebo Harmonic, matching PX4 and
`px4_msgs` revisions, Micro XRCE-DDS Agent, robot dependencies, AI libraries,
and the workspace build. Existing PX4 source changes are never overwritten.

## PX4 notes for developers

- `agr-sim` handles the ROS and Gazebo environment automatically.
- PX4 message names may have version suffixes such as
  `/fmu/out/vehicle_status_v4`. Use `px4_topics.resolve()` instead of assuming
  an exact suffix.
- PX4 instance 0 has no namespace. Later instances use `px4_N`.
- PX4 output topics use BEST_EFFORT quality of service. Use
  `px4_topics.PX4_QOS` in subscribers.
- `vehicle_status` publishes when its value changes, so it is not a reliable
  liveness check. Use a periodic topic such as `battery_status`.

Available UAV airframes can be listed with:

```bash
ros2 run agr_uav_tools list_airframes
```

## License

MIT. Prof. Anis Koubaa — anis.koubaa@gmail.com
