# Two fixes for a clean install

Prepared by Mazen Khalid Hamze, 11 October 2026, on branch `fix/clean-install`.

Both problems were found while installing the UAV simulation on Ubuntu 24.04 on
1 October, and both stop `install.sh` on a machine that has never seen this
repository before. Ahad reported the same two.

## 1. `rosdep` cannot resolve `ament_python`

**What happened.** `install.sh` line 153 runs

```
rosdep install --from-paths "$AGR_SRC" "${AGR_WS}/src/px4_msgs" --ignore-src --rosdistro jazzy -y
```

and it stopped with, for many packages:

```
Cannot locate rosdep definition for [ament_python]
```

No dependencies were installed, so the build that follows had nothing to build
against.

**Why.** 22 packages declared

```xml
<buildtool_depend>ament_python</buildtool_depend>
```

`ament_python` is not a rosdep key — `rosdep resolve ament_python` answers "no
rosdep rule". It is a colcon build type, and colcon reads it from the export
section, which all 22 packages already have:

```xml
<export><build_type>ament_python</build_type></export>
```

So the line was redundant as well as unresolvable.

**Change.** Removed that one line from all 22 package.xml files. Nothing else in
them changed, and each still declares its build type in `<export>`.
`agr_uav_bringup` never carried the line and has always built correctly; the
other 22 now match it.

## 2. `agr_arm_description` installs a directory that is not in the clone

**What happened.** The build stopped with:

```
file INSTALL cannot find ".../arm/agr_arm_description/config": No such file or directory.
```

**Why.** `arm/agr_arm_description/CMakeLists.txt` installs `urdf config`, but the
config directory is empty, and Git does not store empty directories. It exists
on a machine where it was created by hand and is absent from every fresh clone.

**Change.** Added `arm/agr_arm_description/config/.gitkeep`. This is already how
the other two description packages keep their empty config directories:

| Package | config directory in a clone |
|---|---|
| `ground/raisebot_description` | present, holds `.gitkeep` |
| `legged/agr_legged_description` | present, holds `.gitkeep` |
| `arm/agr_arm_description` | was missing, now holds `.gitkeep` |

## How this was tested

On a fresh clone of the branch, not on the working copy where the directory had
already been created by hand.

**Before, on a clean clone of `main`:**

```
agr_arm_examples: Cannot locate rosdep definition for [ament_python]
agr_uav_demos:    Cannot locate rosdep definition for [ament_python]
...
file INSTALL cannot find ".../agr_arm_description/config": No such file or directory.
```

**After, on a clean clone of `fix/clean-install`:**

```
$ rosdep install --from-paths . ~/ros2_ws/src/px4_msgs --ignore-src --rosdistro jazzy -y
#All required rosdeps installed successfully

$ cmake -S arm/agr_arm_description -B build && cmake --install build --prefix install
-- Installing: .../share/agr_arm_description/package.xml
```

Both package.xml files were checked after editing: all 22 still parse as XML and
all 22 still declare `ament_python` as their build type in `<export>`.

**Not tested:** a full `./install.sh` from nothing. That downloads ROS 2, Gazebo,
PX4 and PyTorch and takes hours, and the machine already has them. What was
tested is the two steps that failed, each on a fresh clone, before and after.
