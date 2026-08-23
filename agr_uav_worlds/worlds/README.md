# Worlds

PX4 ships its own worlds under
`PX4-Autopilot/Tools/simulation/gz/worlds/` — `default`, `baylands`, `forest`,
`ridge`, `walls`, `windy`, `lawn`, `aruco`. Pass any of them by name:

```bash
ros2 launch agr_uav_bringup single.launch.py world:=windy
```

`default` is the reference for reproducible runs. `windy` is the useful stress
case: wind changes how much battery a mission actually costs, which is exactly
the kind of constraint a paper-only feasibility check misses.

Course-specific worlds authored here go in this folder as `.sdf`. Adding one
means dropping the file in and putting it on `GZ_SIM_RESOURCE_PATH`; nothing in
the launch files needs to change.
