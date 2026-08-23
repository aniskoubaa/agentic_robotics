# The envelope a mission must fit inside. Supplied by the operator//platform,
# NOT by the planner — a planner that supplies its own constraints can trivially
# declare itself valid.
float64[]  geofence_lat_deg      # polygon vertices, same length as lon array
float64[]  geofence_lon_deg
float32    altitude_min_m        # AMSL
float32    altitude_max_m
float32    battery_reserve_pct   # refuse to plan below this
float32    max_duration_s
float64    home_lat_deg
float64    home_lon_deg
float32    home_alt_m

# Platform capability — a fixed-wing cannot hover, so "loiter here" is
# infeasible for it and feasible for a multirotor. Verification is not
# airframe-independent.
string     airframe_class        # multirotor | vtol | fixedwing
bool       can_hover
float32    min_airspeed_ms       # 0 for multirotor
float32    min_turn_radius_m     # 0 for multirotor
