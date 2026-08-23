# A single mission waypoint, in WGS-84.
float64 latitude_deg
float64 longitude_deg
float32 altitude_m          # AMSL — NOT relative-to-home. See MissionConstraints.
string  action              # "" | takeoff | land | loiter | survey | rth
float32 hold_s              # dwell time at the waypoint, 0 = fly through
