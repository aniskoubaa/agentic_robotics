# One reason a mission was rejected.
#
# `string[] violations` is not enough to act on: a repair step needs to know
# WHICH rule fired, WHERE, and BY HOW MUCH. Every field here exists so a
# downstream repair can be constraint-preserving rather than guesswork.
uint8 SEVERITY_INFO=0
uint8 SEVERITY_WARN=1
uint8 SEVERITY_FATAL=2

string  rule_id             # e.g. "geofence.polygon", "battery.reserve"
uint8   severity
int32   waypoint_index      # -1 when the violation is about the mission as a whole
float64 measured            # what the mission asks for
float64 limit               # what the constraint allows
string  units
string  message             # human-readable, for logs and reports
