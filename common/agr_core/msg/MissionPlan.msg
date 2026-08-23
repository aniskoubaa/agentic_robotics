# A mission as produced by a planner (human, script, or LLM).
string      mission_id
string      objective        # the natural-language intent this plan came from
Waypoint[]  waypoints
bool        return_home
string      generator        # what produced it, e.g. "manual", "gpt-4o", "langgraph-v2"
