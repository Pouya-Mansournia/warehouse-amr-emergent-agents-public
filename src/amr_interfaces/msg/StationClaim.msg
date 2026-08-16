# Peer-to-peer station-claim broadcast for decentralized fleet coordination (master
# prompt spec section 7, Mode C / Phase 5). Published on the shared /fleet/agent/claim
# topic by every robot's fleet_coordination/decentralized_agent - there is no central
# broker; every agent subscribes to every other agent's claims and independently derives
# the same station-occupancy belief (see decentralized_agent.py's ClaimBook).

string robot_id
string station_name
string side           # "input" | "output"
float32 x
float32 y
float32 cost          # lower = more eager/suitable to claim this station right now
bool release          # true = this robot_id is releasing a station it previously claimed
builtin_interfaces/Time stamp
