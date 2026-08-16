# Centralized -> decentralized failover. Published by a
# DecentralizedAgent whenever it transitions between DORMANT (failover standby - not
# touching Nav2/claims at all, central fleet_manager has coordination) and its normal
# active cycle, on /fleet/agent/coordination_mode. event_logger_node.py logs every one
# of these as a COORDINATION_MODE_CHANGED event in events.jsonl.

string robot_id
string from_mode          # "CENTRALIZED" | "DECENTRALIZED"
string to_mode             # "CENTRALIZED" | "DECENTRALIZED"
string reason              # e.g. "CENTRAL_MANAGER_TIMEOUT"
builtin_interfaces/Time stamp
