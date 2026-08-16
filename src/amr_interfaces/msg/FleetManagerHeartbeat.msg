# Centralized -> decentralized failover. Published by fleet_manager.py on
# /fleet/manager/heartbeat at a fixed period for as long as it's alive - its own process
# dying (Mode B kill, or a real crash) simply stops these messages, which is what a
# DecentralizedAgent in DORMANT (failover-standby) state watches for. Content is
# deliberately minimal (a heartbeat's only job is "did a message arrive recently",
# never "what does it say") - see fleet_coordination/heartbeat.py's HeartbeatMonitor,
# which only ever looks at arrival time, never at any field below.

builtin_interfaces/Time stamp
