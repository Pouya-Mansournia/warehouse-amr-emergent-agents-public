"""Station table - MUST mirror amr_ros_dg/scripts/fleet_manager.py's copy exactly (both
in turn mirror worlds/warehouse_fleet_world.sdf - see that file's module docstring for
the full picture). Duplicated rather than imported because fleet_manager.py is installed
as a standalone ament_cmake script, not an importable library module, and introducing a
shared library package for one small table is out of scope for Phase 5. If you change
one copy, change both.
"""

STATIONS = []
for i, y in enumerate(range(-9, 10, 2), start=1):
    STATIONS.append((f"input_station_{i}", "input", -6.9, float(y)))
for i, y in enumerate(range(-9, 10, 2), start=1):
    STATIONS.append((f"output_station_{i}", "output", 6.9, float(y)))

STATIONS_BY_SIDE = {
    "input": [s for s in STATIONS if s[1] == "input"],
    "output": [s for s in STATIONS if s[1] == "output"],
}
OPPOSITE_SIDE = {"input": "output", "output": "input"}
