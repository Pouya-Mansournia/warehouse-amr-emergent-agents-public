# Notice

This package is imported unmodified from
[warehouse-amr-ros2](https://github.com/Pouya-Mansournia/warehouse-amr-ros2)
(commit `4e8aea5`, 2026-08-11), authored by Pouya Mansournia, who also
maintains this repository.

It is kept as the clean engineering baseline: CAD-grounded
differential-drive AMR description, ROS 2 Jazzy + Gazebo Harmonic
simulation, per-robot SLAM Toolbox + Nav2, `fleet_manager.py`
(centralized task assignment + station reservation), and the Flask web
dashboard.

Changes made for the research platform live outside this directory
(`src/experiment_manager/`, `config/`, `docs/`) so this package can be
diffed against the upstream baseline at any time to confirm behavior is
unchanged. If this package is ever modified for the research platform,
document the change and the reason in this file.
