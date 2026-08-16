# legacy/ — export history

Every previous export/migration artifact, kept for traceability. Nothing here is used
by the active package (`../urdf/amr_ros_dg.urdf.xacro` and `../meshes/`) — it exists so
any past decision can be re-checked against the original data.

```
legacy/
├── ros1_original_package/          the original ROS 1 (catkin) SW2URDF export, untouched
│   ├── package.xml                 catkin package.xml (format 2)
│   ├── CMakeLists.txt               catkin_package() build
│   ├── amr_ros_dg_original.urdf     the raw, non-xacro URDF as exported
│   ├── amr_ros_dg_mass_properties.csv   full per-link mass/COM/inertia/joint CSV
│   ├── config/                      original joint_names_AMR-ROS-DG.yaml
│   └── launch/                      original display.launch / gazebo.launch (roslaunch XML)
│
└── broken_export_2_mm_scale_bad_baselink_origin/
    all 11 STL meshes from the SECOND export attempt: correctly isolated per-part,
    but in millimeters (not meters) and with base_link.STL exported outside its own
    coordinate system (~80cm origin offset). Superseded by the current meshes/ set.
```

## Why keep this

- `ros1_original_package/amr_ros_dg_mass_properties.csv` is the primary source for every
  mass/inertia/joint-origin number used in the current xacro — if a value in the xacro
  is ever questioned, this file is the ground truth to check it against.
- The broken mesh set documents a real SolidWorks SW2URDF exporter failure mode
  (STL export not respecting the assigned per-link coordinate system) that's worth
  remembering if it happens again on a future export.

## Not kept here

The very first export attempt (every link's STL containing the *entire* assembly
geometry) was overwritten by the user directly in SolidWorks before a backup could be
made, so no copy of that mesh set exists. It's documented in the main package README's
"Mesh/unit history" section instead.
