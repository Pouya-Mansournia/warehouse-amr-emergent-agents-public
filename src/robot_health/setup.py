from setuptools import find_packages, setup

package_name = "robot_health"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/health.launch.py"]),
        (
            "share/" + package_name + "/config/faults",
            ["config/faults/example_schedule.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pouya Mansournia",
    maintainer_email="p.mansournia@gmail.com",
    description=(
        "Phase 2 research instrumentation: per-robot operational-health publisher "
        "and fault injection for the warehouse-amr-emergent-agents fleet."
    ),
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "health_monitor = robot_health.health_monitor:main",
            "fault_injector = robot_health.fault_injector:main",
        ],
    },
)
