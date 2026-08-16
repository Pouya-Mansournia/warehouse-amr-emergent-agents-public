from setuptools import find_packages, setup

package_name = "experiment_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pouya Mansournia",
    maintainer_email="p.mansournia@gmail.com",
    description=(
        "Phase 1 research instrumentation: experiment IDs, deterministic "
        "seeds, passive event logging, rosbag capture, and run summaries."
    ),
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "event_logger = experiment_manager.event_logger_node:main",
            "run_experiment = experiment_manager.run_experiment:main",
            "summarize_run = experiment_manager.summarize_run:main",
        ],
    },
)
