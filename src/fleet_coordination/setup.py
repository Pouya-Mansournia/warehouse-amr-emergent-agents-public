from setuptools import find_packages, setup

package_name = "fleet_coordination"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/decentralized_agents.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pouya Mansournia",
    maintainer_email="p.mansournia@gmail.com",
    description=(
        "Phase 5 (Mode C): deterministic decentralized peer-to-peer fleet "
        "coordination, no LLM, no central broker."
    ),
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "decentralized_agent = fleet_coordination.decentralized_agent:main",
        ],
    },
)
