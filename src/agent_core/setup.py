from setuptools import find_packages, setup

package_name = "agent_core"

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
        "Phase 6: provider-independent AgentBackend decision interface "
        "(RuleAgent, ReplayAgent today; LLMAgent in Phase 7)."
    ),
    license="BSD",
    tests_require=["pytest"],
)
