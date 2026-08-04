from setuptools import find_packages, setup

package_name = "phy_data_collection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    package_data={package_name: ["py.typed"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team Physic",
    maintainer_email="34768271+whyz-dev@users.noreply.github.com",
    description="Scenario randomization, automated capture, and validation tools for AIC datasets",
    license="Proprietary",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "collect_lerobot_data = phy_data_collection.collect_lerobot_data:main",
            "collect_lerobot_data_aarch = phy_data_collection.collect_lerobot_data_aarch:main",
            "collect_portoffset_randomization_data = phy_data_collection.collect_portoffset_randomization_data:main",
            "collect_yolo_data_aarch = phy_data_collection.collect_yolo_data_aarch:main",
            "plot_scenario_randomization = phy_data_collection.plot_scenario_randomization:main",
            "validate_portoffset_trial = phy_data_collection.validate_portoffset_trial:main",
        ],
    },
)
