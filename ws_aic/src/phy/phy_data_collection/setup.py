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
    description="Randomized img2pos data collection for AIC",
    license="Proprietary",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "collect_portoffset_randomization_data = phy_data_collection.main:main",
            "evaluate_img2pos = phy_data_collection.evaluation:main",
        ],
    },
)
