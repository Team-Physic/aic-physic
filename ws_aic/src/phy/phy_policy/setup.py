from setuptools import find_packages, setup

package_name = "phy_policy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    package_data={package_name: ["py.typed"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team Physic",
    maintainer_email="34768271+whyz-dev@users.noreply.github.com",
    description="Team Physic policies for AIC",
    license="Proprietary",
    extras_require={"test": ["pytest"]},
)
