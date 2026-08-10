from setuptools import find_packages, setup

package_name = "bounding_box_tool"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["py.typed"]},
    install_requires=["setuptools", "PyYAML", "PyQt5>=5.15,<6"],
    zip_safe=True,
    maintainer="Team Physic",
    maintainer_email="34768271+whyz-dev@users.noreply.github.com",
    description="Read-only PyQt viewer for AIC YOLO-pose annotations",
    license="Proprietary",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "view_annotations = bounding_box_tool.main:main",
        ],
    },
)
