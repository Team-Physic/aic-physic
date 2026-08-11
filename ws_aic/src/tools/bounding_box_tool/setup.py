from setuptools import find_packages, setup

package_name = "bounding_box_tool"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["py.typed"]},
    install_requires=[
        "setuptools",
        "PyYAML",
        "PyQt5>=5.15,<6",
        "numpy>=1.25,<3",
        "opencv-python-headless>=4.8",
    ],
    zip_safe=True,
    maintainer="Team Physic",
    maintainer_email="34768271+whyz-dev@users.noreply.github.com",
    description="PyQt editor for AIC YOLO-pose annotations and visibility",
    license="Proprietary",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "view_annotations = bounding_box_tool.main:main",
        ],
    },
)
