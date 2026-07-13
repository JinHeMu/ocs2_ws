import os
from glob import glob
from setuptools import setup

package_name = "tracer_jaka_mujoco"


def get_data_files(directory):
    data_files = []

    for root, dirs, files in os.walk(directory):
        if files:
            source_files = [
                os.path.join(root, file)
                for file in files
            ]

            install_dir = os.path.join(
                "share",
                package_name,
                root
            )

            data_files.append((install_dir, source_files))

    return data_files

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            os.path.join("share", package_name),
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ] + get_data_files("urdf")
        + get_data_files("models"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="ROS2 <-> MuJoCo bridge with LiDAR and depth camera",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mujoco_bridge = tracer_jaka_mujoco.mujoco_bridge_node:main",
        ],
    },
)
