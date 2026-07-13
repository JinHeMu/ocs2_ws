from setuptools import find_packages, setup

package_name = "esdf_simple_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/simple_nav.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Simple A* planner and Pure Pursuit controller for ESDF occupancy maps.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "simple_nav = esdf_simple_nav.simple_nav_node:main",
        ],
    },
)
