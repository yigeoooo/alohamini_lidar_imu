from setuptools import find_packages, setup

package_name = "alohamini_nav_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AlohaMini",
    maintainer_email="todo@example.com",
    description="Bridge ROS2 cmd_vel/odom to the AlohaMini ZMQ host.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "bridge_node = alohamini_nav_bridge.bridge_node:main",
        ],
    },
)

