from setuptools import find_packages, setup

package_name = "xarm_control"

setup(
    name=package_name,
    version="0.1.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bourinto",
    maintainer_email="bourin.tomeo@gmail.com",
    description="A lightweight ROS 2 Humble package for real-time Cartesian control of the xArm6",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "xarm_control = xarm_control.xarm_control:main",
            "demo_circle_publisher = xarm_control.demo_circle_publisher:main",
            "teleop_keyboard = xarm_control.teleop_keyboard:main",
        ],
    },
)
