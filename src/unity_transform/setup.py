from setuptools import find_packages, setup

package_name = "unity_transform"

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
    maintainer="bourinto",
    maintainer_email="bourin.tomeo@gmail.com",
    description="Convert pose data from Unity into commands for the xArm6 robot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vive_absolute_motion = unity_transform.vive_absolute_motion:main",
            "vive_relative_motion = unity_transform.vive_relative_motion:main",
        ],
    },
)
