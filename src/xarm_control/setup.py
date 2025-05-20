from setuptools import find_packages, setup

package_name = 'xarm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoshidalab',
    maintainer_email='simeoncapy@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'ee_pose_controller = xarm_control.ee_pose_controller:main',
        'ee_demo_sinus_publisher = xarm_control.ee_demo_sinus_publisher:main',
    ],
},

)
