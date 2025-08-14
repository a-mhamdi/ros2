from setuptools import find_packages, setup

package_name = 'demo_ros_py'

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
    maintainer='a-mhamdi',
    maintainer_email='a_mhamdi@outlook.com',
    description='Demos in ROS2 using Python',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_pub = demo_ros_py.demo_pub:main',
            'demo_sub = demo_ros_py.demo_sub:main'
        ],
    },
)
