from setuptools import find_packages, setup

package_name = 'listen_talk_pkg'

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
    maintainer='A. Mhamdi',
    maintainer_email='a_mhamdi@outlook.com',
    description='An Introduction To ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'listener = listen_talk_pkg.listen:main',
        'talker = listen_talk_pkg.talk:main'
        ],
    },
)
