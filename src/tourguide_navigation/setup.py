from setuptools import find_packages, setup

package_name = 'tourguide_navigation'

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
    maintainer='husarion',
    maintainer_email='cshukla@ucsd.edu',
    description='Extra nagvigation code',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "ros2_nav_bridge = tourguide_navigation.ros2_nav_bridge:main",
            "apriltag_http_server = tourguide_navigation.apriltag_http_server:main",
        ],
    },
)
