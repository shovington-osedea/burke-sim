from setuptools import find_packages, setup


package_name = 'aircraft_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/aircraft_pose.yaml']),
        ('share/' + package_name + '/config', ['config/perimeter_path.yaml']),
        ('share/' + package_name + '/config', ['config/controller.yaml']),
        ('share/' + package_name + '/launch',
         ['launch/fixed_perimeter_follow.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    test_suite='test',
    zip_safe=True,
    maintainer='Burk-e simulation maintainers',
    maintainer_email='maintainers@example.invalid',
    description='Aircraft-relative navigation utilities for the Burk-e simulation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odometry_baseline_check = '
            'aircraft_navigation.odometry_baseline_check:main',
            'aircraft_frame_publisher = '
            'aircraft_navigation.aircraft_frame_publisher:main',
            'perimeter_path_publisher = '
            'aircraft_navigation.perimeter_path_publisher:main',
            'pure_pursuit_follower = '
            'aircraft_navigation.pure_pursuit_follower:main',
        ],
    },
)
