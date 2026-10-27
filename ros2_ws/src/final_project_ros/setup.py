from setuptools import find_packages, setup
from glob import glob

package_name = 'final_project_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/color_tree.yml', 'models/color_tree.json', 'models/learned_grasp_policy.json']),
        ('share/' + package_name + '/launch', ['launch/simple_delivery.launch.py', 'launch/physical_delivery.launch.py']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/worlds', [
            'worlds/conveyor_sorting.world',
            'worlds/physical_delivery.world',
            'worlds/simple_delivery.world',
            'worlds/two_target_delivery.world',
            'worlds/multi_color_delivery.world',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Simple ROS 2 color-pick delivery demo for final project.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_demo = final_project_ros.record_demo:main',
            'color_pick_demo = final_project_ros.color_pick_demo:main',
            'gazebo_delivery_demo = final_project_ros.gazebo_delivery_demo:main',
            'delivery_controller = final_project_ros.delivery_controller:main',
            'physical_delivery = final_project_ros.physical_delivery:main',
            'conveyor_sorting = final_project_ros.conveyor_sorting:main',
        ],
    },
)
