from setuptools import setup

package_name = 'world_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/system.launch.py', 'launch/order.launch.py', 'launch/viewer.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='subash',
    maintainer_email='subash@todo.todo',
    description='Robot2D world simulation + Module 6 order interface',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world_node = world_sim.world_node:main',
            'goal_nav = world_sim.goal_nav:main',
            'viewer = world_sim.viewer:main',
            'order_cli = world_sim.order_cli:main',
            'human_sim = world_sim.human_sim:main',
            'human_predictor = world_sim.human_predictor:main',
            'metrics_logger = world_sim.metrics_logger:main',
        ],
    },
)
