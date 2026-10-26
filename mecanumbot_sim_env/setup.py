from glob import glob
import os

from setuptools import setup

package_name = 'mecanumbot_sim_env'


def tree(directory):
    """Install every file under directory, keeping its relative layout."""
    entries = []
    for root, _, files in os.walk(directory):
        if files:
            target = os.path.join('share', package_name, root)
            entries.append((target, [os.path.join(root, name) for name in files]))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        *tree('config'),
        *tree('maps'),
        *tree('scenarios'),
        *tree('behavior_trees'),
        *tree('rviz'),
        *tree('models'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zoltán Ozsvárt',
    maintainer_email='75853960+ozsvartzoltan@users.noreply.github.com',
    description='Simulation environment and planner harness for Mecanumbot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plan_scenarios = mecanumbot_sim_env.plan_scenarios:main',
            'make_scenarios = mecanumbot_sim_env.make_scenarios:main',
            'make_world = mecanumbot_sim_env.make_world:main',
            'make_test_maps = mecanumbot_sim_env.make_test_maps:main',
            'drive_scenarios = mecanumbot_sim_env.drive_scenarios:main',
        ],
    },
)
