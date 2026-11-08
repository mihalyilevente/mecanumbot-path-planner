from glob import glob
import os

from setuptools import setup

package_name = 'mecanumbot_planner_ui'


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
    package_data={
        package_name: [
            'templates/*.html',
            'static/*',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        *tree('config'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fahrad Namazov',
    maintainer_email='fahradnamazov@gmail.com',
    description='Browser UI for Mecanumbot planner criteria selection and driving.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_ui = mecanumbot_planner_ui.web_node:main',
        ],
    },
)
