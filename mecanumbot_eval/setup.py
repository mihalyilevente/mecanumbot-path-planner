from setuptools import setup

package_name = 'mecanumbot_eval'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fahrad Namazov',
    maintainer_email='fahradnamazov@gmail.com',
    description='Time and energy comparison of the Mecanumbot Nav2 planners.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compare_runs = mecanumbot_eval.compare_runs:main',
        ],
    },
)
