from glob import glob
from setuptools import setup

package_name = 'marl_bot'


data_files = [
    # Always install package.xml
    ('share/' + package_name, ['package.xml']),
    # Configs
    ('share/' + package_name + '/config', glob('config/*')),
    ('share/' + package_name + '/params', glob('params/*')),
    # Launch files (your real_launch)
    ('share/' + package_name + '/real_launch', glob('real_launch/*.launch.py')),
    # Sim launch, multi launch
    ('share/' + package_name + '/sim_launch', glob('sim_launch/*.launch.py')),
    ('share/' + package_name + '/sim_multi_robot_launch', glob('sim_multi_robot_launch/*.launch.py')),
    # URDF / xacro / description
    ('share/' + package_name + '/description', glob('description/*')),
    # Worlds
    ('share/' + package_name + '/worlds', glob('worlds/*')),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kamal',
    maintainer_email='kamalu211211@gmail.com',
    description='Multi-agent robot Python node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpm_node = marl_bot.rpm_node:main',  # << rpm_node.py must have a main()
        ],
    },
)
