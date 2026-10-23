from setuptools import setup
from glob import glob
import os

package_name = 'ai407_maze_project'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AI407 Team',
    maintainer_email='student@example.com',
    description='AI407 TurtleBot3 maze retrieval simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'maze_task_node = ai407_maze_project.maze_task_node:main',
        ],
    },
)
