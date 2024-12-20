from setuptools import setup
import os
from glob import glob

package_name = 'fishbot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'world'), glob('world/**')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/**')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zen',
    maintainer_email='zen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rotation = fishbot_description.rotation:main"
        ],
    },
)
