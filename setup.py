from setuptools import setup
import os
from glob import glob

package_name = 'rakuda_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),

        # Meshes: separo visual e collision
        (os.path.join('share', package_name, 'meshes', 'visual'),
            glob('meshes/visual/*')),
        (os.path.join('share', package_name, 'meshes', 'collision'),
            glob('meshes/collision/*')),

        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],

    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
