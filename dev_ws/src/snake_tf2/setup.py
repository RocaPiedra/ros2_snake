from setuptools import setup
import os
from glob import glob

package_name = 'snake_tf2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roc',
    maintainer_email='pablo_rocasg97@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'snake_tf2_broadcaster = snake_tf2.snake_tf2_broadcaster:main',
            'snake_tf2_listener = snake_tf2.snake_tf2_listener:main',
            'snake_tf2_tail_frame_broadcaster = snake_tf2.snake_tf2_tail_frame_broadcaster:main',
            'start_snake_service = snake_tf2.start_snake_service:main',
            'start_snake_tf2_broadcaster = snake_tf2.start_snake_tf2_broadcaster:main',
            'start_snake_tf2_listener = snake_tf2.start_snake_tf2_listener:main',
            'start_snake_tf2_tail_frame_broadcaster = snake.tf2.start_snake_tf2_tail_frame_broadcaster:main',
        ],
    },
)
