from setuptools import setup
import os
from glob import glob

package_name = 'doorlock_decoder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rm',
    maintainer_email='rm@example.com',
    description='H.264/HEVC video decoder and SharkDataServer bridge for doorlock sniper',
    license='MIT',
    entry_points={
        'console_scripts': [
            'decoder_node = doorlock_decoder.video_decoder_node:main',
            'shark_bridge_node = doorlock_decoder.shark_bridge_node:main',
        ],
    },
)
