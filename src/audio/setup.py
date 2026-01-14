from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'audio'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/'+package_name,['audio/audio.py', 'audio/audio_subscriber.py', 'audio/volume_processor.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Audio capture and publishing package for Duckiebot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'audio_publisher = audio.audio:main',
            'audio_subscriber = audio.audio_subscriber:main',
            'volume_processor = audio.volume_processor:main'
        ],
    },
)
