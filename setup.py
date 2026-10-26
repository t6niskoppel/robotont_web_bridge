from glob import glob
from setuptools import setup

setup(
    name='robotont_web_bridge',
    version='0.0.1',
    packages=['robotont_web_bridge'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robotont_web_bridge']),
        ('share/robotont_web_bridge', ['package.xml']),
        ('share/robotont_web_bridge/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Tõnis Koppel',
    maintainer_email='t6niskoppel@gmail.com',
    description='ROS 2 API for Robotont web dashboard and Blockly',
    license='Apache-2.0',
    entry_points={
        'console_scripts': ['web_bridge = robotont_web_bridge.web_bridge:main'],
    },
)
