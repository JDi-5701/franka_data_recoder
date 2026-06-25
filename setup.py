from setuptools import setup
from glob import glob

package_name = 'franka_data_recorder'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Junsheng Ding',
    maintainer_email='jding@uni-bremen.de',
    author='Junsheng Ding',
    author_email='ding@fortiss.de',
    description='Record Franka teleop episodes into the LeRobot dataset format.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'recorder = franka_data_recorder.recorder_node:main',
            'gui = franka_data_recorder.gui_node:main',
            'fake = franka_data_recorder.fake_publisher:main',
        ],
    },
)
