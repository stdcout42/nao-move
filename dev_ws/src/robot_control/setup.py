from setuptools import setup
import os
from glob import glob

package_name = 'robot_control'
utils = f'{package_name}/utils'
keypoint_classifier= f'{package_name}/utils/keypoint_classifier'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, utils, keypoint_classifier],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='sammm.sh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'sim_controller = robot_control.sim_controller:main',
          'cam_controller = robot_control.cam_controller:main',
          'speech_controller = robot_control.speech_controller:main',
          'gui_controller = robot_control.gui_controller:main',
          'video_demo_controller = robot_control.video_demo_controller:main',
        ],
    },
)
