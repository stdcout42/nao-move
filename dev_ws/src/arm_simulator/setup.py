from setuptools import setup
import os
from glob import glob

package_name = 'arm_simulator'
utils = 'arm_simulator/utils'
keypoint_classifier= 'arm_simulator/utils/keypoint_classifier'
point_history_classifier = 'arm_simulator/utils/point_history_classifier'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, utils, keypoint_classifier, point_history_classifier],
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
            'sim_listener = arm_simulator.sim_listener:main',
            'gesture_talker = arm_simulator.gesture_talker:main',
            'speech_talker = arm_simulator.speech_talker:main',
        ],
    },
)
