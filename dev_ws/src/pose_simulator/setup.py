from setuptools import setup
import os
from glob import glob

package_name = 'pose_simulator'
utils = f'{package_name}/utils'
keypoint_classifier= f'{package_name}/utils/keypoint_classifier'
point_history_classifier = f'{package_name}/utils/point_history_classifier'

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
          'sim_listener = pose_simulator.sim_subscriber:main',
          'pose_talker = pose_simulator.pose_publisher:main',
          'speech_talker = pose_simulator.speech_publisher:main',
        ],
    },
)
