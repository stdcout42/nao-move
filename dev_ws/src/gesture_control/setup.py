from setuptools import setup
import os
from glob import glob

package_name = 'gesture_control'
utils = f'{package_name}/utils'
keypoint_classifier= f'{package_name}/utils/keypoint_classifier'
point_history_classifier = f'{package_name}/utils/point_history_classifier'

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
          'sim_listener = gesture_control.sim_subscriber:main',
          'pose_talker = gesture_control.pose_publisher:main',
        ],
    },
)
