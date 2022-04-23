from setuptools import setup

package_name = 'arm_simulator'
keypoint_classifier = 'arm_simulator/keypoint_classifier'
point_history_classifier = 'arm_simulator/point_history_classifier'
utils = 'arm_simulator/utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, keypoint_classifier, point_history_classifier, utils],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'sim_listener = arm_simulator.sim_subscriber:main',
            'gesture_talker = arm_simulator.gesture_publisher:main',
        ],
    },
)
