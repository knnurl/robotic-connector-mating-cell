from setuptools import setup

package_name = 'roscam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kadir Ural',
    maintainer_email='k.ural1@salford.ac.uk',
    description='ArUco marker pose publisher for the connector-mating cell. '
                'Publishes the marker pose as PoseStamped in the camera '
                'optical frame; intrinsics come from camera_info.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_pub = roscam.cam_pub:main',
            'handeye_calib = roscam.handeye_calib:main',
            'connector_pose = roscam.connector_pose:main',
        ],
    },
)
