from setuptools import setup

package_name = 'roscam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', [
            'resource/test.yaml',
            'resource/camera_matrix.npy',
            'resource/dist_coeffs.npy',
            'resource/RoboDK-Camera-Settings.yaml',
        ]),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv-bridge', 'numpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A ROS 2 package for publishing camera images',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_pub = roscam.cam_pub:main',
        ],
    },
)

