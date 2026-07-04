from setuptools import setup

package_name = 'melfa_rv5as_masterclass'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='techs',
    maintainer_email='techs@example.com',
    description='A brief description of the package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_pub = melfa_rv5as_masterclass.cam_pub:main',
        ],
    },
)

