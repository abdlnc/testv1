from setuptools import find_packages, setup

package_name = 'mpu6050_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team25', # Or your name/email
    maintainer_email='team25@todo.todo',
    description='Simple MPU6050 IMU driver node',
    license='Apache-2.0', # Match the license in the script
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpu6050_node = mpu6050_driver.mpu6050_node:main',
        ],
    },
)