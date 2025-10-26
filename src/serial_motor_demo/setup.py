from setuptools import setup # Removed find_packages for now
# import os # Keep if using data_files with os.path.join
# from glob import glob # Keep if using data_files with glob

package_name = 'serial_motor_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], # Explicitly list the package directory name
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add these lines if you create launch or config directories later
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='newans', # Or your details
    maintainer_email='josh.newans@gmail.com', # Or your details
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = serial_motor_demo.gui:main',
            'driver = serial_motor_demo.driver:main',
            'twist_to_motor = serial_motor_demo.twist_to_motor_command:main'
        ],
    },
)