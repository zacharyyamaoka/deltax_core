from setuptools import setup
import os
from glob import glob

package_name = 'deltax_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('scripts/*.py')),
        (os.path.join('share', package_name), glob('deltax_driver/*.py')),
        (os.path.join('share', package_name), glob('deltax_driver/*.pyd')),
        (os.path.join('Lib/site-packages', package_name), glob('deltax_driver/*.pyd')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Than',
    maintainer_email='jonyvantha@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = deltax_driver.state_publisher:main',
            'robot_driver = deltax_driver.robot_driver:main'
        ],
    },
)
