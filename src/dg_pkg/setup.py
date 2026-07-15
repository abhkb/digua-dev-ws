import glob
import os

from setuptools import find_packages, setup

package_name = 'dg_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob.glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'display_text'),
            glob.glob('config/display_text/*.txt')),
        (os.path.join('share', package_name, 'behavior_trees'),
            glob.glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DG Robot Team',
    maintainer_email='maintainer@example.com',
    description='Competition ROS2 package for the Digua robotics smart-car project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'qrcode_detector = dg_pkg.qrcode_detector:main',
        'qr_result_display = dg_pkg.qr_result_display:main',
        'nav_cruise = dg_pkg.nav_cruise:main',
        ],
    },
)
