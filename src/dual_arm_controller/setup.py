from setuptools import setup
from glob import glob
import os

package_name = 'dual_arm_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Screening Candidate',
    maintainer_email='user@example.com',
    description='Dual-arm Cartesian controller for synchronized CRX-10iA robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cartesian_controller = dual_arm_controller.cartesian_controller:main',
            'dual_arm_synchronizer = dual_arm_controller.dual_arm_synchronizer:main',
        ],
    },
)