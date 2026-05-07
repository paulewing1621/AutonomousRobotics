from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wpa_cli'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtlebot_admin',
    maintainer_email='cedric.pradalier@georgiatech-metz.fr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wpa_cli_node = wpa_cli.wpa_cli_node:main',
            'wpa_cli_sim = wpa_cli.wpa_cli_sim:main',
            'wpa_cli_const = wpa_cli.wpa_cli_const:main',
        ],
    },
)
