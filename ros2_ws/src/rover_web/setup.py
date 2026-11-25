from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/static', glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='Browser control page for the RoverC.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rover_web = rover_web.web_node:main',
        ],
    },
)
