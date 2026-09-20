from setuptools import find_packages, setup
from glob import glob

package_name = 'mcnm_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joyandai',
    maintainer_email='joyandai@todo.todo',
    description='Bringup launch and task orchestration for mcnm_robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom2tf = mcnm_robot_bringup.odom2tf:main',
            'task_executor = mcnm_robot_bringup.task_executor:main',
        ],
    },
)
