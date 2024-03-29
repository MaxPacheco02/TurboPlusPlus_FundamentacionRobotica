import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'Final_challenge'

setup(
    name=package_name,
    version='0.0.0', 
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel815',
    maintainer_email='daniel815@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'final_challenge = Final_challenge.final_challenge:main'
        ],
    },
)
