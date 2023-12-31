import os
from glob import glob
from setuptools import setup

package_name = 'sailbot_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/'+package_name,  ['web/boat_icon.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matthew',
    maintainer_email='matthew@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_input = sailbot_test.controller_input:main',
            'sailbot_gui = sailbot_test.sailbot_gui:main',
            'compass = sailbot_test.compass:main',
        ],
    },
)
