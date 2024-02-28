from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'cam_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Karoline Heiwolt',
    maintainer_email='contact@karoline.heiwolt.de',
    description='Package containing configurations for a Canon EOS DSLR camera mounted onto a Kinova Gen3 robot arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_camera_tf2_broadcaster = cam_config.static_camera_tf2_broadcaster:main',
        ],
    },
)