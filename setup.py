from setuptools import setup

# add by nishi
import os
from glob import glob

package_name = 'uvc_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # add by nishi
        (os.path.join('share', package_name), glob('launch/.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nishi',
    maintainer_email='non@netosa.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "hello_node = hello.hello_node:main",
        ],
    },
)
