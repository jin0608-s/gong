from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'gong_basic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        ("share/" + package_name + "/launch", glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boyfriend51',
    maintainer_email='luckymijin0608@gmail.com',
    description='gongju university ROS2 basic library',
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "simple_pub = gong_basic.simple_pub:main",
            "class_pub = gong_basic.class_pub:main",
            "class_sub = gong_basic.class_sub:main",
            "header_pub = gong_basic.header_pub:main",
            
            'mpub = gong_basic.mpub:main',
            'tpub = gong_basic.tpub:main',
            'm1_sub = gong_basic.m1_sub:main',
            'm2_sub = gong_basic.m2_sub:main',
            'mt_sub = gong_basic.mt_sub:main',
            
            'mv_turtle = gong_basic.mv_turtle:main',
            'qos_test_pub = gong_basic.qos_test_pub:main',
            'qos_test_sub = gong_basic.qos_test_sub:main',
        ],
    },
)
