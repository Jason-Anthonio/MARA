import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mararos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jason-anthonio',
    maintainer_email='jason.anthonio@binus.ac.id',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_server = mararos.yolo_server:main',
            'llm_client = mararos.llm_client:main',
            'hand_server = mararos.hand_server:main',
            'camera_pub = mararos.camera_pub:main',
            'yolodepth_server = mararos.yolodepth_server:main',
            'stereo_viewer = mararos.stereo_viewer:main',
        ],
    },
)
