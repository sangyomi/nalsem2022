import glob
import os

from setuptools import setup

package_name = 'mecha_autoship_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob.glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auto-ship1',
    maintainer_email='auto-ship1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mecha_autoship_image_pub_node = mecha_autoship_camera.mecha_autoship_image_pub_node:main",
            "mecha_autoship_image_sub_node = mecha_autoship_camera.mecha_autoship_image_sub_node:main",
            "mecha_autoship_image_color_filter_node = mecha_autoship_camera.mecha_autoship_image_color_filter_node:main",
            "mecha_autoship_filtered_image_sub_node = mecha_autoship_camera.mecha_autoship_filtered_image_sub_node:main",
        ],
    },
)
