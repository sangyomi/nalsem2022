import glob
import os

from setuptools import setup

package_name = "nalsem_A"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="parallels",
    maintainer_email="dhksrl0508@naver.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [          
            "nalsem_lider = nalsem_A.nalsem_lider:main",
            "nalsem_main_node = nalsem_A.nalsem_main_node:main",
        ],
    },
)