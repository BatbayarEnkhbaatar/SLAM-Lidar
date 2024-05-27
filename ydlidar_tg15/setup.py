import glob
import os

from setuptools import setup
from glob import glob

package_name = "ydlidar_tg15"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", 
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Batbayar Enkhbaatar",
    maintainer_email="baggi@haanvision.com",
    description="YDLiDAR TG15 example for ROS 2",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["ydlidar_tg15_node = ydlidar_tg15_node.ydlidar_tg15:main"],
    },
)
