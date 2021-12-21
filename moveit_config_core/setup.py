from setuptools import setup, find_packages
from glob import glob

package_name = "moveit_config_core"

setup(
    name=package_name,
    version="2.3.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    license="BSD",
    author="David V. Lu!!",
    author_email="davidvlu@gmail.com",
    description="Core files that other MoveIt configs can build on",
)
