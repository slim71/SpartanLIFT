import os
from glob import glob
from setuptools import setup

package_name = "odst"

setup(
    name=package_name,
    version="0.4.4",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        # Include all config files
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        # Include all scripts from the temp folder
        (
            os.path.join("share", package_name, "temp"),
            glob(os.path.join("temp", "*.sh")),
        ),
        # Include all scripts from the resource folder
        (
            os.path.join("share", package_name, "resource"),
            glob(os.path.join("resource", "*.sh")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Simone Vollaro",
    maintainer_email="slim71sv@gmail.com",
    description="Launch files package for the whole project",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
