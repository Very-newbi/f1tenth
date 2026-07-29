from glob import glob
import os

from setuptools import setup

package_name = "my_algo"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="roboracer",
    maintainer_email="roboracer@example.com",
    description="Disparity extender driving and AEB for a real F1TENTH car.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "disparity_extender = my_algo.disparity_extender:main",
            "aeb_mux = my_algo.aeb_mux:main",
        ],
    },
)
