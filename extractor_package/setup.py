from setuptools import setup

package_name = "extractor_package"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="banana-killer",
    maintainer_email="sashagrachev2005@gmail.com",
    description="ROS2 node for matching and publishing angles from the Unitree H1 robot\
                and from the UKT copier device of the selected joint",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "extractor_node = extractor_package.extractor_node:main"
        ],
    },
)
