from setuptools import setup

package_name = "repeater_package"

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
    description="ROS2 node for receiving UDP packets with JSON data and \
        retransmitting them to a ROS2 topic",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "repeater_node = repeater_package.repeater_node:main"
        ],
    },
)
