from setuptools import setup

package_name = "converter_from_ukt_to_h1_package"

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
    description="ROS2 node for converting joint data from UKT device format \
        to Unitree H1 format",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"converter_from_ukt_to_h1_node = \
                {package_name}.converter_from_ukt_to_h1_node:main",
        ],
    },
)
