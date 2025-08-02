from setuptools import setup

package_name = "converter_angles_ukt_into_rad_package"

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
    description='The ROS2 node converts the raw joint data of the UKT device\
                with a specified scaling factor (0.1 by default)\
                from the "UKT_bare_data" topic to radians.',
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"converter_angles_ukt_into_rad_node = \
                {package_name}.converter_angles_ukt_into_rad_node:main"
        ],
    },
)
