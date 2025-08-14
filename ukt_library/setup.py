from setuptools import find_packages, setup

package_name = "ukt_library"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
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
    description="This is a python3 library for comfortable programming UKT. \
      It lib include reference info and special functions working with UKT.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
