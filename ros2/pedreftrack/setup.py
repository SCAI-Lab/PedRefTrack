from setuptools import find_packages, setup


package_name = "pedreftrack"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/pedreftrack.yaml"]),
        ("share/" + package_name + "/launch", ["launch/pedreftrack.launch.py"]),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="Draxran",
    maintainer_email="draxran@users.noreply.github.com",
    description="General ROS2 deployment package for PedRefTrack.",
    license="MIT",
    entry_points={"console_scripts": ["pedreftrack_node = pedreftrack.ros_node:main"]},
)
