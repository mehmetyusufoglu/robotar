from setuptools import setup

package_name = "robotar_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/harvest_demo.yaml"]),
        ("share/" + package_name + "/launch", ["launch/harvest_demo.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="memo",
    maintainer_email="memo@users.noreply.github.com",
    description="Scripted odom-drive and harvest-demo state machine for the "
                "Robotar olive-harvesting simulation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "odom_driver = robotar_mission.odom_driver:main",
            "harvest_demo = robotar_mission.harvest_demo:main",
        ],
    },
)
