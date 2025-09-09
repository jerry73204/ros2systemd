from setuptools import find_packages, setup

package_name = "ros2_systemd"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "ros2cli",
        "setuptools",
    ],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your.email@example.com",
    description="ROS2 command extension for managing launches and nodes as systemd services",
    license="Apache-2.0",
    tests_require=[
        "pytest",
        "pytest-cov",
    ],
    entry_points={
        "ros2cli.command": [
            "systemd = ros2_systemd.command.systemd:SystemdCommand",
        ],
        "ros2cli.extension_point": [
            "ros2_systemd.verb = ros2_systemd.verb:VerbExtension",
        ],
        "ros2_systemd.verb": [
            "create = ros2_systemd.verb.create:CreateVerb",
            "start = ros2_systemd.verb.start:StartVerb",
            "stop = ros2_systemd.verb.stop:StopVerb",
            "restart = ros2_systemd.verb.restart:RestartVerb",
            "status = ros2_systemd.verb.status:StatusVerb",
            "enable = ros2_systemd.verb.enable:EnableVerb",
            "disable = ros2_systemd.verb.disable:DisableVerb",
            "list = ros2_systemd.verb.list:ListVerb",
            "remove = ros2_systemd.verb.remove:RemoveVerb",
            "template = ros2_systemd.verb.template:TemplateVerb",
            "logs = ros2_systemd.verb.logs:LogsVerb",
            "diagnose = ros2_systemd.verb.diagnose:DiagnoseVerb",
        ],
    },
)
