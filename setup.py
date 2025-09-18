from setuptools import find_packages, setup
import os

package_name = "ros2systemd"

# Determine if we're in a ROS2 build environment
in_ros2_build = os.environ.get('COLCON', False) or os.environ.get('AMENT_PREFIX_PATH', False)

# Common setup parameters
common_params = {
    "name": package_name,
    "version": "0.3.0",
    "packages": find_packages(exclude=["test", "tests"]),
    "zip_safe": True,
    "maintainer": "Your Name",
    "maintainer_email": "your.email@example.com",
    "description": "ROS2 command extension for managing launches and nodes as systemd services",
    "license": "Apache-2.0",
    "entry_points": {
        "ros2cli.command": [
            "systemd = ros2systemd.command.systemd:SystemdCommand",
        ],
        "ros2cli.extension_point": [
            "ros2systemd.verb = ros2systemd.verb:VerbExtension",
        ],
        "ros2systemd.verb": [
            "create = ros2systemd.verb.create:CreateVerb",
            "start = ros2systemd.verb.start:StartVerb",
            "stop = ros2systemd.verb.stop:StopVerb",
            "restart = ros2systemd.verb.restart:RestartVerb",
            "status = ros2systemd.verb.status:StatusVerb",
            "enable = ros2systemd.verb.enable:EnableVerb",
            "disable = ros2systemd.verb.disable:DisableVerb",
            "list = ros2systemd.verb.list:ListVerb",
            "remove = ros2systemd.verb.remove:RemoveVerb",
            "template = ros2systemd.verb.template:TemplateVerb",
            "logs = ros2systemd.verb.logs:LogsVerb",
            "diagnose = ros2systemd.verb.diagnose:DiagnoseVerb",
        ],
    },
}

# ROS2/Colcon specific configuration
if in_ros2_build:
    common_params.update({
        "data_files": [
            ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
            ("share/" + package_name, ["package.xml"]),
        ],
        "install_requires": [
            "ros2cli",
            "setuptools",
        ],
        "extras_require": {
            "dev": [
                "pytest",
                "pytest-cov",
            ],
        },
    })
else:
    # Pure pip installation
    common_params.update({
        "install_requires": [
            "setuptools",
            # ros2cli is assumed to be provided by ROS2 environment
        ],
        "python_requires": ">=3.8",
        "long_description": open("README.md").read() if os.path.exists("README.md") else "",
        "long_description_content_type": "text/markdown",
        "url": "https://github.com/jerry73204/ros2systemd",
        "classifiers": [
            "Development Status :: 4 - Beta",
            "Intended Audience :: Developers",
            "License :: OSI Approved :: Apache Software License",
            "Operating System :: POSIX :: Linux",
            "Programming Language :: Python :: 3",
            "Programming Language :: Python :: 3.8",
            "Programming Language :: Python :: 3.9",
            "Programming Language :: Python :: 3.10",
            "Programming Language :: Python :: 3.11",
        ],
    })

setup(**common_params)
