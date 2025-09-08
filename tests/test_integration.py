#!/usr/bin/env python3
# Copyright 2024 ROS2-Systemd Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Integration tests for ros2-systemd using ROS2 packages."""

import os
import time
import unittest
from pathlib import Path

import pytest

from ros2_systemd.api.systemd_manager import SystemdServiceManager


class TestRos2SystemdIntegration(unittest.TestCase):
    """Integration tests requiring ROS2 environment."""

    @classmethod
    def setUpClass(cls):
        """Set up test environment."""
        cls.manager = SystemdServiceManager(user_mode=True)
        cls.test_services = []
        cls.timestamp = int(time.time())

    @classmethod
    def tearDownClass(cls):
        """Clean up test services."""
        for service_name in cls.test_services:
            try:
                cls.manager.stop_service(service_name)
                cls.manager.remove_service(service_name)
            except Exception:
                pass

    def test_create_node_service(self):
        """Test creating a service for a ROS2 node."""
        service_name = f"test-node-{self.timestamp}"

        result = self.manager.create_node_service(
            service_name=service_name,
            package="demo_nodes_cpp",
            executable="talker",
            description="Integration test node service",
        )

        self.assertTrue(result)
        self.__class__.test_services.append(service_name)

        # Verify service file exists
        service_file = self.manager.service_dir / f"ros2-{service_name}.service"
        self.assertTrue(service_file.exists())

    def test_service_lifecycle(self):
        """Test complete service lifecycle."""
        service_name = f"test-lifecycle-{self.timestamp}"

        # Create service
        self.manager.create_node_service(
            service_name=service_name,
            package="demo_nodes_py",
            executable="talker",
            description="Lifecycle test service",
        )
        self.__class__.test_services.append(service_name)

        # Start service
        success, _ = self.manager.start_service(service_name)
        self.assertTrue(success)

        # Wait for service to start
        time.sleep(2)

        # Check status
        status = self.manager.get_service_status(service_name)
        self.assertTrue(status["exists"])
        self.assertIn(status["active"], ["running", "active"])

        # Stop service
        success, _ = self.manager.stop_service(service_name)
        self.assertTrue(success)

    def test_service_with_environment(self):
        """Test service creation with environment variables."""
        service_name = f"test-env-{self.timestamp}"

        env_vars = {"ROS_DOMAIN_ID": "42", "ROS_NAMESPACE": "/test"}

        result = self.manager.create_node_service(
            service_name=service_name,
            package="demo_nodes_cpp",
            executable="listener",
            env_vars=env_vars,
            description="Environment test service",
        )

        self.assertTrue(result)
        self.__class__.test_services.append(service_name)

        # Check service file contains environment variables
        service_file = self.manager.service_dir / f"ros2-{service_name}.service"
        content = service_file.read_text()
        self.assertIn('Environment="ROS_DOMAIN_ID=42"', content)
        self.assertIn('Environment="ROS_NAMESPACE=/test"', content)

    def test_launch_file_service(self):
        """Test creating a service for a launch file."""
        service_name = f"test-launch-{self.timestamp}"

        # Create a test launch file
        launch_file = Path.home() / f".test_launch_{self.timestamp}.py"
        launch_file.write_text(
            """
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='test_talker'
        )
    ])
"""
        )

        try:
            result = self.manager.create_launch_service(
                service_name=service_name, launch_file=str(launch_file), description="Launch file test service"
            )

            self.assertTrue(result)
            self.__class__.test_services.append(service_name)

            # Verify service file
            service_file = self.manager.service_dir / f"ros2-{service_name}.service"
            self.assertTrue(service_file.exists())

            content = service_file.read_text()
            self.assertIn("ros2 launch", content)
            self.assertIn(str(launch_file), content)

        finally:
            # Clean up launch file
            if launch_file.exists():
                launch_file.unlink()

    def test_list_services(self):
        """Test listing services."""
        # Create multiple test services
        service_names = []
        for i in range(3):
            name = f"test-list-{self.timestamp}-{i}"
            self.manager.create_node_service(
                service_name=name, package="demo_nodes_py", executable="talker", description=f"List test service {i}"
            )
            service_names.append(name)
            self.__class__.test_services.append(name)

        # List services
        services = self.manager.list_services()
        service_list_names = [s["name"] for s in services]

        # Check all test services are in the list
        for name in service_names:
            self.assertIn(name, service_list_names)


if __name__ == "__main__":
    unittest.main()
