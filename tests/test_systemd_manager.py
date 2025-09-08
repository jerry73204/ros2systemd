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

"""Unit tests for SystemdServiceManager."""

import unittest
from pathlib import Path

from ros2_systemd.api.systemd_manager import SystemdServiceManager


class TestSystemdServiceManager(unittest.TestCase):
    """Test SystemdServiceManager functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.manager = SystemdServiceManager(user_mode=True)

    def test_service_name_validation_valid(self):
        """Test validation accepts valid service names."""
        valid_names = [
            "test-service",
            "test_service",
            "test.service",
            "test123",
            "TestService",
            "my-robot-node",
        ]

        for name in valid_names:
            valid, error = self.manager._validate_service_name(name)
            self.assertTrue(valid, f"Should accept valid name '{name}': {error}")

    def test_service_name_validation_invalid(self):
        """Test validation rejects invalid service names."""
        invalid_cases = [
            ("", "service name cannot be empty"),
            ("test service", "must start with alphanumeric"),
            ("test@service", "must start with alphanumeric"),
            ("-test", "must start with alphanumeric"),
            ("all", "reserved name"),
            ("help", "reserved name"),
            ("a" * 201, "too long"),
        ]

        for name, expected_error_part in invalid_cases:
            valid, error = self.manager._validate_service_name(name)
            self.assertFalse(valid, f"Should reject invalid name '{name[:20]}'")
            self.assertIn(expected_error_part, error.lower())

    def test_service_directory_paths(self):
        """Test correct service directory paths."""
        # Test user mode
        user_manager = SystemdServiceManager(user_mode=True)
        expected_user_dir = Path.home() / ".config" / "systemd" / "user"
        self.assertEqual(user_manager.service_dir, expected_user_dir)

        # Test system mode
        system_manager = SystemdServiceManager(user_mode=False)
        expected_system_dir = Path("/etc/systemd/system")
        self.assertEqual(system_manager.service_dir, expected_system_dir)

    def test_service_file_generation(self):
        """Test systemd service file content generation."""
        content = self.manager._generate_service_content(
            description="Test service",
            exec_command="ros2 run demo_nodes_cpp talker",
            env_vars={"TEST_VAR": "test_value"},
        )

        # Check required sections
        self.assertIn("[Unit]", content)
        self.assertIn("[Service]", content)
        self.assertIn("[Install]", content)

        # Check content
        self.assertIn("Description=Test service", content)
        self.assertIn("ros2 run demo_nodes_cpp talker", content)
        self.assertIn('Environment="TEST_VAR=test_value"', content)

        # Check user mode target
        if self.manager.user_mode:
            self.assertIn("WantedBy=default.target", content)
        else:
            self.assertIn("WantedBy=multi-user.target", content)

    def test_systemctl_command_generation(self):
        """Test systemctl command generation."""
        # User mode commands
        user_manager = SystemdServiceManager(user_mode=True)
        cmd = user_manager._get_systemctl_cmd(["status", "test.service"])
        self.assertEqual(cmd, ["systemctl", "--user", "status", "test.service"])

        # System mode commands
        system_manager = SystemdServiceManager(user_mode=False)
        cmd = system_manager._get_systemctl_cmd(["status", "test.service"])
        self.assertEqual(cmd, ["systemctl", "status", "test.service"])

    def test_service_prefix(self):
        """Test service name prefixing."""
        self.assertEqual(self.manager.SERVICE_PREFIX, "ros2-")

        # Test that service names get prefixed
        service_name = "test-service"
        full_name = f"{self.manager.SERVICE_PREFIX}{service_name}"
        self.assertEqual(full_name, "ros2-test-service")


if __name__ == "__main__":
    unittest.main()
