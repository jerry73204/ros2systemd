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

"""Tests for the launch verb."""

import argparse
import os
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from ros2systemd.verb.launch import LaunchVerb


class TestLaunchVerb(unittest.TestCase):
    """Test LaunchVerb functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.verb = LaunchVerb()
        self.parser = argparse.ArgumentParser()
        self.verb.add_arguments(self.parser, "ros2 systemd launch")
        self.timestamp = int(time.time())

    def test_add_arguments(self):
        """Test that all required arguments are added."""
        # Test help contains expected content
        help_text = self.parser.format_help()

        # Check positional arguments
        self.assertIn("package_name", help_text)
        self.assertIn("launch_file_name", help_text)
        self.assertIn("launch_arguments", help_text)

        # Check options
        self.assertIn("--env", help_text)
        self.assertIn("--copy-env", help_text)
        self.assertIn("--source", help_text)
        self.assertIn("--env-mode", help_text)
        self.assertIn("--domain-id", help_text)
        self.assertIn("--rmw", help_text)
        self.assertIn("--localhost-only", help_text)
        self.assertIn("--network-isolation", help_text)
        self.assertIn("--description", help_text)
        self.assertIn("--system", help_text)
        self.assertIn("--name", help_text)
        self.assertIn("--replace", help_text)

    def test_parse_basic_args(self):
        """Test parsing basic arguments."""
        args = self.parser.parse_args(["demo_nodes_cpp", "talker_listener.launch.py"])

        self.assertEqual(args.package_name, "demo_nodes_cpp")
        self.assertEqual(args.launch_file_name, "talker_listener.launch.py")
        self.assertEqual(args.launch_arguments, [])
        self.assertEqual(args.env_mode, "ros")
        self.assertFalse(args.system)
        self.assertIsNone(args.name)

    def test_parse_args_with_options(self):
        """Test parsing arguments with options."""
        args = self.parser.parse_args(
            [
                "--name",
                "my-launch",
                "--domain-id",
                "42",
                "--rmw",
                "rmw_cyclonedx_cpp",
                "--env",
                "TEST_VAR=test_value",
                "--system",
                "demo_nodes_cpp",
                "talker_listener.launch.py",
            ]
        )

        self.assertEqual(args.package_name, "demo_nodes_cpp")
        self.assertEqual(args.launch_file_name, "talker_listener.launch.py")
        self.assertEqual(args.name, "my-launch")
        self.assertEqual(args.domain_id, 42)
        self.assertEqual(args.rmw, "rmw_cyclonedx_cpp")
        self.assertEqual(args.env, ["TEST_VAR=test_value"])
        self.assertTrue(args.system)

    def test_parse_args_with_launch_arguments(self):
        """Test parsing arguments with launch arguments."""
        args = self.parser.parse_args(
            ["demo_nodes_cpp", "talker_listener.launch.py", "use_sim_time:=true", "robot_name:=test_robot"]
        )

        self.assertEqual(args.package_name, "demo_nodes_cpp")
        self.assertEqual(args.launch_file_name, "talker_listener.launch.py")
        self.assertEqual(args.launch_arguments, ["use_sim_time:=true", "robot_name:=test_robot"])

    def test_parse_args_single_package(self):
        """Test parsing with single argument (package only)."""
        args = self.parser.parse_args(["demo_nodes_cpp"])

        self.assertEqual(args.package_name, "demo_nodes_cpp")
        self.assertIsNone(args.launch_file_name)
        self.assertEqual(args.launch_arguments, [])

    def test_capture_environment_ros_mode(self):
        """Test environment capture in ros mode."""
        with patch.dict(
            os.environ,
            {
                "AMENT_PREFIX_PATH": "/opt/ros/humble",
                "ROS_DISTRO": "humble",
                "ROS_DOMAIN_ID": "5",
                "SOME_OTHER_VAR": "should_not_capture",
            },
        ):
            env_vars = self.verb._capture_environment("ros")

            self.assertIn("AMENT_PREFIX_PATH", env_vars)
            self.assertIn("ROS_DISTRO", env_vars)
            self.assertNotIn("ROS_DOMAIN_ID", env_vars)  # Special var, handled separately
            self.assertNotIn("SOME_OTHER_VAR", env_vars)

    def test_capture_environment_all_mode(self):
        """Test environment capture in all mode."""
        with patch.dict(
            os.environ,
            {
                "AMENT_PREFIX_PATH": "/opt/ros/humble",
                "SOME_OTHER_VAR": "should_capture",
                "ROS_DOMAIN_ID": "5",  # Special var, should be excluded
            },
        ):
            env_vars = self.verb._capture_environment("all")

            self.assertIn("AMENT_PREFIX_PATH", env_vars)
            self.assertIn("SOME_OTHER_VAR", env_vars)
            self.assertNotIn("ROS_DOMAIN_ID", env_vars)  # Special var excluded

    def test_capture_environment_none_mode(self):
        """Test environment capture in none mode."""
        with patch.dict(os.environ, {"AMENT_PREFIX_PATH": "/opt/ros/humble", "ROS_DISTRO": "humble"}):
            env_vars = self.verb._capture_environment("none")

            self.assertEqual(env_vars, {})

    def test_generate_service_name_with_launch_file(self):
        """Test automatic service name generation with launch file."""
        # Mock time to get predictable results
        with patch("time.time", return_value=1234567890):
            name = self.verb._generate_service_name("demo_nodes_cpp", "talker_listener.launch.py")
            self.assertEqual(name, "demo_nodes_cpp-talker_listener-1234567890")

    def test_generate_service_name_full_path(self):
        """Test automatic service name generation with full path."""
        # Mock time to get predictable results
        with patch("time.time", return_value=1234567890):
            name = self.verb._generate_service_name("/path/to/my_launch.launch.py")
            self.assertEqual(name, "launch-my_launch-1234567890")

    def test_generate_service_name_package_only(self):
        """Test automatic service name generation with package only."""
        # Mock time to get predictable results
        with patch("time.time", return_value=1234567890):
            name = self.verb._generate_service_name("demo_nodes_cpp")
            self.assertEqual(name, "demo_nodes_cpp-launch-1234567890")

    @patch("ros2systemd.verb.launch.SystemdServiceManager")
    def test_main_success_with_launch_file(self, mock_manager_class):
        """Test successful launch command execution with package + launch file."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager._resolve_package_path.return_value = Path(
            "/opt/ros/humble/share/demo_nodes_cpp/launch/topics/talker_listener.launch.py"
        )
        mock_manager.create_launch_service.return_value = True
        mock_manager.start_service.return_value = (True, "")

        # Parse args
        args = self.parser.parse_args(["demo_nodes_cpp", "talker_listener.launch.py"])

        # Run main
        result = self.verb.main(args=args)

        # Verify success
        self.assertEqual(result, 0)
        mock_manager.create_launch_service.assert_called_once()
        mock_manager.start_service.assert_called_once()

    @patch("ros2systemd.verb.launch.SystemdServiceManager")
    @patch("pathlib.Path.exists")
    def test_main_success_with_full_path(self, mock_exists, mock_manager_class):
        """Test successful launch command execution with full path."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager.create_launch_service.return_value = True
        mock_manager.start_service.return_value = (True, "")
        mock_exists.return_value = True

        # Parse args
        args = self.parser.parse_args(["/path/to/launch/file.launch.py"])

        # Run main
        result = self.verb.main(args=args)

        # Verify success
        self.assertEqual(result, 0)
        mock_manager.create_launch_service.assert_called_once()
        mock_manager.start_service.assert_called_once()

    @patch("ros2systemd.verb.launch.SystemdServiceManager")
    def test_main_package_not_found(self, mock_manager_class):
        """Test launch command when package/launch file not found."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager._resolve_package_path.return_value = None

        # Parse args
        args = self.parser.parse_args(["nonexistent_package", "launch.py"])

        # Run main
        result = self.verb.main(args=args)

        # Verify failure
        self.assertEqual(result, 1)
        mock_manager.create_launch_service.assert_not_called()

    @patch("ros2systemd.verb.launch.SystemdServiceManager")
    def test_main_create_failure(self, mock_manager_class):
        """Test launch command when service creation fails."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager._resolve_package_path.return_value = Path(
            "/opt/ros/humble/share/demo_nodes_cpp/launch/topics/talker_listener.launch.py"
        )
        mock_manager.create_launch_service.return_value = False

        # Parse args
        args = self.parser.parse_args(["demo_nodes_cpp", "talker_listener.launch.py"])

        # Run main
        result = self.verb.main(args=args)

        # Verify failure
        self.assertEqual(result, 1)
        mock_manager.create_launch_service.assert_called_once()
        mock_manager.start_service.assert_not_called()

    @patch("ros2systemd.verb.launch.SystemdServiceManager")
    def test_main_start_failure(self, mock_manager_class):
        """Test launch command when service start fails."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager._resolve_package_path.return_value = Path(
            "/opt/ros/humble/share/demo_nodes_cpp/launch/topics/talker_listener.launch.py"
        )
        mock_manager.create_launch_service.return_value = True
        mock_manager.start_service.return_value = (False, "Service failed to start")

        # Parse args
        args = self.parser.parse_args(["demo_nodes_cpp", "talker_listener.launch.py"])

        # Run main
        result = self.verb.main(args=args)

        # Verify failure
        self.assertEqual(result, 1)
        mock_manager.create_launch_service.assert_called_once()
        mock_manager.start_service.assert_called_once()

    @patch("ros2systemd.verb.launch.SystemdServiceManager")
    def test_main_with_launch_arguments(self, mock_manager_class):
        """Test launch command with launch arguments."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager._resolve_package_path.return_value = Path(
            "/opt/ros/humble/share/demo_nodes_cpp/launch/topics/talker_listener.launch.py"
        )
        mock_manager.create_launch_service.return_value = True
        mock_manager.start_service.return_value = (True, "")

        # Parse args
        args = self.parser.parse_args(
            ["demo_nodes_cpp", "talker_listener.launch.py", "use_sim_time:=true", "robot_name:=test"]
        )

        # Run main
        result = self.verb.main(args=args)

        # Verify success and that launch args were parsed
        self.assertEqual(result, 0)
        call_args = mock_manager.create_launch_service.call_args
        expected_launch_args = {"use_sim_time": "true", "robot_name": "test"}
        self.assertEqual(call_args[1]["launch_args"], expected_launch_args)

    @patch("ros2systemd.verb.launch.SystemdServiceManager")
    def test_main_with_custom_name(self, mock_manager_class):
        """Test launch command with custom service name."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager._resolve_package_path.return_value = Path(
            "/opt/ros/humble/share/demo_nodes_cpp/launch/topics/talker_listener.launch.py"
        )
        mock_manager.create_launch_service.return_value = True
        mock_manager.start_service.return_value = (True, "")

        # Parse args
        args = self.parser.parse_args(["--name", "custom-launch", "demo_nodes_cpp", "talker_listener.launch.py"])

        # Run main
        result = self.verb.main(args=args)

        # Verify custom name was used
        self.assertEqual(result, 0)
        call_args = mock_manager.create_launch_service.call_args
        self.assertEqual(call_args[1]["service_name"], "custom-launch")

    @patch("ros2systemd.verb.launch.SystemdServiceManager")
    def test_main_package_default_launch_found(self, mock_manager_class):
        """Test launch command with package only when default launch file is found."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        # First two calls return None, third returns a path (simulating found default launch)
        mock_manager._resolve_package_path.side_effect = [
            None,  # launch.py
            None,  # default.launch.py
            Path("/opt/ros/humble/share/demo_nodes_cpp/launch/demo_nodes_cpp.launch.py"),  # {package}.launch.py
        ]
        mock_manager.create_launch_service.return_value = True
        mock_manager.start_service.return_value = (True, "")

        # Parse args (package only)
        args = self.parser.parse_args(["demo_nodes_cpp"])

        # Run main
        result = self.verb.main(args=args)

        # Verify success
        self.assertEqual(result, 0)
        # Should have been called 3 times looking for default launch files
        self.assertEqual(mock_manager._resolve_package_path.call_count, 3)
        mock_manager.create_launch_service.assert_called_once()

    @patch("ros2systemd.verb.launch.SystemdServiceManager")
    def test_main_package_no_default_launch(self, mock_manager_class):
        """Test launch command with package only when no default launch file is found."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        # All calls return None (no default launch files found)
        mock_manager._resolve_package_path.return_value = None

        # Parse args (package only)
        args = self.parser.parse_args(["demo_nodes_cpp"])

        # Run main
        result = self.verb.main(args=args)

        # Verify failure
        self.assertEqual(result, 1)
        mock_manager.create_launch_service.assert_not_called()

    @patch("ros2systemd.verb.launch.SystemdServiceManager")
    def test_main_with_replace_existing_service(self, mock_manager_class):
        """Test launch command with --replace option when service exists and is running."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager.get_service_status.return_value = {"exists": True, "active": "running"}
        mock_manager.stop_service.return_value = True
        mock_manager.remove_service.return_value = True
        mock_manager._resolve_package_path.return_value = "/path/to/launch.py"
        mock_manager.create_launch_service.return_value = True
        mock_manager.start_service.return_value = (True, "")

        # Parse args
        args = self.parser.parse_args(["--replace", "--name", "test-service", "demo_nodes_cpp", "launch.py"])

        # Run main
        result = self.verb.main(args=args)

        # Verify replacement workflow
        self.assertEqual(result, 0)
        mock_manager.get_service_status.assert_called_once_with("test-service")
        mock_manager.stop_service.assert_called_once_with("test-service")
        mock_manager.remove_service.assert_called_once_with("test-service")
        mock_manager.create_launch_service.assert_called_once()
        mock_manager.start_service.assert_called_once_with("test-service")

    @patch("ros2systemd.verb.launch.SystemdServiceManager")
    def test_main_with_replace_existing_stopped_service(self, mock_manager_class):
        """Test launch command with --replace option when service exists but is stopped."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager.get_service_status.return_value = {"exists": True, "active": "inactive"}
        mock_manager.remove_service.return_value = True
        mock_manager._resolve_package_path.return_value = "/path/to/launch.py"
        mock_manager.create_launch_service.return_value = True
        mock_manager.start_service.return_value = (True, "")

        # Parse args
        args = self.parser.parse_args(["--replace", "--name", "test-service", "demo_nodes_cpp", "launch.py"])

        # Run main
        result = self.verb.main(args=args)

        # Verify replacement workflow (stop should not be called for inactive service)
        self.assertEqual(result, 0)
        mock_manager.get_service_status.assert_called_once_with("test-service")
        mock_manager.stop_service.assert_not_called()  # Should not stop inactive service
        mock_manager.remove_service.assert_called_once_with("test-service")
        mock_manager.create_launch_service.assert_called_once()
        mock_manager.start_service.assert_called_once_with("test-service")

    @patch("ros2systemd.verb.launch.SystemdServiceManager")
    def test_main_with_replace_nonexistent_service(self, mock_manager_class):
        """Test launch command with --replace option when service does not exist."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager.get_service_status.return_value = {"exists": False}
        mock_manager._resolve_package_path.return_value = "/path/to/launch.py"
        mock_manager.create_launch_service.return_value = True
        mock_manager.start_service.return_value = (True, "")

        # Parse args
        args = self.parser.parse_args(["--replace", "--name", "test-service", "demo_nodes_cpp", "launch.py"])

        # Run main
        result = self.verb.main(args=args)

        # Verify normal creation workflow (no stop/remove should occur)
        self.assertEqual(result, 0)
        mock_manager.get_service_status.assert_called_once_with("test-service")
        mock_manager.stop_service.assert_not_called()
        mock_manager.remove_service.assert_not_called()
        mock_manager.create_launch_service.assert_called_once()
        mock_manager.start_service.assert_called_once_with("test-service")


if __name__ == "__main__":
    unittest.main()
