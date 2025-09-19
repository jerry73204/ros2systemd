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

"""Tests for the run verb."""

import argparse
import os
import time
import unittest
from unittest.mock import MagicMock, patch

from ros2systemd.verb.run import RunVerb


class TestRunVerb(unittest.TestCase):
    """Test RunVerb functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.verb = RunVerb()
        self.parser = argparse.ArgumentParser()
        self.verb.add_arguments(self.parser, "ros2 systemd run")
        self.timestamp = int(time.time())

    def test_add_arguments(self):
        """Test that all required arguments are added."""
        # Test help contains expected content
        help_text = self.parser.format_help()

        # Check positional arguments
        self.assertIn("package_name", help_text)
        self.assertIn("executable_name", help_text)
        self.assertIn("argv", help_text)

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

    def test_parse_basic_args(self):
        """Test parsing basic arguments."""
        args = self.parser.parse_args(["demo_nodes_cpp", "talker"])

        self.assertEqual(args.package_name, "demo_nodes_cpp")
        self.assertEqual(args.executable_name, "talker")
        self.assertEqual(args.argv, [])
        self.assertEqual(args.env_mode, "ros")
        self.assertFalse(args.system)
        self.assertIsNone(args.name)

    def test_parse_args_with_options(self):
        """Test parsing arguments with options."""
        args = self.parser.parse_args(
            [
                "--name",
                "my-service",
                "--domain-id",
                "42",
                "--rmw",
                "rmw_cyclonedx_cpp",
                "--env",
                "TEST_VAR=test_value",
                "--system",
                "demo_nodes_cpp",
                "talker",
            ]
        )

        self.assertEqual(args.package_name, "demo_nodes_cpp")
        self.assertEqual(args.executable_name, "talker")
        self.assertEqual(args.name, "my-service")
        self.assertEqual(args.domain_id, 42)
        self.assertEqual(args.rmw, "rmw_cyclonedx_cpp")
        self.assertEqual(args.env, ["TEST_VAR=test_value"])
        self.assertTrue(args.system)

    def test_parse_args_with_argv(self):
        """Test parsing arguments with node arguments."""
        args = self.parser.parse_args(["demo_nodes_cpp", "talker", "--", "--ros-args", "-p", "frequency:=2.0"])

        self.assertEqual(args.package_name, "demo_nodes_cpp")
        self.assertEqual(args.executable_name, "talker")
        self.assertEqual(args.argv, ["--ros-args", "-p", "frequency:=2.0"])

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

    def test_generate_service_name(self):
        """Test automatic service name generation."""
        # Mock time to get predictable results
        with patch("time.time", return_value=1234567890):
            name = self.verb._generate_service_name("demo_nodes_cpp", "talker")
            self.assertEqual(name, "demo_nodes_cpp-talker-1234567890")

    @patch("ros2systemd.verb.run.SystemdServiceManager")
    @patch("sys.argv", ["ros2", "systemd", "run", "demo_nodes_cpp", "talker"])
    def test_main_success(self, mock_manager_class):
        """Test successful run command execution."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager.create_node_service.return_value = True
        mock_manager.start_service.return_value = (True, "")

        # Parse args
        args = self.parser.parse_args(["demo_nodes_cpp", "talker"])

        # Run main
        result = self.verb.main(args=args)

        # Verify success
        self.assertEqual(result, 0)
        mock_manager.create_node_service.assert_called_once()
        mock_manager.start_service.assert_called_once()

    @patch("ros2systemd.verb.run.SystemdServiceManager")
    @patch("sys.argv", ["ros2", "systemd", "run", "demo_nodes_cpp", "talker"])
    def test_main_create_failure(self, mock_manager_class):
        """Test run command when service creation fails."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager.create_node_service.return_value = False

        # Parse args
        args = self.parser.parse_args(["demo_nodes_cpp", "talker"])

        # Run main
        result = self.verb.main(args=args)

        # Verify failure
        self.assertEqual(result, 1)
        mock_manager.create_node_service.assert_called_once()
        mock_manager.start_service.assert_not_called()

    @patch("ros2systemd.verb.run.SystemdServiceManager")
    @patch("sys.argv", ["ros2", "systemd", "run", "demo_nodes_cpp", "talker"])
    def test_main_start_failure(self, mock_manager_class):
        """Test run command when service start fails."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager.create_node_service.return_value = True
        mock_manager.start_service.return_value = (False, "Service failed to start")

        # Parse args
        args = self.parser.parse_args(["demo_nodes_cpp", "talker"])

        # Run main
        result = self.verb.main(args=args)

        # Verify failure
        self.assertEqual(result, 1)
        mock_manager.create_node_service.assert_called_once()
        mock_manager.start_service.assert_called_once()

    @patch("ros2systemd.verb.run.SystemdServiceManager")
    @patch("sys.argv", ["ros2", "systemd", "run", "demo_nodes_cpp", "talker", "--", "--ros-args"])
    def test_main_with_delimiter(self, mock_manager_class):
        """Test run command with -- delimiter for node args."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager.create_node_service.return_value = True
        mock_manager.start_service.return_value = (True, "")

        # Parse args
        args = self.parser.parse_args(["demo_nodes_cpp", "talker", "--", "--ros-args"])

        # Run main
        result = self.verb.main(args=args)

        # Verify success and that args were passed
        self.assertEqual(result, 0)
        call_args = mock_manager.create_node_service.call_args
        self.assertEqual(call_args[1]["node_args"], ["--ros-args"])

    @patch("ros2systemd.verb.run.SystemdServiceManager")
    @patch("sys.argv", ["ros2", "systemd", "run", "demo_nodes_cpp", "talker", "--ros-args"])
    def test_main_missing_delimiter(self, mock_manager_class):
        """Test run command fails when -- delimiter is missing for dash args."""
        # Mock the manager
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager

        # Parse args
        args = self.parser.parse_args(["demo_nodes_cpp", "talker", "--ros-args"])

        # Run main - should fail due to missing delimiter
        result = self.verb.main(args=args)

        # Verify failure
        self.assertEqual(result, 1)
        mock_manager.create_node_service.assert_not_called()

    @patch("ros2systemd.verb.run.SystemdServiceManager")
    @patch("sys.argv", ["ros2", "systemd", "run", "--name", "custom-name", "demo_nodes_cpp", "talker"])
    def test_main_with_custom_name(self, mock_manager_class):
        """Test run command with custom service name."""
        # Mock the manager and its methods
        mock_manager = MagicMock()
        mock_manager_class.return_value = mock_manager
        mock_manager.create_node_service.return_value = True
        mock_manager.start_service.return_value = (True, "")

        # Parse args
        args = self.parser.parse_args(["--name", "custom-name", "demo_nodes_cpp", "talker"])

        # Run main
        result = self.verb.main(args=args)

        # Verify custom name was used
        self.assertEqual(result, 0)
        call_args = mock_manager.create_node_service.call_args
        self.assertEqual(call_args[1]["service_name"], "custom-name")


if __name__ == "__main__":
    unittest.main()
