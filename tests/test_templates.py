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

"""Unit tests for template system."""

import unittest

from ros2_systemd.templates import get_template, get_template_info, list_templates


class TestTemplates(unittest.TestCase):
    """Test template system functionality."""

    def test_list_templates(self):
        """Test listing available templates."""
        templates = list_templates()
        self.assertIsInstance(templates, list)
        self.assertGreater(len(templates), 0)

        # Check for some expected templates
        self.assertIn("talker", templates)
        self.assertIn("listener", templates)
        self.assertIn("turtlesim", templates)

    def test_get_template_valid(self):
        """Test getting a valid template."""
        template = get_template("talker")
        self.assertIsNotNone(template)
        self.assertIn("package", template)
        self.assertIn("executable", template)
        self.assertIn("description", template)

        # Check specific values
        self.assertEqual(template["package"], "demo_nodes_cpp")
        self.assertEqual(template["executable"], "talker")

    def test_get_template_invalid(self):
        """Test getting a non-existent template."""
        template = get_template("non_existent_template")
        self.assertIsNone(template)

    def test_get_template_info(self):
        """Test getting template information."""
        info = get_template_info("turtlesim")
        self.assertIsInstance(info, str)
        self.assertIn("Template: turtlesim", info)
        self.assertIn("turtlesim_node", info)
        self.assertIn("QT_QPA_PLATFORM", info)

    def test_template_with_environment_vars(self):
        """Test template with environment variables."""
        template = get_template("turtlesim")
        self.assertIn("env_vars", template)
        self.assertIn("QT_QPA_PLATFORM", template["env_vars"])

    def test_template_with_node_args(self):
        """Test template with node arguments."""
        template = get_template("robot-state-publisher")
        self.assertIn("node_args", template)
        self.assertIsInstance(template["node_args"], list)

    def test_all_templates_valid(self):
        """Test that all templates have required fields."""
        templates = list_templates()

        for template_name in templates:
            template = get_template(template_name)
            with self.subTest(template=template_name):
                self.assertIsNotNone(template)
                self.assertIn("description", template)
                self.assertIn("package", template)
                self.assertIn("executable", template)

                # Description should not be empty
                self.assertTrue(template["description"])
                self.assertTrue(template["package"])
                self.assertTrue(template["executable"])


if __name__ == "__main__":
    unittest.main()
