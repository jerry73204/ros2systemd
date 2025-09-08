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

"""Test for code style using flake8."""

import subprocess
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Test that code complies with flake8."""
    # Use system flake8 directly to avoid ament_flake8 compatibility issues
    # Disable quote checking since the project uses double quotes
    result = subprocess.run(
        [
            "python3",
            "-m",
            "flake8",
            "ros2_systemd",
            "--max-line-length=120",
            "--exclude=__pycache__",
            "--extend-ignore=Q000,D100,D101,D102,D103,D104,D105,D107",
        ],
        capture_output=True,
        text=True,
    )

    if result.returncode != 0:
        errors = result.stdout.strip().split("\n") if result.stdout else []
        pytest.fail(f"Found {len(errors)} code style errors:\n" + result.stdout)
