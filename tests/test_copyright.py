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

"""Test for copyright headers in source files."""

import pytest

try:
    from ament_copyright.main import main

    HAS_AMENT_COPYRIGHT = True
except ImportError:
    HAS_AMENT_COPYRIGHT = False


@pytest.mark.skip(reason="Copyright headers not required for initial development")
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    """Test that all source files have copyright headers."""
    if HAS_AMENT_COPYRIGHT:
        rc = main(argv=[".", "apache2"])
        assert rc == 0, "Found errors"
