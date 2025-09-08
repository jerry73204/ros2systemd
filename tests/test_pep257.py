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

"""Test for docstring style using pep257."""

import pytest

try:
    from ament_pep257.main import main

    HAS_AMENT_PEP257 = True
except ImportError:
    HAS_AMENT_PEP257 = False


@pytest.mark.skip(reason="Docstring formatting not enforced for initial development")
@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Test that docstrings comply with PEP 257."""
    if HAS_AMENT_PEP257:
        rc = main(argv=[])
        assert rc == 0, "Found code style errors / warnings"
