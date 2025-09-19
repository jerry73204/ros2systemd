# Changelog

## [0.4.0] - 2025-09-19

### Added
- `ros2 systemd run` command for instant node service creation and startup (create + start in one step)
- `ros2 systemd launch` command for instant launch file service creation and startup (create + start in one step)
- Auto-generated service names with timestamp for uniqueness (`package-executable-timestamp`)
- `--name` option for custom service names in both run and launch commands
- Comprehensive test coverage for new run and launch verbs (34 new tests)
- Improved README.md with intuitive usage progression (`ros2 run` → `ros2 systemd run`)

### Changed
- Reorganized README.md tutorial structure for better user experience
- Compacted Command Reference section to focus on essential syntax
- Enhanced argument parsing and environment handling in new commands
- Updated integration tests to properly handle ROS2 environment setup

### Fixed
- Integration test failures due to missing ROS2 environment setup in systemd services
- Service name generation for launch files (properly removes .launch.py extensions)
- Line length issues in code formatting

### Technical Improvements
- Consistent environment capture logic across run and launch commands
- Robust error handling and validation for service creation and startup
- Mock-based testing for comprehensive coverage without system dependencies
- Proper delimiter validation for command arguments

## [0.3.0] - 2025-09-18

### Added
- `--env-mode` flag with options `ros`, `all`, and `none` for flexible environment variable capture
- Expanded ROS environment variable capture to include more keys (ROS_NAMESPACE, ROS_LOG_DIR, etc.)
- GitHub installation instructions in README

### Changed
- Replaced `--no-capture-env` flag with more flexible `--env-mode={ros,all,none}`
- Environment capture now includes additional ROS-specific variables for better service compatibility
- Simplified installation with removal of pyproject.toml in favor of setup.py only
- Updated CLI usage documentation with clearer examples

### Fixed
- Setup.py compatibility issues with pip installations
- Environment variable capture completeness

### Removed
- pyproject.toml file (using setup.py exclusively)
- `--no-capture-env` flag (replaced by `--env-mode`)

## [0.2.0] - 2025-09-15

### Added
- exec() based terminal control for `status` and `logs` commands
- Automatic color support with TTY detection  
- `--no-color` flag for `status` and `logs` commands
- Comprehensive ROS/Ament environment variable capture
- `--source` flag for chaining multiple setup scripts (can be used multiple times)
- `--copy-env` flag for selective environment variable copying
- `--no-capture-env` flag to disable automatic environment capture
- Mandatory `--` delimiter validation for extra arguments starting with dashes
- Enhanced error messages and user guidance
- Smart environment setup warnings when no environment is provided
- Automatic detection of terminal capabilities for color support

### Changed
- `status` command now uses `os.execvp()` instead of `subprocess.run()` for direct terminal control
- `logs` command now uses `os.execvp()` instead of `subprocess.run()` for direct terminal control  
- Argument parsing now requires `--` delimiter when extra_args contain flags starting with dashes
- Environment variable handling is now more intelligent and configurable
- Default behavior now captures current shell environment automatically
- Color support is now handled natively by systemctl/journalctl instead of manual detection

### Fixed
- Argument parsing failures when extra_args contain ROS arguments with dashes
- Color support inconsistencies across different terminal types
- Memory usage issues in long-running status/logs commands
- Signal handling problems in logs follow mode (`-f`)
- Environment variable capture edge cases
- subprocess management overhead and potential memory leaks

### Breaking Changes
- Extra arguments containing flags that start with dashes now require `--` delimiter
  - Example: `ros2 systemd create name node pkg exe -- --ros-args -p param:=value`
  - This affects ROS arguments like `--ros-args`, `-p`, `-r`, etc.

### Deprecated
- None

### Removed
- Complex subprocess-based color detection logic (replaced with native terminal control)
- Manual ANSI color code handling (now handled by system tools)

### Security
- No security-related changes

### Migration Guide
1. Update commands that pass ROS arguments to use `--` delimiter
2. Test color output in your terminal (should work automatically)
3. Consider using `--no-capture-env` with explicit `--source` for reproducible builds
4. Use `--no-color` flag for scripts and automation

---

## [0.1.x] - Previous versions
- Initial implementation
- Basic systemd service management
- Core ros2cli integration
