# Changelog

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
