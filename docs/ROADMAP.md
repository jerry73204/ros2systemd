# ROS2-Systemd Extension Roadmap

## Project Status: v0.2.0 Complete ✓

## Overview
This roadmap outlines the development plan for the ROS2-Systemd extension, a simple tool that enables management of ROS2 launches and nodes as systemd services.

## Project Scope
This extension focuses on providing essential systemd service management capabilities:
- **Create** services for ROS2 nodes and launch files
- **Control** service lifecycle (start, stop, restart, enable, disable)
- **Monitor** service status
- **List** existing services
- **Remove** services when no longer needed

## Phase 1: Core Foundation (Complete ✓)

### Features Implemented
- [x] ROS2 CLI extension framework
- [x] Systemd service management API with context separation
- [x] Service creation for launch files and nodes
- [x] Service lifecycle management (start, stop, restart)
- [x] Service persistence (enable, disable)
- [x] Service monitoring (status, list)
- [x] Service removal with confirmation
- [x] User and system service isolation
- [x] Namespace separation (ros2- prefix)
- [x] Environment variable configuration
- [x] Launch arguments support
- [x] Safe defaults (user services by default)

### Development Infrastructure
- [x] Project structure and setup files
- [x] Python package configuration
- [x] Build system (colcon integration)
- [x] Development tools (Makefile, test scripts)
- [x] Documentation (README, DESIGN, ROADMAP)
- [x] Git ignore configuration

## Phase 2: Polish & Stability (Complete ✓)

### Testing with ROS2 Built-in Components
- [x] Add test dependencies to package.xml (demo_nodes_cpp, demo_nodes_py, turtlesim)
- [x] Test suite using demo_nodes_cpp (talker/listener)
- [x] Test with turtlesim package
- [x] Test with ros2 topic/service tools as services
- [x] Test with example launch files from ros2_launch
- [x] Automated test script for all commands
- [x] Test both user and system service contexts

### Improvements (v0.2.0)
- [x] Better error messages and handling
- [x] Service name validation
- [x] Basic service templates for common ROS2 packages
- [x] Simple log viewing with journalctl
- [ ] Improved help text and examples
- [ ] Testing with multiple ROS2 distributions

### Bug Fixes & Refinements
- [ ] Handle edge cases in service creation
- [ ] Improve sudo permission checks
- [ ] Better handling of missing dependencies
- [ ] Clearer context switching feedback

## Phase 3: Maintenance (Ongoing)

### Compatibility
- [ ] Test with new ROS2 releases
- [ ] Update for new systemd features as needed
- [ ] Maintain compatibility with Ubuntu LTS versions

### Documentation
- [ ] Add more examples to README
- [ ] Create troubleshooting guide
- [ ] Document common use cases

## Version Planning

### v0.1.0 (Released)
- Core functionality with context separation
- User/system service management
- CLI integration with safe defaults
- Service isolation and namespace management

### v0.2.0 (Current)
- Improved error handling
- Service templates for common packages
- Basic log viewing
- Better documentation

### v1.0.0 (Future)
- Stable, well-tested release
- Complete documentation
- Full compatibility with ROS2 Humble, Iron, and Jazzy

## Work Items by Priority

### High Priority
1. Update package.xml with test dependencies
2. Create comprehensive test suite with ROS2 built-in nodes
3. Improve error messages
4. Add service name validation
5. Test with ROS2 Iron and Jazzy

### Medium Priority
1. Create basic service templates
2. Simple log viewing integration
3. Better sudo/permission handling
4. Expand documentation with examples

### Low Priority
1. Performance optimizations
2. Additional service templates
3. Bash completion support

## Testing Strategy

### Current
- [x] Manual testing scripts
- [x] Basic functionality verification
- [x] Development environment setup

### Planned (Phase 2)
- [ ] Update package.xml with test dependencies:
  - [ ] demo_nodes_cpp (test_depend)
  - [ ] demo_nodes_py (test_depend)
  - [ ] turtlesim (test_depend)
  - [ ] ros2launch (test_depend)
  - [ ] launch_testing (test_depend)
  - [ ] launch_testing_ros (test_depend)
- [ ] Unit tests for core functionality
- [ ] Integration tests using ROS2 built-in packages:
  - [ ] demo_nodes_cpp (talker/listener nodes)
  - [ ] demo_nodes_py (Python equivalents)
  - [ ] turtlesim (turtlesim_node, turtle_teleop_key)
  - [ ] ros2 run examples (parameter demos, service demos)
  - [ ] ros2 launch examples from ros2launch package
- [ ] Test coverage for all commands:
  - [ ] create (node and launch variants)
  - [ ] start/stop/restart
  - [ ] enable/disable
  - [ ] status/list
  - [ ] remove
- [ ] Context testing (user vs system services)
- [ ] Multi-distribution testing (Humble, Iron, Jazzy)

## Known Limitations

### Current
- Single ROS2 distribution hardcoded (Humble)
- Basic error messages
- No service templates
- Manual testing only

### Accepted Limitations (By Design)
- No advanced orchestration features
- No distributed management
- No web UI or graphical interface
- No automatic dependency resolution
- No service composition

## Support Matrix

### ROS2 Distributions
- Humble: Full support
- Iron: Planned
- Jazzy: Planned

### Operating Systems
- Ubuntu 22.04: Primary
- Ubuntu 20.04: Supported
- Debian 11: Best effort

### Python Versions
- 3.8: Minimum
- 3.9-3.11: Supported

### Systemd Versions
- 230+: Required (Ubuntu 20.04+)

## Contributing

We welcome contributions for:

1. **Bug Reports**: Issues with service management
2. **Testing**: Verification on different systems
3. **Documentation**: Improved examples and guides
4. **Templates**: Service templates for popular packages
5. **Compatibility**: Testing with different ROS2 distributions

## Out of Scope

The following features are explicitly out of scope for this project:

- Web dashboards or GUIs
- Service orchestration
- Distributed management
- CI/CD integrations
- Advanced monitoring and metrics
- Service mesh or clustering
- Container orchestration
- Enterprise features

This project aims to remain a simple, focused tool for basic systemd service management of ROS2 nodes and launches.

## Contact

- Issue Tracker: GitHub Issues
- Documentation: README.md, DESIGN.md