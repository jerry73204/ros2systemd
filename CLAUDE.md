# Claude Code Session Guide for ros2systemd

## Project Overview
This is a ROS2 command extension that manages ROS2 nodes and launch files as systemd services. The project is named `ros2systemd` (no underscore).

## Key Information

### Package Structure
- **Package name**: `ros2systemd` (renamed from `ros2_systemd`)
- **Main module**: `ros2systemd/`
- **Command entry**: `ros2 systemd <verb> <args>`
- **GitHub repo**: https://github.com/jerry73204/ros2systemd

### Testing
```bash
# Run all tests
colcon test

# Or using make
make test

# Run specific test types
make lint      # Run flake8
make format    # Format with black
```

### Building
```bash
# Clean build
make clean
colcon build

# Or just
make build
```

### Installation Methods
1. **pip install from source**: `pip install .`
2. **pip install from GitHub**: `pip install git+https://github.com/jerry73204/ros2systemd.git`
3. **Colcon build**: `colcon build --packages-select ros2systemd`

## Important Technical Details

### Network Isolation Limitation
- `PrivateNetwork=yes` does NOT work for user systemd services (requires root)
- The code correctly sets the flag but it has no effect for user services
- Warning is shown to users when they try to use `--network-isolation` without `--system`
- This is a systemd limitation, not a bug in our code

### ROS2 Daemon Sticky Behavior
- The ROS2 daemon maintains the environment (RMW, domain ID) from when it was first started
- Services with different settings won't be discoverable via `ros2 topic list`
- The `diagnose` command helps identify these mismatches
- Solution: Stop daemon and restart with matching environment

### Environment Variable Inheritance
- When flags like `--domain-id`, `--rmw`, `--localhost-only` are NOT specified, the service inherits values from the current shell
- The create command shows what values are being used and their source (shell/specified/default)

## Code Style
- **Line length**: 120 characters
- **Formatter**: black
- **Linter**: flake8 with some ignores (Q000, D100-D107)
- **No docstring requirements** for tests
- **Import sorting**: stdlib, third-party, local (enforced by isort)

## Common Tasks

### Adding a New Verb
1. Create new file in `ros2systemd/verb/`
2. Inherit from `VerbExtension`
3. Add entry point in `setup.py` under `ros2systemd.verb`
4. Implement `add_arguments()` and `main()` methods

### Modifying Service Creation
- Main logic in `ros2systemd/api/systemd_manager.py`
- Service file generation in `_generate_service_content()` method
- Environment handling in `ros2systemd/verb/create.py`

### Testing Changes
```bash
# Quick test after changes
colcon build && source install/setup.bash
ros2 systemd create test node demo_nodes_cpp talker
ros2 systemd list
ros2 systemd remove test
```

## Known Issues and Solutions

### Issue: Topics not visible in `ros2 topic list`
**Cause**: Environment mismatch between service and daemon
**Solution**: Use `ros2 systemd diagnose` to check, restart daemon with matching environment

### Issue: Network isolation not working
**Cause**: User services can't use PrivateNetwork (systemd limitation)
**Solution**: Use `--system` flag or alternative isolation (different domain ID, localhost-only)

### Issue: Service fails to start
**Check**:
1. `ros2 systemd logs <service-name>`
2. Verify package/executable exists
3. Check ROS2 environment is sourced

## File Naming Conventions
- Service names get `ros2-` prefix automatically
- Service files stored in:
  - User: `~/.config/systemd/user/ros2-*.service`
  - System: `/etc/systemd/system/ros2-*.service`

## Development Tips

1. **Always test both user and system services** (system requires sudo)
2. **Use demo_nodes_cpp for testing** - it's always available in ROS2
3. **Check daemon status** when debugging discovery issues
4. **Read service logs directly** with `journalctl` for detailed debugging
5. **Format before committing**: `make format`
6. **Run tests before pushing**: `colcon test`

## Recent Changes (Last Session)
- Renamed project from `ros2_systemd` to `ros2systemd`
- Revised README with examples-first approach
- Added pip installation support with pyproject.toml
- Added warning for network isolation with user services
- Added environment variable inheritance from shell
- Removed confirmation prompt from `ros2 systemd remove` command

## Contact
- GitHub: https://github.com/jerry73204/ros2systemd
- Issues: https://github.com/jerry73204/ros2systemd/issues