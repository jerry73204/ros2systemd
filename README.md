# ros2systemd

A ROS2 command extension for managing ROS2 launches and nodes as systemd services. Turn your ROS2 nodes into reliable system services that start on boot, restart on failure, and integrate with system logging.

## Quick Start

### Installation

From GitHub (recommended):
```bash
pip install git+https://github.com/jerry73204/ros2systemd.git
```

From source:
```bash
git clone https://github.com/jerry73204/ros2systemd.git
cd ros2systemd
pip install .
```

For ROS2 workspace development:
```bash
cd ~/ros2_ws/src
git clone https://github.com/jerry73204/ros2systemd.git
cd ..
colcon build --packages-select ros2systemd
source install/setup.bash
```

### Basic Example

Here's a complete workflow to manage a ROS2 node as a systemd service:

```bash
# 1. Create a service for the demo talker node
ros2 systemd create talker-demo node demo_nodes_cpp talker

# 2. List all ROS2 systemd services
ros2 systemd list

# 3. Start the service
ros2 systemd start talker-demo

# 4. Check service status
ros2 systemd status talker-demo

# 5. View service logs
ros2 systemd logs talker-demo

# 6. Restart the service
ros2 systemd restart talker-demo

# 7. Stop the service
ros2 systemd stop talker-demo

# 8. Enable service to start on boot
ros2 systemd enable talker-demo

# 9. Remove the service when no longer needed
ros2 systemd remove talker-demo
```

### Launch File Example

Managing a launch file with multiple nodes:

```bash
# Create service from a launch file
ros2 systemd create robot-bringup launch demo_nodes_cpp talker_listener.launch.py

# Start and enable for automatic startup
ros2 systemd start robot-bringup
ros2 systemd enable robot-bringup

# Monitor the service
ros2 systemd status robot-bringup
ros2 systemd logs robot-bringup --follow
```

### Environment Configuration Example

Create services with specific ROS2 configurations:

```bash
# Service with specific domain ID and RMW implementation
ros2 systemd create my-robot node my_package my_node \
    --domain-id 42 \
    --rmw rmw_cyclonedds_cpp \
    --localhost-only 1

# The service inherits your current environment if not specified
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 systemd create another-robot node my_package my_node
# Service will use domain ID 10 and FastRTPS from your shell
```

## Command Reference

### Create Services

Create a service for a ROS2 node:
```bash
ros2 systemd create <service-name> node <package> <executable> [options]
```

Create a service for a launch file:
```bash
ros2 systemd create <service-name> launch <package_or_path> [launch-file] [options]
```

#### Options:
- `--domain-id N`: Set ROS_DOMAIN_ID (0-232)
- `--rmw IMPL`: Set RMW implementation (e.g., rmw_fastrtps_cpp)
- `--localhost-only {0,1}`: Set ROS_LOCALHOST_ONLY
- `--env KEY=VALUE`: Add custom environment variables
- `--system`: Create system-wide service (requires sudo)
- `--network-isolation`: Enable network isolation (only works with --system)

### Service Management

```bash
# Start/stop services
ros2 systemd start <service-name>
ros2 systemd stop <service-name>
ros2 systemd restart <service-name>

# Enable/disable automatic startup
ros2 systemd enable <service-name>   # Start on boot
ros2 systemd disable <service-name>  # Don't start on boot

# Check service status
ros2 systemd status <service-name>

# View logs
ros2 systemd logs <service-name>
ros2 systemd logs <service-name> --follow    # Real-time logs
ros2 systemd logs <service-name> --lines 50   # Last 50 lines

# List all services
ros2 systemd list

# Remove a service
ros2 systemd remove <service-name>
```

### Diagnostic Tools

Diagnose environment mismatches between services and the ROS2 daemon:
```bash
ros2 systemd diagnose [service-name]
```

Generate service templates:
```bash
# Show template for a node service
ros2 systemd template node my_package my_node

# Show template for a launch service  
ros2 systemd template launch my_package my_launch.py

# Save template to file
ros2 systemd template node my_package my_node > my-service.txt
```

## Advanced Usage

### System vs User Services

By default, services are created as **user services** that run under your user account:
```bash
ros2 systemd create my-service node demo_nodes_cpp talker
```

For system-wide services that start before user login (requires sudo):
```bash
ros2 systemd create my-service node demo_nodes_cpp talker --system
sudo systemctl start ros2-my-service
```

### Network Isolation

Network isolation prevents services from interfering with each other but requires system services:
```bash
# Create isolated service (requires sudo for system services)
ros2 systemd create isolated-talker node demo_nodes_cpp talker \
    --system \
    --network-isolation

# Alternative isolation methods for user services
ros2 systemd create isolated-talker node demo_nodes_cpp talker \
    --domain-id 100 \
    --localhost-only 1
```

### Working with the ROS2 Daemon

The ROS2 daemon can cause discovery issues when services use different configurations. The `diagnose` command helps identify these problems:

```bash
# Check for environment mismatches
ros2 systemd diagnose

# If issues are found, restart the daemon with matching settings
ros2 daemon stop
export ROS_DOMAIN_ID=<your-domain>
export RMW_IMPLEMENTATION=<your-rmw>
ros2 daemon start
```

### Custom Launch Arguments

Pass arguments to launch files:
```bash
ros2 systemd create nav-stack launch nav2_bringup bringup_launch.py \
    --launch-args use_sim_time:=true \
    --launch-args params_file:=/path/to/params.yaml
```

## Troubleshooting

### Services not visible in `ros2 topic list`

This usually happens due to environment mismatches:

1. Check service configuration:
   ```bash
   ros2 systemd diagnose my-service
   ```

2. If using different RMW or domain ID, restart the daemon:
   ```bash
   ros2 daemon stop
   # Set matching environment
   export ROS_DOMAIN_ID=<service-domain>
   export RMW_IMPLEMENTATION=<service-rmw>
   ros2 topic list
   ```

3. For isolated services, topics won't be visible by design.

### Permission Denied Errors

User services can't use certain features like network isolation. For these features, use system services with sudo:
```bash
ros2 systemd create my-service node pkg exe --system
sudo systemctl start ros2-my-service
```

### Service Fails to Start

Check logs for details:
```bash
ros2 systemd logs my-service --lines 100
```

Common issues:
- Missing dependencies: Ensure the package is installed
- Wrong paths: Verify launch file or node exists
- Environment issues: Check ROS2 is properly installed

## Requirements

- ROS2 (Humble, Iron, or newer)
- Python 3.8+
- systemd-based Linux distribution (Ubuntu 20.04+, Debian 11+, etc.)

## License

Apache License 2.0

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests on [GitHub](https://github.com/jerry73204/ros2systemd).
