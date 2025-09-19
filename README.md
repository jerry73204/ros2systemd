# ros2systemd

A ROS2 command extension for managing ROS2 launches and nodes as systemd services. Turn your ROS2 nodes into reliable system services that start on boot, restart on failure, and integrate with system logging.

## Quick Start

### Installation

From GitHub release (recommended):

Download the wheel file from the [v0.4.0 release page](https://github.com/jerry73204/ros2systemd/releases/tag/v0.4.0) and install:

```bash
pip install https://github.com/jerry73204/ros2systemd/releases/download/v0.4.0/ros2systemd-0.4.0-py3-none-any.whl
```

From source:
```bash
# Activate the ROS environment
source /opt/ros/humble/setup.bash

git clone https://github.com/jerry73204/ros2systemd.git
cd ros2systemd
pip install .
```

### Basic Usage - Running Nodes as Services

You already know how to run ROS2 nodes:

```bash
# Traditional way - runs in foreground, stops when terminal closes
ros2 run demo_nodes_cpp talker
```

Now run the same node as a persistent systemd service:

```bash
# Run as a service - runs in background, survives terminal closure
ros2 systemd run demo_nodes_cpp talker

# Stop and remove the service when done
ros2 systemd stop demo_nodes_cpp-talker-1234567890  # (auto-generated name)
ros2 systemd remove demo_nodes_cpp-talker-1234567890
```

### Launch Files as Services

You already know how to launch ROS2 launch files:

```bash
# Traditional way - runs in foreground, stops when terminal closes
ros2 launch demo_nodes_cpp talker_listener.launch.py
```

Now run the same launch file as a persistent systemd service:

```bash
# Launch as a service - runs in background, survives terminal closure
ros2 systemd launch demo_nodes_cpp talker_listener.launch.py

# Stop and remove the service when done
ros2 systemd stop demo_nodes_cpp-talker_listener-1234567890  # (auto-generated name)
ros2 systemd remove demo_nodes_cpp-talker_listener-1234567890
```

### Detailed Usage

#### Service Management Workflow

For more control over the service lifecycle, use the traditional create/start/stop workflow:

```bash
# 1. Create a service for the demo talker node
ros2 systemd create talker-demo node demo_nodes_cpp talker

# 2. Start the service
ros2 systemd start talker-demo

# 3. Check service status and logs
ros2 systemd status talker-demo
ros2 systemd logs talker-demo

# 4. Stop and remove when done
ros2 systemd stop talker-demo
ros2 systemd remove talker-demo
```

#### Advanced Options

```bash
# Custom service name and arguments
ros2 systemd run --name my-talker demo_nodes_cpp talker -- --ros-args -p frequency:=2.0

# Environment settings
ros2 systemd run --domain-id 42 --rmw rmw_cyclonedds_cpp demo_nodes_cpp talker

# Launch files with arguments
ros2 systemd launch --name my-demo demo_nodes_cpp talker_listener.launch.py use_sim_time:=true

# Enable automatic startup on boot
ros2 systemd create my-service node demo_nodes_cpp talker
ros2 systemd start my-service
ros2 systemd enable my-service  # Starts automatically on boot
```

#### Environment Configuration

Create services with specific ROS2 configurations:

```bash
# Service with specific domain ID and RMW implementation
ros2 systemd create my-robot node my_package my_node \
    --domain-id 42 \
    --rmw rmw_cyclonedds_cpp \
    --localhost-only 1

# The service automatically inherits your current ROS environment
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 systemd create another-robot node my_package my_node
# Service will capture and use domain ID 10 and FastRTPS from your shell

# Use custom environment setup scripts
ros2 systemd create workspace-robot --source ~/my_workspace/install/setup.bash \
    node my_package my_node

# Disable environment capture and use explicit setup
ros2 systemd create minimal-robot --env-mode none \
    --source /opt/ros/humble/setup.bash \
    node demo_nodes_cpp talker
```

## Command Reference

```bash
# Quick commands (create + start)
ros2 systemd run [options] <package> <executable> [-- <args>]
ros2 systemd launch [options] <package> [launch-file] [<launch-args>]

# Service lifecycle
ros2 systemd create <service-name> {node|launch} <package> <executable|launch-file> [options]
ros2 systemd start|stop|restart <service-name>
ros2 systemd enable|disable <service-name>
ros2 systemd status <service-name>
ros2 systemd logs <service-name> [--follow] [--lines N]
ros2 systemd list
ros2 systemd remove <service-name>

# Diagnostics
ros2 systemd diagnose [service-name]
ros2 systemd template {node|launch} <package> <executable|launch-file>
```

**Note**: Use `ros2 systemd <command> --help` for detailed options and examples.

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

### ROS Arguments with Nodes

Pass ROS arguments to nodes (requires `--` delimiter):
```bash
# Node with ROS parameters
ros2 systemd create param-talker node demo_nodes_cpp talker -- \
    --ros-args -p frequency:=10.0 -p topic_name:=my_topic

# Node with remapping
ros2 systemd create remapped-listener node demo_nodes_cpp listener -- \
    --ros-args -r chatter:=my_topic
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
