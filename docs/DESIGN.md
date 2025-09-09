# ROS2-Systemd Extension Design Document

## Table of Contents
1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Service Naming Convention](#service-naming-convention)
4. [Service Isolation](#service-isolation)
5. [Command Behavior](#command-behavior)
6. [Implementation Details](#implementation-details)
7. [Security Considerations](#security-considerations)
8. [Design Rationale](#design-rationale)

## Overview

The ROS2-Systemd extension provides a seamless interface between ROS2 and systemd, enabling users to manage ROS2 nodes and launch files as systemd services. The design prioritizes safety, usability, and clear separation between user and system contexts.

### Core Principles
1. **Safe by Default**: User services require no elevated privileges
2. **Clear Namespace Separation**: User and system services are isolated
3. **Predictable Behavior**: Commands work consistently across contexts
4. **ROS2 Integration**: Follows ROS2 CLI patterns and conventions

## Architecture

### Component Structure
```
ros2-systemd/
├── ros2_systemd/
│   ├── api/                    # Core functionality
│   │   └── systemd_manager.py  # SystemD interaction layer
│   ├── command/                 # CLI entry point
│   │   └── systemd.py          # Main command handler
│   └── verb/                    # Command implementations
│       ├── create.py           # Service creation
│       ├── start.py            # Service control
│       ├── stop.py             # Service control
│       ├── restart.py          # Service control
│       ├── status.py           # Service monitoring
│       ├── enable.py           # Persistence control
│       ├── disable.py          # Persistence control
│       ├── list.py             # Service discovery
│       └── remove.py           # Service deletion
```

### Layered Design
1. **CLI Layer**: ROS2 CLI plugin interface
2. **Verb Layer**: Command parsing and validation
3. **API Layer**: Business logic and systemd interaction
4. **System Layer**: Direct systemd communication

## Service Naming Convention

### Naming Structure
All services follow a strict naming convention to ensure uniqueness and prevent conflicts:

```
ros2-<user-provided-name>
```

#### Examples:
- User provides: `navigation` → Service name: `ros2-navigation`
- User provides: `my-robot` → Service name: `ros2-my-robot`

### Namespace Isolation
Services are isolated by systemd's built-in user/system separation:

#### User Services
- Full name: `ros2-<name>.service`
- Location: `~/.config/systemd/user/`
- Scope: Current user only
- Example: `ros2-navigation.service` (user alice)

#### System Services
- Full name: `ros2-<name>.service`
- Location: `/etc/systemd/system/`
- Scope: System-wide
- Example: `ros2-navigation.service` (system)

### Name Collision Handling
**Important**: The same service name CAN exist in both user and system contexts without conflict:

```
# These are two completely different services:
~/.config/systemd/user/ros2-navigation.service     # Alice's navigation
/etc/systemd/system/ros2-navigation.service        # System navigation
```

This is by design - systemd maintains separate namespaces for user and system services.

## Service Isolation

### User Service Context
```bash
# When --system flag is NOT used (default):
ros2 systemd list                  # Lists only current user's services
ros2 systemd start navigation      # Starts user's ros2-navigation
ros2 systemd status navigation     # Shows user's service status
```

**Characteristics:**
- Cannot see or affect other users' services
- Cannot see or affect system services
- Runs with user's permissions
- Environment inherited from user session

### System Service Context
```bash
# When --system flag IS used:
ros2 systemd list --system         # Lists only system services
ros2 systemd start navigation --system    # Starts system's ros2-navigation
ros2 systemd status navigation --system   # Shows system's service status
```

**Characteristics:**
- Cannot see or affect any user's services
- Requires sudo for modifications
- Runs with configured permissions (usually root)
- Clean environment, explicitly configured

### Cross-Context Visibility
The `--all` flag (only for list command) shows both contexts:
```bash
ros2 systemd list --all
# Output:
# === User Services ===
# navigation     loaded    active
# 
# === System Services ===
# navigation     loaded    active
```

Note: Even though both are named "navigation", they are completely independent services.

## Command Behavior

### Service Creation
```bash
# Creates user service (default)
ros2 systemd create <name> ...

# Creates system service (requires sudo)
sudo ros2 systemd create <name> ... --system
```

**Design Choice**: Creation requires explicit `--system` flag to prevent accidental system modifications.

### Service Management Commands

All management commands follow the same pattern:

| Command | User Context (Default)        | System Context                              |
|---------|-------------------------------|---------------------------------------------|
| start   | `ros2 systemd start <name>`   | `sudo ros2 systemd start <name> --system`   |
| stop    | `ros2 systemd stop <name>`    | `sudo ros2 systemd stop <name> --system`    |
| restart | `ros2 systemd restart <name>` | `sudo ros2 systemd restart <name> --system` |
| enable  | `ros2 systemd enable <name>`  | `sudo ros2 systemd enable <name> --system`  |
| disable | `ros2 systemd disable <name>` | `sudo ros2 systemd disable <name> --system` |
| remove  | `ros2 systemd remove <name>`  | `sudo ros2 systemd remove <name> --system`  |

### Query Commands

Query commands can be run without sudo even for system services:

| Command | User Context | System Context |
|---------|--------------|----------------|
| status  | `ros2 systemd status <name>` | `ros2 systemd status <name> --system` |
| list    | `ros2 systemd list` | `ros2 systemd list --system` |

**Design Choice**: Read-only operations don't require elevated privileges.

## Implementation Details

### Service File Generation

#### User Service Template
```ini
[Unit]
Description=<user-provided-description>
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && <command>'
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
Environment="ROS_DOMAIN_ID=0"
Environment="ROS_LOCALHOST_ONLY=0"

[Install]
WantedBy=default.target    # User services use default.target
```

#### System Service Template
```ini
[Unit]
Description=<user-provided-description>
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && <command>'
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
Environment="ROS_DOMAIN_ID=0"
Environment="ROS_LOCALHOST_ONLY=0"

[Install]
WantedBy=multi-user.target  # System services use multi-user.target
```

### SystemdServiceManager Class

The `SystemdServiceManager` class maintains context through its lifetime:

```python
class SystemdServiceManager:
    def __init__(self, user_mode: bool = True):
        self.user_mode = user_mode
        self.service_dir = self.SYSTEMD_USER_DIR if user_mode else self.SYSTEMD_SYSTEM_DIR
```

**Design Choice**: Context is immutable after initialization to prevent confusion.

### Command Execution Flow

1. **Parse Arguments**: Determine if `--system` flag is present
2. **Create Manager**: Initialize with appropriate context
3. **Execute Operation**: Perform requested action in context
4. **Return Results**: Provide context-aware feedback

Example flow for `start` command:
```python
def main(self, *, args):
    # Step 1: Determine context from args
    user_mode = not args.system
    
    # Step 2: Create context-aware manager
    manager = SystemdServiceManager(user_mode=user_mode)
    
    # Step 3: Execute in context
    success, message = manager.start_service(args.service_name)
    
    # Step 4: Provide feedback
    if success:
        context = "system" if args.system else "user"
        print(f"Started {context} service 'ros2-{args.service_name}'")
```

## Security Considerations

### Privilege Escalation Prevention
- User services cannot affect system services
- User services cannot affect other users' services
- System operations require explicit sudo

### Safe Defaults
- Default to user context (no privileges needed)
- Require explicit `--system` flag for system operations
- Confirmation prompts for destructive operations (remove)

### Service Isolation
- User services run with user's permissions only
- System services run in clean environment
- No automatic privilege inheritance

### Attack Surface Minimization
- Service names are prefixed to prevent hijacking
- Service files are created with appropriate permissions
- No arbitrary code execution without explicit file paths

## Design Rationale

### Why Separate User and System Contexts?

1. **Security**: Prevents accidental system-wide changes
2. **Multi-user Support**: Each user has independent services
3. **Development vs Production**: User services for development, system for production
4. **Permission Management**: No sudo required for personal robots

### Why the ros2- Prefix?

1. **Namespace Clarity**: Immediately identifiable as ROS2-managed
2. **Conflict Prevention**: Won't collide with manually created services
3. **Discoverability**: Easy to list all ROS2 services with wildcards
4. **Cleanup**: Simple to identify and remove all ROS2 services

### Why Default to User Services?

1. **Accessibility**: New users can start immediately without admin access
2. **Safety**: Mistakes affect only the current user
3. **Development Workflow**: Matches typical ROS2 development patterns
4. **Teaching**: Safer for educational environments

### Why Not Auto-detect Context?

1. **Explicit is Better**: Clear intention prevents surprises
2. **Consistency**: Same command structure everywhere
3. **Scriptability**: Scripts can rely on predictable behavior
4. **Audit Trail**: Clear in logs what context was intended

## Future Considerations

### Potential Minor Enhancements

1. **Service Templates**: Pre-configured services for common packages
2. **Better Error Messages**: More descriptive error reporting
3. **Log Viewing**: Simple journalctl integration for logs

### Compatibility

1. **Systemd Versions**: Support systemd 230+ (Ubuntu 20.04+)
2. **ROS2 Distributions**: Test with Humble, Iron, Jazzy

## Conclusion

The ROS2-Systemd extension design prioritizes safety, clarity, and usability while maintaining the power and flexibility of systemd. The clear separation between user and system contexts, combined with consistent naming conventions and predictable command behavior, creates a robust tool for managing ROS2 services in production and development environments.
