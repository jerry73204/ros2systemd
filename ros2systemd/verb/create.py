import argparse

from ros2systemd.api.systemd_manager import SystemdServiceManager
from ros2systemd.verb import VerbExtension


class CreateVerb(VerbExtension):
    r"""Create a systemd service for a ROS2 launch file or node.

    Examples:
        # Create service for a launch file with full path
        ros2 systemd create my-robot launch /path/to/robot.launch.py

        # Create service for a launch file from package
        ros2 systemd create talker-listener launch demo_nodes_cpp talker_listener.launch.py

        # Create service for a launch file with launch arguments
        ros2 systemd create my-robot --domain-id 42 launch my_package robot.launch.py use_sim_time:=true

        # Create service for a node
        ros2 systemd create my-talker node demo_nodes_cpp talker

        # Create service for a node with node arguments
        ros2 systemd create my-talker --domain-id 42 node demo_nodes_cpp talker -- --ros-args -p frequency:=2.0

        # With environment options
        ros2 systemd create my-service --domain-id 42 --rmw rmw_cyclonedds_cpp --env CUSTOM_VAR=value \
            node pkg exe

        # With network isolation (no daemon interaction)
        ros2 systemd create isolated-node --network-isolation node pkg exe
    """

    def add_arguments(self, parser, cli_name):
        # Add examples to the parser
        parser.epilog = """
Examples:
  # Create service for a launch file with full path
  ros2 systemd create my-robot launch /path/to/robot.launch.py

  # Create service for a launch file from package
  ros2 systemd create talker-listener launch demo_nodes_cpp talker_listener.launch.py

  # Create service for a launch file with launch arguments
  ros2 systemd create my-robot --domain-id 42 launch my_package robot.launch.py use_sim_time:=true

  # Create service for a node
  ros2 systemd create my-talker node demo_nodes_cpp talker

  # Create service for a node with node arguments
  ros2 systemd create my-talker --domain-id 42 node demo_nodes_cpp talker -- --ros-args -p frequency:=2.0

  # With environment options
  ros2 systemd create my-service --domain-id 42 --rmw rmw_cyclonedds_cpp --env CUSTOM_VAR=value \\
      node pkg exe

  # With network isolation (no daemon interaction)
  ros2 systemd create isolated-node --network-isolation node pkg exe

  # Create system-wide service (requires sudo)
  ros2 systemd create my-system-service --system node pkg exe

Environment Options:
  --env-mode MODE   Environment copying mode (ros/all/none, default: ros)
                    - 'ros': Copy ROS/DDS variables only (safe, recommended)
                    - 'all': Copy all environment variables (security risk warning)
                    - 'none': Copy only explicitly specified variables
  --source PATH     Source a setup script before running (can be used multiple times)
  --copy-env KEY    Copy specific environment variable from current shell
  --env KEY=VALUE   Set environment variable
  --domain-id N     Sets ROS_DOMAIN_ID=N (valid range: 0-232)
  --rmw IMPL        Sets RMW_IMPLEMENTATION=IMPL

  By default (--env-mode=ros), ROS/DDS environment variables are captured.
  This includes AMENT_PREFIX_PATH, ROS_*, RMW_*, CYCLONEDDS_*, FASTRTPS_*, etc.

  Examples:
    # Use current environment (default, recommended)
    ros2 systemd create my-service node pkg exe

    # Add extra setup script to current environment
    ros2 systemd create my-service --source ~/special/setup.bash node pkg exe

    # Use only explicit setup scripts
    ros2 systemd create my-service --env-mode=none \\
      --source /opt/ros/humble/setup.bash \\
      --source ~/ws/install/setup.bash \\
      node pkg exe

Network Isolation:
  --network-isolation creates a service that runs in an isolated network namespace.

  IMPORTANT: Network isolation (PrivateNetwork=yes) requires root privileges.
  It only works with system services (--system flag), not user services.
  For user services, consider using:
  - ROS_LOCALHOST_ONLY=1 for partial isolation
  - Different ROS_DOMAIN_ID values for logical isolation

  When working with system services:
  - Services that should not affect the global ROS2 discovery
  - Testing isolated components
  - Services with different RMW implementations that shouldn't conflict
"""
        parser.formatter_class = argparse.RawDescriptionHelpFormatter

        # Service name comes first
        parser.add_argument("service_name", help="Name for the systemd service (will be prefixed with ros2-)")

        # Systemd-specific flags (before the subcommand)
        parser.add_argument(
            "--env",
            action="append",
            metavar="KEY=VALUE",
            help="Additional environment variables in KEY=VALUE format (can be used multiple times)",
        )
        parser.add_argument(
            "--copy-env",
            action="append",
            metavar="KEY",
            help="Copy specific environment variable from current shell (can be used multiple times)",
        )
        parser.add_argument(
            "--source",
            action="append",
            metavar="PATH",
            help="Source a setup script before running (can be used multiple times)",
        )
        parser.add_argument(
            "--env-mode",
            choices=["ros", "all", "none"],
            default="ros",
            help="Environment variable copying mode: 'ros' (ROS/DDS variables only, default), 'all' (all variables), 'none' (explicit only)",
        )
        parser.add_argument(
            "--domain-id",
            type=int,
            metavar="ID",
            help="ROS domain ID (0-232), sets ROS_DOMAIN_ID (uses current shell value if not specified)",
        )
        parser.add_argument(
            "--rmw",
            metavar="IMPL",
            help="RMW implementation, e.g., rmw_fastrtps_cpp (uses current shell value if not specified)",
        )
        parser.add_argument(
            "--localhost-only",
            choices=["0", "1"],
            help="Set ROS_LOCALHOST_ONLY (0=disabled, 1=enabled, uses current shell value if not specified)",
        )
        parser.add_argument(
            "--network-isolation",
            action="store_true",
            help="Enable network isolation - service runs in isolated network namespace (no daemon interaction)",
        )
        parser.add_argument("--description", metavar="TEXT", help="Custom service description text")
        parser.add_argument(
            "--system",
            action="store_true",
            help="Create system-wide service instead of user service (requires sudo)",
        )

        # Service type (launch or node)
        parser.add_argument(
            "service_type", choices=["launch", "node"], help="Type of service to create (launch or node)"
        )

        # Package/path specification
        parser.add_argument(
            "package_or_path", help="For 'launch': package name or /full/path/to/launch.py\n" "For 'node': package name"
        )

        # Optional second positional for executable/launch file
        parser.add_argument(
            "file_or_executable",
            nargs="?",
            help="For 'launch': launch file name (when package name is provided)\n"
            "For 'node': executable name (required)",
        )

        # Remaining arguments after '--' are either launch args (KEY:=VALUE) or node args
        # Using REMAINDER to capture everything after the last positional argument
        parser.add_argument(
            "extra_args",
            nargs=argparse.REMAINDER,
            help="For 'launch': launch arguments in KEY:=VALUE format\n"
            "For 'node': additional arguments to pass to the node\n"
            "Note: Use '--' before extra arguments if they contain flags starting with '-'\n"
            "Example: ros2 systemd create name node pkg exe -- --ros-args -p param:=value",
        )

    def _capture_environment(self, mode="ros"):
        """Capture environment variables based on the specified mode."""
        import os

        if mode == "none":
            return {}

        if mode == "all":
            # Copy all environment variables except those we handle specially
            special_vars = {"ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION"}
            return {k: v for k, v in os.environ.items() if k not in special_vars}

        # mode == "ros" - ROS/DDS-specific variables only
        captured_vars = {}

        # Core ROS/Ament paths
        core_keys = [
            "AMENT_PREFIX_PATH",
            "CMAKE_PREFIX_PATH",
            "LD_LIBRARY_PATH",
            "PATH",
            "PYTHONPATH",
            "PKG_CONFIG_PATH",
            "COLCON_PREFIX_PATH",
        ]

        # DDS-specific configuration variables
        dds_keys = [
            "CYCLONEDDS_URI",
            "CYCLONEDDS_NETWORK_INTERFACE",
            "CYCLONEDDS_PEER_ADDRESSES",
            "FASTRTPS_DEFAULT_PROFILES_FILE",
            "RMW_FASTRTPS_USE_QOS_FROM_XML",
            "RMW_FASTRTPS_PUBLICATION_MODE",
        ]

        # Variables we handle specially
        special_vars = {"ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION"}

        for key, value in os.environ.items():
            if key in core_keys or key in dds_keys:
                captured_vars[key] = value
            elif key.startswith("ROS_") and key not in special_vars:
                captured_vars[key] = value
            elif key.startswith("RMW_") and key not in special_vars:
                captured_vars[key] = value
            elif key.startswith("CYCLONEDDS_"):
                captured_vars[key] = value
            elif key.startswith("FASTRTPS_"):
                captured_vars[key] = value

        return captured_vars

    def main(self, *, args):
        import os
        from pathlib import Path

        # Handle '--' delimiter validation for extra_args
        # Note: We need to check sys.argv to see if '--' was actually used
        # because argparse.REMAINDER doesn't preserve the '--' delimiter
        import sys

        extra_args = args.extra_args
        delimiter_used = "--" in sys.argv

        if extra_args:
            # Check if any arguments start with dashes
            has_dash_args = any(arg.startswith("-") for arg in extra_args)
            if has_dash_args and not delimiter_used:
                print("Error: Arguments starting with '-' must be preceded by '--' delimiter.")
                print("Example: ros2 systemd create name node pkg exe -- --ros-args -p param:=value")
                return 1
            elif delimiter_used:
                # Validate that '--' was in the right position
                # Find where extra_args would start in sys.argv
                try:
                    # Look for the delimiter after the main arguments
                    delimiter_index = sys.argv.index("--")
                    # Check if there are any create-command flags after the delimiter
                    args_after_delimiter = sys.argv[delimiter_index + 1 :]

                    # The extra_args should match what comes after '--'
                    if args_after_delimiter != extra_args:
                        print("Warning: Argument parsing mismatch detected.")
                        print(f"Expected: {args_after_delimiter}")
                        print(f"Got: {extra_args}")

                except ValueError:
                    # This shouldn't happen if delimiter_used is True, but handle gracefully
                    pass

        manager = SystemdServiceManager(user_mode=not args.system)

        # Determine environment capture mode
        env_mode = args.env_mode

        # Add security warning for 'all' mode
        if env_mode == "all":
            print("⚠️  WARNING: --env-mode=all copies ALL environment variables!")
            print("   This may expose sensitive data (SSH keys, tokens, etc.) via systemd.")
            print("   Environment variables will be visible via 'systemctl show' command.")
            print("   Use --env-mode=ros for safer ROS-only variable copying.")
            print()

        # Capture environment variables based on mode
        env_vars = {}
        captured_env = self._capture_environment(env_mode)
        env_vars.update(captured_env)

        # Handle explicitly copied environment variables
        if args.copy_env:
            for key in args.copy_env:
                if key in os.environ:
                    env_vars[key] = os.environ[key]
                else:
                    print(f"Warning: Environment variable '{key}' not found in current shell")

        # Parse additional environment variables from --env
        if args.env:
            for env_var in args.env:
                if "=" in env_var:
                    key, value = env_var.split("=", 1)
                    env_vars[key] = value
                else:
                    print(f"Warning: Invalid environment variable format '{env_var}' (expected KEY=VALUE)")

        # Handle ROS_DOMAIN_ID - use shell value if not specified
        if args.domain_id is not None:
            env_vars["ROS_DOMAIN_ID"] = str(args.domain_id)
        elif "ROS_DOMAIN_ID" not in env_vars:
            # Copy from shell environment if available, otherwise use default
            env_vars["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "0")

        # Handle RMW_IMPLEMENTATION - use shell value if not specified
        if args.rmw:
            env_vars["RMW_IMPLEMENTATION"] = args.rmw
        elif "RMW_IMPLEMENTATION" not in env_vars:
            # Copy from shell environment if available, otherwise use default
            env_vars["RMW_IMPLEMENTATION"] = os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

        # Handle ROS_LOCALHOST_ONLY - use shell value if not specified
        if args.localhost_only:
            env_vars["ROS_LOCALHOST_ONLY"] = args.localhost_only
        elif "ROS_LOCALHOST_ONLY" not in env_vars:
            # Copy from shell environment if available, otherwise use default
            env_vars["ROS_LOCALHOST_ONLY"] = os.environ.get("ROS_LOCALHOST_ONLY", "0")

        # Process source scripts
        source_scripts = []
        if args.source:
            for script_path in args.source:
                resolved_path = Path(script_path).expanduser().resolve()
                if not resolved_path.exists():
                    print(f"Warning: Source script not found: {script_path}")
                else:
                    source_scripts.append(str(resolved_path))

            # Warn if using both source scripts and environment capture
            if env_mode != "none" and source_scripts:
                print("⚠️  Note: Setup scripts specified. ROS/Ament environment will still be captured.")
                print("   Use --env-mode=none to disable environment capture.")
                print()

        # Warn if no environment setup is provided
        if env_mode == "none" and not source_scripts:
            has_ros_env = any(key in env_vars for key in ["AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"])
            if not has_ros_env:
                print("⚠️  Warning: No ROS environment setup provided!")
                print("   - Environment capture is disabled (--env-mode=none)")
                print("   - No source scripts specified (--source)")
                print("   The service may fail to find ROS2 commands and packages.")
                print("   Consider either:")
                print("   1. Use --env-mode=ros to capture current environment")
                print("   2. Add --source /opt/ros/humble/setup.bash or your workspace setup")
                print()

        # Warn about network isolation limitations
        if args.network_isolation and not args.system:
            print("⚠️  Warning: Network isolation (PrivateNetwork=yes) requires root privileges.")
            print("   It will NOT work with user services. Consider using --system flag with sudo,")
            print("   or use ROS_LOCALHOST_ONLY=1 / different ROS_DOMAIN_ID for isolation.")
            print()

        if args.service_type == "launch":
            # Determine launch file path
            if args.file_or_executable:
                # Package name + launch file format
                package_name = args.package_or_path
                launch_file = args.file_or_executable
                resolved_path = manager._resolve_package_path(package_name, launch_file)
                if resolved_path:
                    launch_file_path = str(resolved_path)
                else:
                    print(f"Error: Could not find launch file '{launch_file}' in package '{package_name}'")
                    return 1
            else:
                # Single argument - either full path or package name
                path_or_package = args.package_or_path

                if Path(path_or_package).exists():
                    # It's a full path
                    launch_file_path = path_or_package
                else:
                    # It's a package name - look for common launch files
                    for common_name in ["launch.py", "default.launch.py", f"{path_or_package}.launch.py"]:
                        resolved_path = manager._resolve_package_path(path_or_package, common_name)
                        if resolved_path:
                            launch_file_path = str(resolved_path)
                            print(f"Found launch file: {launch_file_path}")
                            break
                    else:
                        print(
                            f"Error: '{path_or_package}' is not a valid file path or package with default launch file"
                        )
                        return 1

            # Parse launch arguments from extra_args
            launch_args = {}
            if args.extra_args:
                for arg in args.extra_args:
                    if ":=" in arg:
                        key, value = arg.split(":=", 1)
                        launch_args[key] = value
                    else:
                        print(f"Warning: Ignoring invalid launch argument '{arg}' (expected KEY:=VALUE format)")

            # Create launch service
            success = manager.create_launch_service(
                service_name=args.service_name,
                launch_file=launch_file_path,
                launch_args=launch_args,
                env_vars=env_vars,
                source_scripts=source_scripts,
                description=args.description,
                network_isolation=args.network_isolation,
            )

            if success:
                print(f"Successfully created service 'ros2-{args.service_name}' for launch file '{launch_file_path}'")
                self._print_environment_info(env_vars, args.network_isolation, captured_env, source_scripts)
                print(f"Use 'ros2 systemd start {args.service_name}' to start the service")
                return 0
            else:
                print(f"Failed to create service 'ros2-{args.service_name}'")
                return 1

        elif args.service_type == "node":
            # For node, executable is required
            if not args.file_or_executable:
                print("Error: For node type, executable name is required")
                print("Usage: ros2 systemd create NAME [FLAGS...] node PACKAGE EXECUTABLE [ARGS...]")
                return 1

            package = args.package_or_path
            executable = args.file_or_executable
            node_args = args.extra_args  # All extra args are passed to the node

            # Create node service
            success = manager.create_node_service(
                service_name=args.service_name,
                package=package,
                executable=executable,
                node_args=node_args,
                env_vars=env_vars,
                source_scripts=source_scripts,
                description=args.description,
                network_isolation=args.network_isolation,
            )

            if success:
                print(f"Successfully created service 'ros2-{args.service_name}' " f"for node '{package}/{executable}'")
                self._print_environment_info(env_vars, args.network_isolation, captured_env, source_scripts)
                print(f"Use 'ros2 systemd start {args.service_name}' to start the service")
                return 0
            else:
                print(f"Failed to create service 'ros2-{args.service_name}'")
                return 1

    def _print_environment_info(self, env_vars, network_isolation, captured_env=None, source_scripts=None):
        """Print information about the environment variables set for the service."""
        import os

        print("\nEnvironment configuration:")

        # Print captured ROS/Ament environment info
        if captured_env:
            print("  ✓ Captured ROS/Ament environment from current shell")
            if "AMENT_PREFIX_PATH" in captured_env:
                paths = captured_env["AMENT_PREFIX_PATH"].split(":")
                if len(paths) > 2:
                    print(f"    - AMENT_PREFIX_PATH: {paths[0]}:...:{paths[-1]} ({len(paths)} paths)")
                else:
                    print(f"    - AMENT_PREFIX_PATH: {captured_env['AMENT_PREFIX_PATH']}")
            if "ROS_DISTRO" in captured_env:
                print(f"    - ROS_DISTRO: {captured_env['ROS_DISTRO']}")

            # Count other captured variables
            special_vars = {
                "AMENT_PREFIX_PATH",
                "ROS_DISTRO",
                "ROS_DOMAIN_ID",
                "RMW_IMPLEMENTATION",
                "ROS_LOCALHOST_ONLY",
            }
            other_count = len([k for k in captured_env.keys() if k not in special_vars])
            if other_count > 0:
                print(f"    - [{other_count} more variables captured]")

        # Print source scripts
        if source_scripts:
            print("  ✓ Setup scripts to source:")
            for i, script in enumerate(source_scripts, 1):
                print(f"    {i}. {script}")

        # Print main ROS environment variables
        print("\n  ✓ ROS Configuration:")
        domain_id = env_vars.get("ROS_DOMAIN_ID", "0")
        rmw = env_vars.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
        localhost = env_vars.get("ROS_LOCALHOST_ONLY", "0")

        # Indicate source of values
        domain_source = "(from shell)" if domain_id == os.environ.get("ROS_DOMAIN_ID") else "(specified)"
        rmw_source = "(from shell)" if rmw == os.environ.get("RMW_IMPLEMENTATION") else "(specified)"
        localhost_source = "(from shell)" if localhost == os.environ.get("ROS_LOCALHOST_ONLY") else "(specified)"

        # Use default indicator if shell didn't have the value
        if domain_id == "0" and "ROS_DOMAIN_ID" not in os.environ:
            domain_source = "(default)"
        if rmw == "rmw_fastrtps_cpp" and "RMW_IMPLEMENTATION" not in os.environ:
            rmw_source = "(default)"
        if localhost == "0" and "ROS_LOCALHOST_ONLY" not in os.environ:
            localhost_source = "(default)"

        print(f"    - ROS_DOMAIN_ID={domain_id} {domain_source}")
        print(f"    - RMW_IMPLEMENTATION={rmw} {rmw_source}")
        print(f"    - ROS_LOCALHOST_ONLY={localhost} {localhost_source}")

        if network_isolation:
            print("    - Network: Isolated (PrivateNetwork=yes)")

        # Print additional environment variables (not in captured env)
        additional_vars = {}
        for k, v in env_vars.items():
            if k not in ["ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY"]:
                # Only show if it wasn't in captured_env or was explicitly set
                if not captured_env or k not in captured_env or captured_env.get(k) != v:
                    additional_vars[k] = v

        if additional_vars:
            print("\n  ✓ Additional environment:")
            for key, value in additional_vars.items():
                # Determine source
                if key in os.environ and os.environ[key] == value:
                    source = "(copied from shell)"
                else:
                    source = "(specified)"
                # Truncate long values
                display_value = value if len(value) <= 50 else value[:47] + "..."
                print(f"    - {key}={display_value} {source}")

        print()
