import argparse
import time

from ros2systemd.api.systemd_manager import SystemdServiceManager
from ros2systemd.verb import VerbExtension


class LaunchVerb(VerbExtension):
    """Launch a ROS2 launch file as a systemd service (create and start in one step).

    This command creates a systemd service for the specified launch file and starts it immediately.
    The service name is automatically generated based on the package and launch file name.
    """

    def add_arguments(self, parser, cli_name):
        parser.epilog = """
Examples:
  # Launch a file from a package
  ros2 systemd launch demo_nodes_cpp talker_listener.launch.py

  # Launch with arguments
  ros2 systemd launch demo_nodes_cpp talker_listener.launch.py use_sim_time:=true

  # Launch with custom environment
  ros2 systemd launch --domain-id 42 demo_nodes_cpp talker_listener.launch.py

  # Launch from full path
  ros2 systemd launch /path/to/launch/file.launch.py

  # Launch as system service
  ros2 systemd launch --system demo_nodes_cpp talker_listener.launch.py

  # Replace existing service with same name
  ros2 systemd launch --name my-launch --replace demo_nodes_cpp talker_listener.launch.py

The service will be named automatically as 'ros2-{package}-{launch_file}-{timestamp}'.
Use 'ros2 systemd list' to see all services and 'ros2 systemd stop <name>' to stop.
"""
        parser.formatter_class = argparse.RawDescriptionHelpFormatter

        # Environment options (subset of create command)
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
            help="Environment variable copying mode: 'ros' (ROS/DDS variables only, default), "
            "'all' (all variables), 'none' (explicit only)",
        )
        parser.add_argument(
            "--domain-id",
            type=int,
            metavar="ID",
            help="ROS domain ID (0-232), sets ROS_DOMAIN_ID",
        )
        parser.add_argument(
            "--rmw",
            metavar="IMPL",
            help="RMW implementation, e.g., rmw_fastrtps_cpp",
        )
        parser.add_argument(
            "--localhost-only",
            choices=["0", "1"],
            help="Set ROS_LOCALHOST_ONLY (0=disabled, 1=enabled)",
        )
        parser.add_argument(
            "--network-isolation",
            action="store_true",
            help="Enable network isolation - service runs in isolated network namespace",
        )
        parser.add_argument("--description", metavar="TEXT", help="Custom service description text")
        parser.add_argument(
            "--system",
            action="store_true",
            help="Create system-wide service instead of user service (requires sudo)",
        )
        parser.add_argument(
            "--name",
            metavar="SERVICE_NAME",
            help="Custom service name (default: auto-generated from package-launch_file-timestamp)",
        )
        parser.add_argument(
            "--replace",
            action="store_true",
            help="Stop and remove existing service with the same name before creating new one",
        )
        parser.add_argument(
            "--verbose",
            "-v",
            action="store_true",
            help="Show detailed output including environment configuration",
        )

        # Required positional arguments (matching ros2 launch)
        parser.add_argument("package_name", help="Name of the ROS package containing the launch file")
        parser.add_argument(
            "launch_file_name",
            nargs="?",
            help="Name of the launch file (optional if package_name is a full path)",
        )

        # Launch arguments
        parser.add_argument(
            "launch_arguments",
            nargs=argparse.REMAINDER,
            help="Arguments to the launch file in 'name:=value' format",
        )

    def _capture_environment(self, mode="ros"):
        """Capture environment variables based on the specified mode."""
        import os

        if mode == "none":
            return {}

        if mode == "all":
            special_vars = {"ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION"}
            return {k: v for k, v in os.environ.items() if k not in special_vars}

        # mode == "ros" - ROS/DDS-specific variables only
        captured_vars = {}

        core_keys = [
            "AMENT_PREFIX_PATH",
            "CMAKE_PREFIX_PATH",
            "LD_LIBRARY_PATH",
            "PATH",
            "PYTHONPATH",
            "PKG_CONFIG_PATH",
            "COLCON_PREFIX_PATH",
        ]

        dds_keys = [
            "CYCLONEDDS_URI",
            "CYCLONEDDS_NETWORK_INTERFACE",
            "CYCLONEDDS_PEER_ADDRESSES",
            "FASTRTPS_DEFAULT_PROFILES_FILE",
            "RMW_FASTRTPS_USE_QOS_FROM_XML",
            "RMW_FASTRTPS_PUBLICATION_MODE",
        ]

        logging_keys = [
            "RCUTILS_LOGGING_USE_STDOUT",
            "RCUTILS_LOGGING_BUFFERED_STREAM",
            "RCUTILS_COLORIZED_OUTPUT",
            "RCUTILS_CONSOLE_OUTPUT_FORMAT",
            "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED",
            "RCUTILS_LOGGING_SEPARATOR",
        ]

        special_vars = {"ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION"}

        for key, value in os.environ.items():
            if key in core_keys or key in dds_keys or key in logging_keys:
                captured_vars[key] = value
            elif key.startswith("ROS_") and key not in special_vars:
                captured_vars[key] = value
            elif key.startswith("RMW_") and key not in special_vars:
                captured_vars[key] = value
            elif key.startswith("CYCLONEDDS_"):
                captured_vars[key] = value
            elif key.startswith("FASTRTPS_"):
                captured_vars[key] = value
            elif key.startswith("RCUTILS_"):
                captured_vars[key] = value

        return captured_vars

    def _print_environment_info(self, env_vars, network_isolation, captured_env=None, source_scripts=None):
        """Print information about the environment variables set for the service."""
        import os

        print("\nEnvironment configuration:")

        if captured_env:
            print("  Captured ROS/Ament environment from current shell")
            if "AMENT_PREFIX_PATH" in captured_env:
                paths = captured_env["AMENT_PREFIX_PATH"].split(":")
                if len(paths) > 2:
                    print(f"    - AMENT_PREFIX_PATH: {paths[0]}:...:{paths[-1]} ({len(paths)} paths)")
                else:
                    print(f"    - AMENT_PREFIX_PATH: {captured_env['AMENT_PREFIX_PATH']}")
            if "ROS_DISTRO" in captured_env:
                print(f"    - ROS_DISTRO: {captured_env['ROS_DISTRO']}")

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

        if source_scripts:
            print("  Setup scripts to source:")
            for i, script in enumerate(source_scripts, 1):
                print(f"    {i}. {script}")

        print("\n  ROS Configuration:")
        domain_id = env_vars.get("ROS_DOMAIN_ID", "0")
        rmw = env_vars.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
        localhost = env_vars.get("ROS_LOCALHOST_ONLY", "0")

        domain_source = "(from shell)" if domain_id == os.environ.get("ROS_DOMAIN_ID") else "(specified)"
        rmw_source = "(from shell)" if rmw == os.environ.get("RMW_IMPLEMENTATION") else "(specified)"
        localhost_source = "(from shell)" if localhost == os.environ.get("ROS_LOCALHOST_ONLY") else "(specified)"

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

        additional_vars = {}
        for k, v in env_vars.items():
            if k not in ["ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY"]:
                if not captured_env or k not in captured_env or captured_env.get(k) != v:
                    additional_vars[k] = v

        if additional_vars:
            print("\n  Additional environment:")
            for key, value in additional_vars.items():
                if key in os.environ and os.environ[key] == value:
                    source = "(copied from shell)"
                else:
                    source = "(specified)"
                display_value = value if len(value) <= 50 else value[:47] + "..."
                print(f"    - {key}={display_value} {source}")

        print()

    def _generate_service_name(self, package_or_path, launch_file_name=None):
        """Generate a unique service name based on package/path, launch file, and timestamp."""
        from pathlib import Path

        timestamp = int(time.time())

        def _clean_launch_name(name):
            """Remove .launch.py or .launch.xml extensions from launch file name."""
            name = str(name)
            if name.endswith(".launch.py"):
                return name[:-10]  # Remove .launch.py
            elif name.endswith(".launch.xml"):
                return name[:-11]  # Remove .launch.xml
            elif name.endswith(".py"):
                return name[:-3]  # Remove .py
            return name

        if launch_file_name:
            # Package + launch file format
            package_name = Path(package_or_path).name
            launch_name = _clean_launch_name(launch_file_name)
            return f"{package_name}-{launch_name}-{timestamp}"
        else:
            # Full path format
            launch_path = Path(package_or_path)
            if launch_path.is_absolute():
                launch_name = _clean_launch_name(launch_path.name)
                return f"launch-{launch_name}-{timestamp}"
            else:
                # Treat as package name
                return f"{package_or_path}-launch-{timestamp}"

    def main(self, *, args):
        import os
        from pathlib import Path

        # Determine launch file path and generate service name
        if args.launch_file_name:
            # Package name + launch file format
            package_name = args.package_name
            launch_file = args.launch_file_name
            service_name = args.name if args.name else self._generate_service_name(package_name, launch_file)
        else:
            # Single argument - either full path or package name
            path_or_package = args.package_name

            if Path(path_or_package).exists():
                # It's a full path
                launch_file_path = path_or_package
                service_name = args.name if args.name else self._generate_service_name(path_or_package)
            else:
                # It's a package name - look for common launch files
                package_name = path_or_package
                launch_file = None
                service_name = args.name if args.name else self._generate_service_name(package_name)

        manager = SystemdServiceManager(user_mode=not args.system)

        # Environment setup (simplified from create.py)
        env_mode = args.env_mode

        if env_mode == "all" and args.verbose:
            print("WARNING: --env-mode=all copies ALL environment variables!")
            print("This may expose sensitive data. Use --env-mode=ros for safer operation.")

        env_vars = {}
        captured_env = self._capture_environment(env_mode)
        env_vars.update(captured_env)

        # Handle explicitly copied environment variables
        if args.copy_env:
            for key in args.copy_env:
                if key in os.environ:
                    env_vars[key] = os.environ[key]
                elif args.verbose:
                    print(f"Warning: Environment variable '{key}' not found in current shell")

        # Parse additional environment variables from --env
        if args.env:
            for env_var in args.env:
                if "=" in env_var:
                    key, value = env_var.split("=", 1)
                    env_vars[key] = value
                elif args.verbose:
                    print(f"Warning: Invalid environment variable format '{env_var}' (expected KEY=VALUE)")

        # Handle ROS environment variables
        if args.domain_id is not None:
            env_vars["ROS_DOMAIN_ID"] = str(args.domain_id)
        elif "ROS_DOMAIN_ID" not in env_vars:
            env_vars["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "0")

        if args.rmw:
            env_vars["RMW_IMPLEMENTATION"] = args.rmw
        elif "RMW_IMPLEMENTATION" not in env_vars:
            env_vars["RMW_IMPLEMENTATION"] = os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

        if args.localhost_only:
            env_vars["ROS_LOCALHOST_ONLY"] = args.localhost_only
        elif "ROS_LOCALHOST_ONLY" not in env_vars:
            env_vars["ROS_LOCALHOST_ONLY"] = os.environ.get("ROS_LOCALHOST_ONLY", "0")

        # Process source scripts
        source_scripts = []
        if args.source:
            for script_path in args.source:
                resolved_path = Path(script_path).expanduser().resolve()
                if not resolved_path.exists():
                    if args.verbose:
                        print(f"Warning: Source script not found: {script_path}")
                else:
                    source_scripts.append(str(resolved_path))

        # Network isolation warning
        if args.network_isolation and not args.system and args.verbose:
            print("Warning: Network isolation requires root privileges and --system flag.")

        # Handle service replacement if requested
        service_replaced = False
        if args.replace:
            status = manager.get_service_status(service_name)
            if status["exists"]:
                service_replaced = True
                # Stop the service if it's running
                if status["active"] in ["running", "active"]:
                    manager.stop_service(service_name)
                # Remove the existing service
                manager.remove_service(service_name)

        # Resolve launch file path
        if args.launch_file_name:
            # Package name + launch file format
            resolved_path = manager._resolve_package_path(package_name, launch_file)
            if resolved_path:
                launch_file_path = str(resolved_path)
            else:
                print(f"Error: Could not find launch file '{launch_file}' in package '{package_name}'")
                return 1
        else:
            # Handle single argument case
            path_or_package = args.package_name

            if Path(path_or_package).exists():
                # It's a full path
                launch_file_path = path_or_package
            else:
                # It's a package name - look for common launch files
                for common_name in ["launch.py", "default.launch.py", f"{path_or_package}.launch.py"]:
                    resolved_path = manager._resolve_package_path(path_or_package, common_name)
                    if resolved_path:
                        launch_file_path = str(resolved_path)
                        if args.verbose:
                            print(f"Found launch file: {launch_file_path}")
                        break
                else:
                    print(f"Error: '{path_or_package}' is not a valid file path or package with default launch file")
                    return 1

        # Parse launch arguments
        launch_args = {}
        if args.launch_arguments:
            for arg in args.launch_arguments:
                if ":=" in arg:
                    key, value = arg.split(":=", 1)
                    launch_args[key] = value
                elif args.verbose:
                    print(f"Warning: Ignoring invalid launch argument '{arg}' (expected KEY:=VALUE format)")

        # Create the service
        success = manager.create_launch_service(
            service_name=service_name,
            launch_file=launch_file_path,
            launch_args=launch_args,
            env_vars=env_vars,
            source_scripts=source_scripts,
            description=args.description,
            network_isolation=args.network_isolation,
        )

        if not success:
            print(f"Failed to create service 'ros2-{service_name}'")
            return 1

        # Start the service
        start_success, start_message = manager.start_service(service_name)

        if start_success:
            action = "Replaced" if service_replaced else "Started"
            print(f"{action} service 'ros2-{service_name}'")
            if args.verbose:
                self._print_environment_info(env_vars, args.network_isolation, captured_env, source_scripts)
            return 0
        else:
            print(f"Failed to start service 'ros2-{service_name}'")
            if args.verbose:
                print(f"Error: {start_message}")
                print(f"Service was created but not started. Use 'ros2 systemd start {service_name}' to retry.")
            return 1
