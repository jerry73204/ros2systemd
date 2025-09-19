import argparse
import time

from ros2systemd.api.systemd_manager import SystemdServiceManager
from ros2systemd.verb import VerbExtension


class RunVerb(VerbExtension):
    """Run a ROS2 node as a systemd service (create and start in one step).

    This command creates a systemd service for the specified node and starts it immediately.
    The service name is automatically generated based on the package and executable name.
    """

    def add_arguments(self, parser, cli_name):
        parser.epilog = """
Examples:
  # Run a simple node
  ros2 systemd run demo_nodes_cpp talker

  # Run node with arguments
  ros2 systemd run demo_nodes_cpp talker -- --ros-args -p frequency:=2.0

  # Run with custom environment
  ros2 systemd run --domain-id 42 demo_nodes_cpp talker

  # Run as system service
  ros2 systemd run --system demo_nodes_cpp talker

The service will be named automatically as 'ros2-{package}-{executable}-{timestamp}'.
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
            help="Custom service name (default: auto-generated from package-executable-timestamp)",
        )

        # Required positional arguments (matching ros2 run)
        parser.add_argument("package_name", help="Name of the ROS package")
        parser.add_argument("executable_name", help="Name of the executable")

        # Optional arguments to pass to the executable
        parser.add_argument(
            "argv",
            nargs=argparse.REMAINDER,
            help="Arguments to pass to the executable (use -- to separate from command options)",
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

    def _generate_service_name(self, package_name, executable_name):
        """Generate a unique service name based on package, executable, and timestamp."""
        timestamp = int(time.time())
        return f"{package_name}-{executable_name}-{timestamp}"

    def main(self, *, args):
        import os
        import sys
        from pathlib import Path

        # Handle '--' delimiter validation for argv
        extra_args = args.argv
        delimiter_used = "--" in sys.argv

        if extra_args:
            has_dash_args = any(arg.startswith("-") for arg in extra_args)
            if has_dash_args and not delimiter_used:
                print("Error: Arguments starting with '-' must be preceded by '--' delimiter.")
                print("Example: ros2 systemd run demo_nodes_cpp talker -- --ros-args -p param:=value")
                return 1

        # Generate service name if not provided
        service_name = args.name if args.name else self._generate_service_name(args.package_name, args.executable_name)

        manager = SystemdServiceManager(user_mode=not args.system)

        # Environment setup (simplified from create.py)
        env_mode = args.env_mode

        if env_mode == "all":
            print("⚠️  WARNING: --env-mode=all copies ALL environment variables!")
            print("   This may expose sensitive data. Use --env-mode=ros for safer operation.")
            print()

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
                    print(f"Warning: Source script not found: {script_path}")
                else:
                    source_scripts.append(str(resolved_path))

        # Network isolation warning
        if args.network_isolation and not args.system:
            print("⚠️  Warning: Network isolation requires root privileges and --system flag.")
            print()

        # Create the service
        print(f"Creating service 'ros2-{service_name}'...")
        success = manager.create_node_service(
            service_name=service_name,
            package=args.package_name,
            executable=args.executable_name,
            node_args=extra_args,
            env_vars=env_vars,
            source_scripts=source_scripts,
            description=args.description,
            network_isolation=args.network_isolation,
        )

        if not success:
            print(f"Failed to create service 'ros2-{service_name}'")
            return 1

        print(f"Service 'ros2-{service_name}' created successfully")

        # Start the service
        print(f"Starting service 'ros2-{service_name}'...")
        start_success, start_message = manager.start_service(service_name)

        if start_success:
            print(f"✓ Service 'ros2-{service_name}' is now running")
            print()
            print(f"Service name: ros2-{service_name}")
            print(f"  Stop with:   ros2 systemd stop {service_name}")
            print(f"  Status with: ros2 systemd status {service_name}")
            print(f"  Logs with:   ros2 systemd logs {service_name}")
            return 0
        else:
            print(f"✗ Failed to start service 'ros2-{service_name}'")
            print(f"Error: {start_message}")
            print(f"Service was created but not started. Use 'ros2 systemd start {service_name}' to retry.")
            return 1
