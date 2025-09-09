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

        # Create service for a node
        ros2 systemd create my-talker node demo_nodes_cpp talker

        # With environment options
        ros2 systemd create my-service node pkg exe \
            --domain-id 42 \
            --rmw rmw_cyclonedds_cpp \
            --env CUSTOM_VAR=value

        # With network isolation (no daemon interaction)
        ros2 systemd create isolated-node node pkg exe --network-isolation
    """

    def add_arguments(self, parser, cli_name):
        # Add examples to the parser
        parser.epilog = """
Examples:
  # Create service for a launch file with full path
  ros2 systemd create my-robot launch /path/to/robot.launch.py

  # Create service for a launch file from package
  ros2 systemd create talker-listener launch demo_nodes_cpp talker_listener.launch.py

  # Create service for a node
  ros2 systemd create my-talker node demo_nodes_cpp talker

  # With environment options
  ros2 systemd create my-service node pkg exe \\
      --domain-id 42 \\
      --rmw rmw_cyclonedds_cpp \\
      --env CUSTOM_VAR=value

  # With network isolation (no daemon interaction)
  ros2 systemd create isolated-node node pkg exe --network-isolation

  # Create system-wide service (requires sudo)
  ros2 systemd create my-system-service node pkg exe --system

Environment Variables:
  --domain-id N     Sets ROS_DOMAIN_ID=N (valid range: 0-232)
  --rmw IMPL        Sets RMW_IMPLEMENTATION=IMPL
  --env KEY=VALUE   Sets custom environment variables

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
        parser.add_argument("service_name", help="Name for the systemd service (will be prefixed with ros2-)")

        # Create subparsers for launch and node
        subparsers = parser.add_subparsers(dest="service_type", help="Type of service to create")

        # Launch file parser
        launch_parser = subparsers.add_parser("launch", help="Create service for ROS2 launch file")
        launch_parser.add_argument(
            "package_or_path", help="Package name (e.g., 'turtlesim') or full path to launch file"
        )
        launch_parser.add_argument(
            "launch_file", nargs="?", help="Launch file name (e.g., 'multisim.launch.py') when package name is provided"
        )
        launch_parser.add_argument(
            "--launch-args", nargs="*", help="Launch arguments in key:=value format (e.g., 'use_sim_time:=true')"
        )

        # Node parser
        node_parser = subparsers.add_parser("node", help="Create service for ROS2 node")
        node_parser.add_argument("package", help="ROS2 package name (e.g., 'demo_nodes_cpp')")
        node_parser.add_argument("executable", help="Executable name in the package (e.g., 'talker')")
        node_parser.add_argument("--node-args", nargs="*", help="Additional node arguments")

        # Common arguments
        for p in [launch_parser, node_parser]:
            p.add_argument(
                "--env", nargs="*", help="Additional environment variables in KEY=VALUE format (e.g., 'FOO=bar')"
            )
            p.add_argument(
                "--domain-id",
                type=int,
                metavar="ID",
                help="ROS domain ID (0-232), sets ROS_DOMAIN_ID (uses current shell value if not specified)",
            )
            p.add_argument(
                "--rmw",
                metavar="IMPL",
                help="RMW implementation, e.g., rmw_fastrtps_cpp (uses current shell value if not specified)",
            )
            p.add_argument(
                "--localhost-only",
                choices=["0", "1"],
                help="Set ROS_LOCALHOST_ONLY (0=disabled, 1=enabled, uses current shell value if not specified)",
            )
            p.add_argument(
                "--network-isolation",
                action="store_true",
                help="Enable network isolation - service runs in isolated network namespace (no daemon interaction)",
            )
            p.add_argument("--description", metavar="TEXT", help="Custom service description text")
            p.add_argument(
                "--system",
                action="store_true",
                help="Create system-wide service instead of user service (requires sudo)",
            )

    def main(self, *, args):
        import os

        manager = SystemdServiceManager(user_mode=not args.system)

        # Parse environment variables
        env_vars = {}
        if args.env:
            for env_var in args.env:
                if "=" in env_var:
                    key, value = env_var.split("=", 1)
                    env_vars[key] = value

        # Handle ROS_DOMAIN_ID - use shell value if not specified
        if hasattr(args, "domain_id") and args.domain_id is not None:
            env_vars["ROS_DOMAIN_ID"] = str(args.domain_id)
        elif "ROS_DOMAIN_ID" not in env_vars:
            # Copy from shell environment if available, otherwise use default
            env_vars["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "0")

        # Handle RMW_IMPLEMENTATION - use shell value if not specified
        if hasattr(args, "rmw") and args.rmw:
            env_vars["RMW_IMPLEMENTATION"] = args.rmw
        elif "RMW_IMPLEMENTATION" not in env_vars:
            # Copy from shell environment if available, otherwise use default
            env_vars["RMW_IMPLEMENTATION"] = os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
        # Handle ROS_LOCALHOST_ONLY - use shell value if not specified
        if hasattr(args, "localhost_only") and args.localhost_only:
            env_vars["ROS_LOCALHOST_ONLY"] = args.localhost_only
        elif "ROS_LOCALHOST_ONLY" not in env_vars:
            # Copy from shell environment if available, otherwise use default
            env_vars["ROS_LOCALHOST_ONLY"] = os.environ.get("ROS_LOCALHOST_ONLY", "0")

        if args.service_type == "launch":
            # Determine launch file path
            if args.launch_file:
                # Package name + launch file format
                resolved_path = manager._resolve_package_path(args.package_or_path, args.launch_file)
                if resolved_path:
                    launch_file_path = str(resolved_path)
                else:
                    print(f"Error: Could not find launch file '{args.launch_file}' in package '{args.package_or_path}'")
                    return 1
            else:
                # Direct path format or check if it's a package name with common launch file
                from pathlib import Path

                if Path(args.package_or_path).exists():
                    launch_file_path = args.package_or_path
                else:
                    # Check if it's just a package name - look for common launch files
                    for common_name in ["launch.py", "default.launch.py", f"{args.package_or_path}.launch.py"]:
                        resolved_path = manager._resolve_package_path(args.package_or_path, common_name)
                        if resolved_path:
                            launch_file_path = str(resolved_path)
                            print(f"Found launch file: {launch_file_path}")
                            break
                    else:
                        print(f"Error: '{args.package_or_path}' is not a valid file path or package")
                        return 1

            # Parse launch arguments
            launch_args = {}
            if args.launch_args:
                for arg in args.launch_args:
                    if ":=" in arg:
                        key, value = arg.split(":=", 1)
                        launch_args[key] = value

            # Warn about network isolation limitations
            network_isolation = args.network_isolation if hasattr(args, "network_isolation") else False
            if network_isolation and not args.system:
                print("⚠️  Warning: Network isolation (PrivateNetwork=yes) requires root privileges.")
                print("   It will NOT work with user services. Consider using --system flag with sudo,")
                print("   or use ROS_LOCALHOST_ONLY=1 / different ROS_DOMAIN_ID for isolation.")
                print()

            # Create launch service
            success = manager.create_launch_service(
                service_name=args.service_name,
                launch_file=launch_file_path,
                launch_args=launch_args,
                env_vars=env_vars,
                description=args.description,
                network_isolation=network_isolation,
            )

            if success:
                print(f"Successfully created service 'ros2-{args.service_name}' for launch file '{launch_file_path}'")
                self._print_environment_info(env_vars, network_isolation)
                print(f"Use 'ros2 systemd start {args.service_name}' to start the service")
                return 0
            else:
                print(f"Failed to create service 'ros2-{args.service_name}'")
                return 1

        elif args.service_type == "node":
            # Warn about network isolation limitations
            network_isolation = args.network_isolation if hasattr(args, "network_isolation") else False
            if network_isolation and not args.system:
                print("⚠️  Warning: Network isolation (PrivateNetwork=yes) requires root privileges.")
                print("   It will NOT work with user services. Consider using --system flag with sudo,")
                print("   or use ROS_LOCALHOST_ONLY=1 / different ROS_DOMAIN_ID for isolation.")
                print()

            # Create node service
            success = manager.create_node_service(
                service_name=args.service_name,
                package=args.package,
                executable=args.executable,
                node_args=args.node_args,
                env_vars=env_vars,
                description=args.description,
                network_isolation=network_isolation,
            )

            if success:
                print(
                    f"Successfully created service 'ros2-{args.service_name}' "
                    f"for node '{args.package}/{args.executable}'"
                )
                self._print_environment_info(env_vars, network_isolation)
                print(f"Use 'ros2 systemd start {args.service_name}' to start the service")
                return 0
            else:
                print(f"Failed to create service 'ros2-{args.service_name}'")
                return 1
        else:
            print("Please specify 'launch' or 'node' as the service type")
            return 1

    def _print_environment_info(self, env_vars, network_isolation):
        """Print information about the environment variables set for the service."""
        import os

        print("\nService environment:")

        # Print ROS environment variables
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

        print(f"  ROS_DOMAIN_ID={domain_id} {domain_source}")
        print(f"  RMW_IMPLEMENTATION={rmw} {rmw_source}")
        print(f"  ROS_LOCALHOST_ONLY={localhost} {localhost_source}")

        if network_isolation:
            print("  Network: Isolated (PrivateNetwork=yes)")

        # Print custom environment variables
        custom_vars = {
            k: v for k, v in env_vars.items() if k not in ["ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY"]
        }
        if custom_vars:
            print("  Custom variables:")
            for key, value in custom_vars.items():
                print(f"    {key}={value}")
        print()
