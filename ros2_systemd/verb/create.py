from ros2_systemd.api.systemd_manager import SystemdServiceManager
from ros2_systemd.verb import VerbExtension


class CreateVerb(VerbExtension):
    """Create a systemd service for a ROS2 launch file or node."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("service_name", help="Name for the systemd service (will be prefixed with ros2-)")

        # Create subparsers for launch and node
        subparsers = parser.add_subparsers(dest="service_type", help="Type of service to create")

        # Launch file parser
        launch_parser = subparsers.add_parser("launch", help="Create service for launch file")
        launch_parser.add_argument("package_or_path", help="Package name or path to launch file")
        launch_parser.add_argument("launch_file", nargs="?", help="Launch file name (when package name is provided)")
        launch_parser.add_argument("--launch-args", nargs="*", help="Launch arguments in key:=value format")

        # Node parser
        node_parser = subparsers.add_parser("node", help="Create service for ROS2 node")
        node_parser.add_argument("package", help="ROS2 package name")
        node_parser.add_argument("executable", help="Executable name in the package")
        node_parser.add_argument("--node-args", nargs="*", help="Additional node arguments")

        # Common arguments
        for p in [launch_parser, node_parser]:
            p.add_argument("--env", nargs="*", help="Environment variables in KEY=VALUE format")
            p.add_argument("--description", help="Service description")
            p.add_argument(
                "--system", action="store_true", help="Create system service instead of user service (requires sudo)"
            )

    def main(self, *, args):
        manager = SystemdServiceManager(user_mode=not args.system)

        # Parse environment variables
        env_vars = {}
        if args.env:
            for env_var in args.env:
                if "=" in env_var:
                    key, value = env_var.split("=", 1)
                    env_vars[key] = value

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

            # Create launch service
            success = manager.create_launch_service(
                service_name=args.service_name,
                launch_file=launch_file_path,
                launch_args=launch_args,
                env_vars=env_vars,
                description=args.description,
            )

            if success:
                print(f"Successfully created service 'ros2-{args.service_name}' for launch file '{launch_file_path}'")
                print(f"Use 'ros2 systemd start {args.service_name}' to start the service")
                return 0
            else:
                print(f"Failed to create service 'ros2-{args.service_name}'")
                return 1

        elif args.service_type == "node":
            # Create node service
            success = manager.create_node_service(
                service_name=args.service_name,
                package=args.package,
                executable=args.executable,
                node_args=args.node_args,
                env_vars=env_vars,
                description=args.description,
            )

            if success:
                print(
                    f"Successfully created service 'ros2-{args.service_name}' "
                    f"for node '{args.package}/{args.executable}'"
                )
                print(f"Use 'ros2 systemd start {args.service_name}' to start the service")
                return 0
            else:
                print(f"Failed to create service 'ros2-{args.service_name}'")
                return 1
        else:
            print("Please specify 'launch' or 'node' as the service type")
            return 1
