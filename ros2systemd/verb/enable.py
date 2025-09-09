from ros2systemd.api.systemd_manager import SystemdServiceManager
from ros2systemd.verb import VerbExtension


class EnableVerb(VerbExtension):
    """Enable a ROS2 systemd service to start on boot."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("service_name", help="Name of the service to enable (without ros2- prefix)")
        parser.add_argument("--system", action="store_true", help="Enable system service instead of user service")
        parser.add_argument("--now", action="store_true", help="Also start the service immediately")

    def main(self, *, args):
        manager = SystemdServiceManager(user_mode=not args.system)
        success, message = manager.enable_service(args.service_name)

        if success:
            print(f"Successfully enabled service 'ros2-{args.service_name}'")

            if args.now:
                success, message = manager.start_service(args.service_name)
                if success:
                    print(f"Service 'ros2-{args.service_name}' started")
                else:
                    print(f"Warning: Service enabled but failed to start: {message}")
                    return 1

            return 0
        else:
            print(f"Failed to enable service 'ros2-{args.service_name}'")
            print(f"Error: {message}")
            return 1
