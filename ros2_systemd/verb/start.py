from ros2_systemd.verb import VerbExtension
from ros2_systemd.api.systemd_manager import SystemdServiceManager


class StartVerb(VerbExtension):
    """Start a ROS2 systemd service."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'service_name',
            help='Name of the service to start (without ros2- prefix)')
        parser.add_argument(
            '--system',
            action='store_true',
            help='Start system service instead of user service')

    def main(self, *, args):
        manager = SystemdServiceManager(user_mode=not args.system)
        success, message = manager.start_service(args.service_name)
        
        if success:
            print(f"Successfully started service 'ros2-{args.service_name}'")
            print(f"Use 'ros2 systemd status {args.service_name}' to check status")
            return 0
        else:
            print(f"Failed to start service 'ros2-{args.service_name}'")
            print(f"Error: {message}")
            return 1