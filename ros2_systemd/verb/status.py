from ros2_systemd.verb import VerbExtension
from ros2_systemd.api.systemd_manager import SystemdServiceManager


class StatusVerb(VerbExtension):
    """Show status of a ROS2 systemd service."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'service_name',
            help='Name of the service to check (without ros2- prefix)')
        parser.add_argument(
            '--system',
            action='store_true',
            help='Check system service instead of user service')
        parser.add_argument(
            '--verbose',
            action='store_true',
            help='Show full systemctl status output')

    def main(self, *, args):
        manager = SystemdServiceManager(user_mode=not args.system)
        status_info = manager.get_service_status(args.service_name)
        
        if not status_info['exists']:
            print(f"Service 'ros2-{args.service_name}' does not exist")
            return 1
        
        print(f"Service: ros2-{args.service_name}")
        print(f"Status: {status_info['active']}")
        print(f"Enabled: {status_info['enabled']}")
        
        if args.verbose:
            print("\nFull status output:")
            print(status_info['output'])
        
        return 0