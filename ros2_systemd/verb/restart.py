from ros2_systemd.api.systemd_manager import SystemdServiceManager
from ros2_systemd.verb import VerbExtension


class RestartVerb(VerbExtension):
    """Restart a ROS2 systemd service."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("service_name", help="Name of the service to restart (without ros2- prefix)")
        parser.add_argument("--system", action="store_true", help="Restart system service instead of user service")

    def main(self, *, args):
        manager = SystemdServiceManager(user_mode=not args.system)
        success, message = manager.restart_service(args.service_name)

        if success:
            print(f"Successfully restarted service 'ros2-{args.service_name}'")
            return 0
        else:
            print(f"Failed to restart service 'ros2-{args.service_name}'")
            print(f"Error: {message}")
            return 1
