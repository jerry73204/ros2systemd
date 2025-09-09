from ros2systemd.api.systemd_manager import SystemdServiceManager
from ros2systemd.verb import VerbExtension


class RemoveVerb(VerbExtension):
    """Remove a ROS2 systemd service."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("service_name", help="Name of the service to remove (without ros2- prefix)")
        parser.add_argument("--system", action="store_true", help="Remove system service instead of user service")

    def main(self, *, args):
        manager = SystemdServiceManager(user_mode=not args.system)
        success, message = manager.remove_service(args.service_name)

        if success:
            print(message)
            return 0
        else:
            print(f"Failed to remove service: {message}")
            return 1
