from ros2_systemd.api.systemd_manager import SystemdServiceManager
from ros2_systemd.verb import VerbExtension


class RemoveVerb(VerbExtension):
    """Remove a ROS2 systemd service."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("service_name", help="Name of the service to remove (without ros2- prefix)")
        parser.add_argument("--system", action="store_true", help="Remove system service instead of user service")
        parser.add_argument("--force", action="store_true", help="Force removal without confirmation")

    def main(self, *, args):
        if not args.force:
            # Ask for confirmation
            response = input(f"Are you sure you want to remove service 'ros2-{args.service_name}'? [y/N]: ")
            if response.lower() != "y":
                print("Removal cancelled")
                return 0

        manager = SystemdServiceManager(user_mode=not args.system)
        success, message = manager.remove_service(args.service_name)

        if success:
            print(message)
            return 0
        else:
            print(f"Failed to remove service: {message}")
            return 1
