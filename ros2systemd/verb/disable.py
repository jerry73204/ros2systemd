from ros2systemd.api.systemd_manager import SystemdServiceManager
from ros2systemd.verb import VerbExtension


class DisableVerb(VerbExtension):
    """Disable a ROS2 systemd service from starting on boot."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("service_name", help="Name of the service to disable (without ros2- prefix)")
        parser.add_argument("--system", action="store_true", help="Disable system service instead of user service")
        parser.add_argument("--now", action="store_true", help="Also stop the service immediately")

    def main(self, *, args):
        manager = SystemdServiceManager(user_mode=not args.system)
        success, message = manager.disable_service(args.service_name)

        if success:
            print(f"Successfully disabled service 'ros2-{args.service_name}'")

            if args.now:
                success, message = manager.stop_service(args.service_name)
                if success:
                    print(f"Service 'ros2-{args.service_name}' stopped")
                else:
                    print(f"Warning: Service disabled but failed to stop: {message}")

            return 0
        else:
            print(f"Failed to disable service 'ros2-{args.service_name}'")
            print(f"Error: {message}")
            return 1
