from ros2systemd.api.systemd_manager import SystemdServiceManager
from ros2systemd.verb import VerbExtension


class ListVerb(VerbExtension):
    """List all ROS2 systemd services."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("--system", action="store_true", help="List system services instead of user services")
        parser.add_argument("--all", action="store_true", help="List both user and system services")

    def main(self, *, args):
        if args.all:
            # List both user and system services
            print("=== User Services ===")
            user_manager = SystemdServiceManager(user_mode=True)
            user_services = user_manager.list_services()
            self._print_services(user_services)

            print("\n=== System Services ===")
            system_manager = SystemdServiceManager(user_mode=False)
            system_services = system_manager.list_services()
            self._print_services(system_services)
        else:
            # List only the specified type
            manager = SystemdServiceManager(user_mode=not args.system)
            services = manager.list_services()

            if not services:
                service_type = "system" if args.system else "user"
                print(f"No ROS2 {service_type} services found")
                return 0

            self._print_services(services)

        return 0

    def _print_services(self, services):
        """Print services in a formatted table."""
        if not services:
            print("No services found")
            return

        # Print header
        print(f"{'Service Name':<30} {'Status':<15} {'Active':<10}")
        print("-" * 55)

        # Print services
        for service in services:
            print(f"{service['name']:<30} {service['status']:<15} {service['active']:<10}")
