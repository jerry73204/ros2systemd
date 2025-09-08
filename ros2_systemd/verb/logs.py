import subprocess

from ros2_systemd.api.systemd_manager import SystemdServiceManager
from ros2_systemd.verb import VerbExtension


class LogsVerb(VerbExtension):
    """View logs for a ROS2 systemd service."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("service_name", help="Name of the service (without ros2- prefix)")
        parser.add_argument(
            "--system", action="store_true", help="View logs for system service instead of user service"
        )
        parser.add_argument("-n", "--lines", type=int, default=50, help="Number of log lines to show (default: 50)")
        parser.add_argument("-f", "--follow", action="store_true", help="Follow log output (like tail -f)")
        parser.add_argument("--since", help='Show logs since time (e.g., "1 hour ago", "2023-01-01")')
        parser.add_argument("--until", help="Show logs until time")

    def main(self, *, args):
        manager = SystemdServiceManager(user_mode=not args.system)

        # Check if service exists
        status = manager.get_service_status(args.service_name)
        if not status["exists"]:
            print(f"Service 'ros2-{args.service_name}' does not exist")
            return 1

        # Build journalctl command
        cmd = ["journalctl"]

        # Add user/system flag
        if not args.system:
            cmd.append("--user")

        # Add unit filter
        cmd.extend(["-u", f"ros2-{args.service_name}"])

        # Add line limit (unless following)
        if not args.follow:
            cmd.extend(["-n", str(args.lines)])

        # Add follow flag
        if args.follow:
            cmd.append("-f")
            print(f"Following logs for 'ros2-{args.service_name}' (Ctrl+C to stop)...")

        # Add time filters
        if args.since:
            cmd.extend(["--since", args.since])
        if args.until:
            cmd.extend(["--until", args.until])

        # Add color output if terminal supports it
        cmd.append("--no-pager")

        try:
            # Run journalctl
            result = subprocess.run(cmd, text=True)
            return result.returncode
        except KeyboardInterrupt:
            print("\nStopped following logs")
            return 0
        except Exception as e:  # noqa: B902
            print(f"Error viewing logs: {e}")
            return 1
