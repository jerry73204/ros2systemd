import os
import sys

from ros2systemd.api.systemd_manager import SystemdServiceManager
from ros2systemd.verb import VerbExtension


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
        parser.add_argument("--no-color", action="store_true", help="Disable colored output")

    def main(self, *, args):
        manager = SystemdServiceManager(user_mode=not args.system)

        # Check if service exists
        status = manager.get_service_status(args.service_name)
        if not status["exists"]:
            print(f"Service 'ros2-{args.service_name}' does not exist")
            return 1

        # Build journalctl command arguments
        cmd_args = ["journalctl"]

        # Add user/system flag
        if not args.system:
            cmd_args.append("--user")

        # Add unit filter
        cmd_args.extend(["-u", f"ros2-{args.service_name}"])

        # Add line limit (unless following)
        if not args.follow:
            cmd_args.extend(["-n", str(args.lines)])

        # Add follow flag
        if args.follow:
            cmd_args.append("-f")
            # Print info message before exec
            if not args.no_color and sys.stdout.isatty():
                print(f"\033[1;34mFollowing logs for 'ros2-{args.service_name}' (Ctrl+C to stop)...\033[0m")
            else:
                print(f"Following logs for 'ros2-{args.service_name}' (Ctrl+C to stop)...")

        # Add time filters
        if args.since:
            cmd_args.extend(["--since", args.since])
        if args.until:
            cmd_args.extend(["--until", args.until])

        # Always add --no-pager to prevent paging and allow colors
        cmd_args.append("--no-pager")

        # Setup environment for color control
        if args.no_color:
            # Disable colors by setting SYSTEMD_COLORS=0
            os.environ["SYSTEMD_COLORS"] = "0"
        else:
            # Enable colors if terminal supports it (journalctl will auto-detect)
            # Remove any existing SYSTEMD_COLORS=0 to let journalctl decide
            if os.environ.get("SYSTEMD_COLORS") == "0":
                del os.environ["SYSTEMD_COLORS"]

        # Use exec to replace current process with journalctl
        # This allows journalctl to have full terminal control for colors, follow mode, etc.
        try:
            os.execvp("journalctl", cmd_args)
        except OSError as e:
            print(f"Error executing journalctl: {e}")
            return 1
