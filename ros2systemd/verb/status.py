import os

from ros2systemd.verb import VerbExtension


class StatusVerb(VerbExtension):
    """Show status of a ROS2 systemd service."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("service_name", help="Name of the service to check (without ros2- prefix)")
        parser.add_argument("--system", action="store_true", help="Check system service instead of user service")
        parser.add_argument("--no-color", action="store_true", help="Disable colored output")

    def main(self, *, args):
        # Build systemctl status command arguments
        cmd_args = ["systemctl"]
        if not args.system:
            cmd_args.append("--user")
        cmd_args.extend(["status", f"ros2-{args.service_name}", "--no-pager"])

        # Setup environment for color control
        if args.no_color:
            # Disable colors by setting SYSTEMD_COLORS=0
            os.environ["SYSTEMD_COLORS"] = "0"
        else:
            # Enable colors if terminal supports it (systemctl will auto-detect)
            # Remove any existing SYSTEMD_COLORS=0 to let systemctl decide
            if os.environ.get("SYSTEMD_COLORS") == "0":
                del os.environ["SYSTEMD_COLORS"]

        # Use exec to replace current process with systemctl
        # This allows systemctl to have full terminal control for colors, paging, etc.
        try:
            os.execvp("systemctl", cmd_args)
        except OSError as e:
            print(f"Error executing systemctl: {e}")
            return 1
