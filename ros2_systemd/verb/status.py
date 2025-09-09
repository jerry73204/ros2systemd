import subprocess

from ros2_systemd.verb import VerbExtension


class StatusVerb(VerbExtension):
    """Show status of a ROS2 systemd service."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("service_name", help="Name of the service to check (without ros2- prefix)")
        parser.add_argument("--system", action="store_true", help="Check system service instead of user service")

    def main(self, *, args):
        # Build systemctl status command
        cmd = ["systemctl"]
        if not args.system:
            cmd.append("--user")
        cmd.extend(["status", f"ros2-{args.service_name}", "--no-pager"])

        # Run and print output directly
        result = subprocess.run(cmd, capture_output=True, text=True)

        # Print the output regardless of return code
        # systemctl status returns non-zero for stopped services but still shows info
        print(result.stdout)
        if result.stderr:
            print(result.stderr)

        # Return 0 for successful command execution (not service status)
        return 0
