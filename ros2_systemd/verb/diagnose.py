import os
import subprocess

from ros2_systemd.api.systemd_manager import SystemdServiceManager
from ros2_systemd.verb import VerbExtension


class DiagnoseVerb(VerbExtension):
    """Diagnose potential issues with ROS2 systemd services."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "service_name",
            nargs="?",
            help="Name of the service to diagnose (without ros2- prefix). If not provided, diagnose all.",
        )
        parser.add_argument("--system", action="store_true", help="Diagnose system services instead of user services")

    def main(self, *, args):
        manager = SystemdServiceManager(user_mode=not args.system)

        # Get current daemon info
        daemon_info = self._get_daemon_info()

        print("=== ROS2 Daemon Status ===")
        if daemon_info:
            print("Daemon is running:")
            print(f"  RMW: {daemon_info['rmw']}")
            print(f"  Domain ID: {daemon_info['domain']}")
            print(f"  PID: {daemon_info['pid']}")
        else:
            print("No daemon running")
        print()

        # Get current shell environment
        print("=== Current Shell Environment ===")
        shell_rmw = os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp (default)")
        shell_domain = os.environ.get("ROS_DOMAIN_ID", "0 (default)")
        print(f"  RMW: {shell_rmw}")
        print(f"  Domain ID: {shell_domain}")
        print()

        # Check services
        if args.service_name:
            # Check specific service
            self._diagnose_service(manager, args.service_name, daemon_info)
        else:
            # Check all services
            services = manager.list_services()
            if not services:
                print("No ros2-systemd services found")
                return 0

            print("=== Service Diagnostics ===")
            for service in services:
                self._diagnose_service(manager, service["name"], daemon_info)
                print()

        # Provide recommendations
        self._print_recommendations(daemon_info)

        return 0

    def _get_daemon_info(self):
        """Get information about the running daemon."""
        try:
            # Find daemon process
            result = subprocess.run(["ps", "aux"], capture_output=True, text=True)

            for line in result.stdout.splitlines():
                if "ros2cli.daemon.daemonize" in line and "grep" not in line:
                    parts = line.split()
                    pid = parts[1]

                    # Parse command line for RMW and domain
                    rmw = "unknown"
                    domain = "unknown"

                    if "--rmw-implementation" in line:
                        idx = line.index("--rmw-implementation")
                        rmw = line[idx:].split()[1]
                        rmw = rmw.replace("rmw_", "").replace("_cpp", "")

                    if "--ros-domain-id" in line:
                        idx = line.index("--ros-domain-id")
                        domain = line[idx:].split()[1]

                    return {"pid": pid, "rmw": rmw, "domain": domain, "full_line": line}
        except Exception:  # noqa: B902
            pass

        return None

    def _diagnose_service(self, manager, service_name, daemon_info):
        """Diagnose a specific service."""
        print(f"Service: ros2-{service_name}")

        # Get service file content
        service_file = manager.service_dir / f"ros2-{service_name}.service"
        if not service_file.exists():
            print("  ERROR: Service file not found")
            return

        content = service_file.read_text()

        # Parse environment from service
        service_rmw = "fastrtps"  # default
        service_domain = "0"  # default
        is_isolated = False

        for line in content.splitlines():
            if "RMW_IMPLEMENTATION" in line:
                service_rmw = line.split("=")[-1].strip('"')
                service_rmw = service_rmw.replace("rmw_", "").replace("_cpp", "")
            if "ROS_DOMAIN_ID" in line:
                service_domain = line.split("=")[-1].strip('"')
            if "PrivateNetwork=yes" in line:
                is_isolated = True

        # Get status
        status_cmd = ["systemctl"]
        if not manager.user_mode:
            status_cmd.append("--system")
        else:
            status_cmd.append("--user")
        status_cmd.extend(["is-active", f"ros2-{service_name}"])

        result = subprocess.run(status_cmd, capture_output=True, text=True)
        is_active = result.stdout.strip() == "active"

        # Print diagnostics
        print(f"  Status: {result.stdout.strip()}")
        print(f"  RMW: {service_rmw}")
        print(f"  Domain ID: {service_domain}")
        print(f"  Network: {'Isolated' if is_isolated else 'Public'}")

        # Check for issues
        issues = []

        if is_active and not is_isolated and daemon_info:
            if service_rmw != daemon_info["rmw"]:
                issues.append(f"RMW mismatch: service uses {service_rmw}, daemon uses {daemon_info['rmw']}")
            if service_domain != daemon_info["domain"]:
                issues.append(f"Domain mismatch: service uses {service_domain}, daemon uses {daemon_info['domain']}")

        if is_isolated:
            print("  Note: Service runs in isolated network (no daemon interaction)")

        if issues:
            print("  ⚠️  Potential Issues:")
            for issue in issues:
                print(f"    - {issue}")
            print("    → Service may not be discoverable via 'ros2 topic list'")
            print("    → Use 'ros2 daemon stop' and restart with matching environment")

    def _print_recommendations(self, daemon_info):
        """Print recommendations based on diagnostics."""
        print("\n=== Recommendations ===")

        if daemon_info:
            print("1. To ensure service discovery, stop and restart daemon with matching environment:")
            print("   ros2 daemon stop")
            print("   export RMW_IMPLEMENTATION=<your_rmw>")
            print("   export ROS_DOMAIN_ID=<your_domain>")
            print("   ros2 topic list  # This will start daemon with new settings")
        else:
            print("1. No daemon is currently running. It will start with your next ros2 command.")

        print("\n2. For isolated services, use network isolation (--network-isolation flag)")
        print("   These services won't interfere with the daemon.")

        print("\n3. To check service logs directly (bypasses daemon):")
        print("   ros2 systemd logs <service_name>")

        print("\n4. Service descriptions now show environment in 'systemctl status':")
        print("   Look for [RMW=xxx, Domain=N] in the Description field")
