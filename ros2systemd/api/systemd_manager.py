import os
import subprocess
from pathlib import Path
from typing import Dict, List, Optional, Tuple


class SystemdServiceManager:
    """Manager for creating and managing systemd services for ROS2 nodes and launches."""

    SERVICE_PREFIX = "ros2-"
    SYSTEMD_USER_DIR = Path.home() / ".config" / "systemd" / "user"
    SYSTEMD_SYSTEM_DIR = Path("/etc/systemd/system")

    def __init__(self, user_mode: bool = True):
        """
        Initialize the SystemdServiceManager.

        Args:
            user_mode: If True, manages user services. If False, manages system services.
        """
        self.user_mode = user_mode
        self.service_dir = self.SYSTEMD_USER_DIR if user_mode else self.SYSTEMD_SYSTEM_DIR

        if user_mode and not self.service_dir.exists():
            self.service_dir.mkdir(parents=True, exist_ok=True)

    def _resolve_package_path(self, package_name: str, file_name: str) -> Optional[Path]:
        """
        Resolve a package name and file to its full path.

        Args:
            package_name: ROS2 package name
            file_name: File name within the package

        Returns:
            Full path to the file, or None if not found
        """
        try:
            # Try to import ament_index_python if available
            from ament_index_python.packages import get_package_share_directory

            package_dir = get_package_share_directory(package_name)

            # Search for the file in common locations
            search_paths = [
                Path(package_dir) / file_name,
                Path(package_dir) / "launch" / file_name,
                Path(package_dir) / "launch" / "topics" / file_name,
                Path(package_dir) / "launch" / "services" / file_name,
            ]

            for path in search_paths:
                if path.exists():
                    return path

        except ImportError:
            # Fallback: try to find in /opt/ros/humble/share
            ros_distro = os.environ.get("ROS_DISTRO", "humble")
            package_dir = Path(f"/opt/ros/{ros_distro}/share") / package_name
            if package_dir.exists():
                search_paths = [
                    package_dir / file_name,
                    package_dir / "launch" / file_name,
                    package_dir / "launch" / "topics" / file_name,
                    package_dir / "launch" / "services" / file_name,
                ]

                for path in search_paths:
                    if path.exists():
                        return path

        return None

    def _get_systemctl_cmd(self, command: List[str]) -> List[str]:
        """Get the appropriate systemctl command based on mode."""
        base_cmd = ["systemctl"]
        if self.user_mode:
            base_cmd.append("--user")
        return base_cmd + command

    def _run_systemctl(self, command: List[str]) -> Tuple[int, str, str]:
        """Run a systemctl command and return the result."""
        cmd = self._get_systemctl_cmd(command)
        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.returncode, result.stdout, result.stderr

    def _validate_service_name(self, service_name: str) -> Tuple[bool, str]:
        """
        Validate service name.

        Args:
            service_name: Name to validate

        Returns:
            Tuple of (is_valid, error_message)
        """
        if not service_name:
            return False, "Service name cannot be empty"

        if len(service_name) > 200:
            return False, "Service name too long (max 200 characters)"

        # Check for invalid characters
        import re

        if not re.match(r"^[a-zA-Z0-9][a-zA-Z0-9-_.]*$", service_name):
            return (
                False,
                "Service name must start with alphanumeric and contain only letters, "
                "numbers, hyphens, underscores, and dots",
            )

        # Check for reserved names
        reserved_names = ["all", "help", "list", "status"]
        if service_name.lower() in reserved_names:
            return False, f"'{service_name}' is a reserved name"

        return True, ""

    def create_launch_service(
        self,
        service_name: str,
        launch_file: str,
        launch_args: Optional[Dict[str, str]] = None,
        env_vars: Optional[Dict[str, str]] = None,
        source_scripts: Optional[List[str]] = None,
        description: Optional[str] = None,
        network_isolation: bool = False,
    ) -> bool:
        """
        Create a systemd service for a ROS2 launch file.

        Args:
            service_name: Name of the service (will be prefixed with 'ros2-')
            launch_file: Path to the launch file
            launch_args: Optional launch arguments
            env_vars: Optional environment variables
            description: Optional service description

        Returns:
            True if service was created successfully
        """
        # Validate service name
        valid, error_msg = self._validate_service_name(service_name)
        if not valid:
            print(f"Error: Invalid service name - {error_msg}")
            return False

        # Check if launch file exists
        if not Path(launch_file).exists():
            print(f"Error: Launch file not found: {launch_file}")
            return False

        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"
        service_file = self.service_dir / f"{full_service_name}.service"

        # Check if service already exists
        if service_file.exists():
            print(f"Error: Service '{full_service_name}' already exists. Remove it first or choose a different name.")
            return False

        # Build the launch command
        launch_cmd = ["ros2", "launch", launch_file]
        if launch_args:
            for key, value in launch_args.items():
                launch_cmd.append(f"{key}:={value}")

        # Create service content
        service_content = self._generate_service_content(
            description=description or f"ROS2 launch service for {launch_file}",
            exec_command=" ".join(launch_cmd),
            env_vars=env_vars,
            source_scripts=source_scripts,
            network_isolation=network_isolation,
        )

        # Write service file
        service_file.write_text(service_content)

        # Reload systemd daemon
        self._run_systemctl(["daemon-reload"])

        return service_file.exists()

    def create_node_service(
        self,
        service_name: str,
        package: str,
        executable: str,
        node_args: Optional[List[str]] = None,
        env_vars: Optional[Dict[str, str]] = None,
        source_scripts: Optional[List[str]] = None,
        description: Optional[str] = None,
        network_isolation: bool = False,
    ) -> bool:
        """
        Create a systemd service for a ROS2 node.

        Args:
            service_name: Name of the service (will be prefixed with 'ros2-')
            package: ROS2 package name
            executable: Executable name in the package
            node_args: Optional node arguments
            env_vars: Optional environment variables
            description: Optional service description

        Returns:
            True if service was created successfully
        """
        # Validate service name
        valid, error_msg = self._validate_service_name(service_name)
        if not valid:
            print(f"Error: Invalid service name - {error_msg}")
            return False

        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"
        service_file = self.service_dir / f"{full_service_name}.service"

        # Check if service already exists
        if service_file.exists():
            print(f"Error: Service '{full_service_name}' already exists. Remove it first or choose a different name.")
            return False

        # Build the node command
        node_cmd = ["ros2", "run", package, executable]
        if node_args:
            node_cmd.extend(node_args)

        # Create service content
        service_content = self._generate_service_content(
            description=description or f"ROS2 node service for {package}/{executable}",
            exec_command=" ".join(node_cmd),
            env_vars=env_vars,
            source_scripts=source_scripts,
            network_isolation=network_isolation,
        )

        # Write service file
        service_file.write_text(service_content)

        # Reload systemd daemon
        self._run_systemctl(["daemon-reload"])

        return service_file.exists()

    def _generate_service_content(
        self,
        description: str,
        exec_command: str,
        env_vars: Optional[Dict[str, str]] = None,
        source_scripts: Optional[List[str]] = None,
        network_isolation: bool = False,
    ) -> str:
        """Generate systemd service file content."""
        # Build enhanced description with environment info
        rmw = env_vars.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp") if env_vars else "rmw_fastrtps_cpp"
        domain = env_vars.get("ROS_DOMAIN_ID", "0") if env_vars else "0"

        # Add environment info to description
        env_info = f" [RMW={rmw.replace('rmw_', '').replace('_cpp', '')}, Domain={domain}"
        if network_isolation:
            env_info += ", Isolated"
        env_info += "]"
        enhanced_description = description + env_info

        # Build the ExecStart command
        if source_scripts:
            # Source all scripts in order
            source_cmds = " && ".join([f"source {script}" for script in source_scripts])
            exec_start = f"/bin/bash -c '{source_cmds} && {exec_command}'"
        else:
            # No source scripts provided - run command directly
            exec_start = f"/bin/bash -c '{exec_command}'"

        service_lines = [
            "[Unit]",
            f"Description={enhanced_description}",
            "After=network.target",
            "",
            "[Service]",
            "Type=simple",
            f"ExecStart={exec_start}",
            "Restart=on-failure",
            "RestartSec=5",
            "StandardOutput=journal",
            "StandardError=journal",
        ]

        # Add network isolation if requested
        if network_isolation:
            service_lines.append("PrivateNetwork=yes")

        # Add environment variables
        if env_vars:
            for key, value in env_vars.items():
                service_lines.append(f'Environment="{key}={value}"')

        # Add ROS2 specific environment (only if not already set by user)
        if not env_vars or "ROS_DOMAIN_ID" not in env_vars:
            service_lines.append('Environment="ROS_DOMAIN_ID=0"')
        if not env_vars or "ROS_LOCALHOST_ONLY" not in env_vars:
            service_lines.append('Environment="ROS_LOCALHOST_ONLY=0"')

        service_lines.extend(
            [
                "",
                "[Install]",
                "WantedBy=default.target" if self.user_mode else "WantedBy=multi-user.target",
            ]
        )

        return "\n".join(service_lines)

    def start_service(self, service_name: str) -> Tuple[bool, str]:
        """Start a systemd service."""
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"

        # Check if service exists first
        status = self.get_service_status(service_name)
        if not status["exists"]:
            return False, f"Service '{full_service_name}' does not exist. Create it first with 'ros2 systemd create'."

        returncode, stdout, stderr = self._run_systemctl(["start", full_service_name])
        if returncode != 0:
            error_msg = stderr.strip() if stderr else "Unknown error"
            return False, f"Failed to start service: {error_msg}"
        return True, f"Service '{full_service_name}' started successfully"

    def stop_service(self, service_name: str) -> Tuple[bool, str]:
        """Stop a systemd service."""
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"

        # Check if service exists first
        status = self.get_service_status(service_name)
        if not status["exists"]:
            return False, f"Service '{full_service_name}' does not exist."

        returncode, stdout, stderr = self._run_systemctl(["stop", full_service_name])
        if returncode != 0:
            error_msg = stderr.strip() if stderr else "Unknown error"
            return False, f"Failed to stop service: {error_msg}"
        return True, f"Service '{full_service_name}' stopped successfully"

    def restart_service(self, service_name: str) -> Tuple[bool, str]:
        """Restart a systemd service."""
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"

        # Check if service exists first
        status = self.get_service_status(service_name)
        if not status["exists"]:
            return False, f"Service '{full_service_name}' does not exist. Create it first with 'ros2 systemd create'."

        returncode, stdout, stderr = self._run_systemctl(["restart", full_service_name])
        if returncode != 0:
            error_msg = stderr.strip() if stderr else "Unknown error"
            return False, f"Failed to restart service: {error_msg}"
        return True, f"Service '{full_service_name}' restarted successfully"

    def enable_service(self, service_name: str) -> Tuple[bool, str]:
        """Enable a systemd service to start on boot."""
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"

        # Check if service exists first
        status = self.get_service_status(service_name)
        if not status["exists"]:
            return False, f"Service '{full_service_name}' does not exist. Create it first with 'ros2 systemd create'."

        returncode, stdout, stderr = self._run_systemctl(["enable", full_service_name])
        if returncode != 0:
            error_msg = stderr.strip() if stderr else "Unknown error"
            return False, f"Failed to enable service: {error_msg}"
        return True, f"Service '{full_service_name}' enabled to start on boot"

    def disable_service(self, service_name: str) -> Tuple[bool, str]:
        """Disable a systemd service from starting on boot."""
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"

        # Check if service exists first
        status = self.get_service_status(service_name)
        if not status["exists"]:
            return False, f"Service '{full_service_name}' does not exist."

        returncode, stdout, stderr = self._run_systemctl(["disable", full_service_name])
        if returncode != 0:
            error_msg = stderr.strip() if stderr else "Unknown error"
            return False, f"Failed to disable service: {error_msg}"
        return True, f"Service '{full_service_name}' disabled from starting on boot"

    def get_service_status(self, service_name: str) -> Dict[str, str]:
        """Get the status of a systemd service."""
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"
        returncode, stdout, stderr = self._run_systemctl(["status", full_service_name, "--no-pager"])

        # Parse basic status info
        status_info = {
            "name": full_service_name,
            "exists": returncode != 4,  # Return code 4 means service not found
            "active": "inactive",
            "enabled": "disabled",
            "output": stdout,
        }

        if status_info["exists"]:
            # Parse active state
            for line in stdout.splitlines():
                if "Active:" in line:
                    if "active (running)" in line:
                        status_info["active"] = "running"
                    elif "active" in line:
                        status_info["active"] = "active"
                    elif "failed" in line:
                        status_info["active"] = "failed"
                elif "Loaded:" in line:
                    if "enabled" in line:
                        status_info["enabled"] = "enabled"

        return status_info

    def list_services(self) -> List[Dict[str, str]]:
        """List all ROS2 systemd services."""
        services = []
        seen_services = set()

        # First, list all unit files (includes inactive services)
        returncode, stdout, stderr = self._run_systemctl(
            ["list-unit-files", f"{self.SERVICE_PREFIX}*.service", "--no-pager", "--plain"]
        )

        if returncode == 0:
            lines = stdout.strip().splitlines()
            for line in lines[1:]:  # Skip header
                if self.SERVICE_PREFIX in line:
                    parts = line.split()
                    if len(parts) >= 2:
                        service_name = parts[0].replace(".service", "").replace(self.SERVICE_PREFIX, "")
                        if service_name and service_name not in seen_services:
                            seen_services.add(service_name)
                            # Get status for this service
                            status = self.get_service_status(service_name)
                            services.append(
                                {
                                    "name": service_name,
                                    "status": status.get("state", "unknown"),
                                    "active": status.get("active_state", "inactive"),
                                }
                            )

        return services

    def remove_service(self, service_name: str) -> Tuple[bool, str]:
        """Remove a systemd service."""
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"
        service_file = self.service_dir / f"{full_service_name}.service"

        # Stop and disable the service first
        self.stop_service(service_name)
        self.disable_service(service_name)

        # Remove the service file
        if service_file.exists():
            service_file.unlink()

            # Reload systemd daemon
            self._run_systemctl(["daemon-reload"])
            return True, f"Service {full_service_name} removed successfully"
        else:
            return False, f"Service file {full_service_name}.service not found"
