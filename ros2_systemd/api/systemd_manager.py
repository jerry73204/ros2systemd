import os
import subprocess
import tempfile
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
    
    def create_launch_service(self, 
                            service_name: str,
                            launch_file: str,
                            launch_args: Optional[Dict[str, str]] = None,
                            env_vars: Optional[Dict[str, str]] = None,
                            description: Optional[str] = None) -> bool:
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
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"
        service_file = self.service_dir / f"{full_service_name}.service"
        
        # Build the launch command
        launch_cmd = ["ros2", "launch", launch_file]
        if launch_args:
            for key, value in launch_args.items():
                launch_cmd.append(f"{key}:={value}")
        
        # Create service content
        service_content = self._generate_service_content(
            description=description or f"ROS2 launch service for {launch_file}",
            exec_command=" ".join(launch_cmd),
            env_vars=env_vars
        )
        
        # Write service file
        service_file.write_text(service_content)
        
        # Reload systemd daemon
        self._run_systemctl(["daemon-reload"])
        
        return service_file.exists()
    
    def create_node_service(self,
                          service_name: str,
                          package: str,
                          executable: str,
                          node_args: Optional[List[str]] = None,
                          env_vars: Optional[Dict[str, str]] = None,
                          description: Optional[str] = None) -> bool:
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
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"
        service_file = self.service_dir / f"{full_service_name}.service"
        
        # Build the node command
        node_cmd = ["ros2", "run", package, executable]
        if node_args:
            node_cmd.extend(node_args)
        
        # Create service content
        service_content = self._generate_service_content(
            description=description or f"ROS2 node service for {package}/{executable}",
            exec_command=" ".join(node_cmd),
            env_vars=env_vars
        )
        
        # Write service file
        service_file.write_text(service_content)
        
        # Reload systemd daemon
        self._run_systemctl(["daemon-reload"])
        
        return service_file.exists()
    
    def _generate_service_content(self,
                                 description: str,
                                 exec_command: str,
                                 env_vars: Optional[Dict[str, str]] = None) -> str:
        """Generate systemd service file content."""
        service_lines = [
            "[Unit]",
            f"Description={description}",
            "After=network.target",
            "",
            "[Service]",
            "Type=simple",
            f"ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && {exec_command}'",
            "Restart=on-failure",
            "RestartSec=5",
            "StandardOutput=journal",
            "StandardError=journal",
        ]
        
        # Add environment variables
        if env_vars:
            for key, value in env_vars.items():
                service_lines.append(f"Environment=\"{key}={value}\"")
        
        # Add ROS2 specific environment
        service_lines.extend([
            "Environment=\"ROS_DOMAIN_ID=0\"",
            "Environment=\"ROS_LOCALHOST_ONLY=0\"",
        ])
        
        service_lines.extend([
            "",
            "[Install]",
            "WantedBy=default.target" if self.user_mode else "WantedBy=multi-user.target",
        ])
        
        return "\n".join(service_lines)
    
    def start_service(self, service_name: str) -> Tuple[bool, str]:
        """Start a systemd service."""
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"
        returncode, stdout, stderr = self._run_systemctl(["start", full_service_name])
        return returncode == 0, stderr if returncode != 0 else stdout
    
    def stop_service(self, service_name: str) -> Tuple[bool, str]:
        """Stop a systemd service."""
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"
        returncode, stdout, stderr = self._run_systemctl(["stop", full_service_name])
        return returncode == 0, stderr if returncode != 0 else stdout
    
    def restart_service(self, service_name: str) -> Tuple[bool, str]:
        """Restart a systemd service."""
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"
        returncode, stdout, stderr = self._run_systemctl(["restart", full_service_name])
        return returncode == 0, stderr if returncode != 0 else stdout
    
    def enable_service(self, service_name: str) -> Tuple[bool, str]:
        """Enable a systemd service to start on boot."""
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"
        returncode, stdout, stderr = self._run_systemctl(["enable", full_service_name])
        return returncode == 0, stderr if returncode != 0 else stdout
    
    def disable_service(self, service_name: str) -> Tuple[bool, str]:
        """Disable a systemd service from starting on boot."""
        full_service_name = f"{self.SERVICE_PREFIX}{service_name}"
        returncode, stdout, stderr = self._run_systemctl(["disable", full_service_name])
        return returncode == 0, stderr if returncode != 0 else stdout
    
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
            "output": stdout
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
        
        # List all services with our prefix
        returncode, stdout, stderr = self._run_systemctl(
            ["list-units", f"{self.SERVICE_PREFIX}*", "--all", "--no-pager", "--plain"]
        )
        
        if returncode == 0:
            lines = stdout.strip().splitlines()
            for line in lines[1:]:  # Skip header
                if self.SERVICE_PREFIX in line:
                    parts = line.split()
                    if len(parts) >= 4:
                        service_name = parts[0].replace(".service", "").replace(self.SERVICE_PREFIX, "")
                        services.append({
                            "name": service_name,
                            "status": parts[2],
                            "active": parts[3]
                        })
        
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