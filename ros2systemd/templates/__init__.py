"""Service templates for common ROS2 packages."""

TEMPLATES = {
    "turtlesim": {
        "description": "TurtleSim visualization node",
        "package": "turtlesim",
        "executable": "turtlesim_node",
        "env_vars": {"QT_QPA_PLATFORM": "xcb"},  # Ensure GUI works
    },
    "talker": {"description": "Demo talker node (C++)", "package": "demo_nodes_cpp", "executable": "talker"},
    "listener": {"description": "Demo listener node (C++)", "package": "demo_nodes_cpp", "executable": "listener"},
    "talker-py": {"description": "Demo talker node (Python)", "package": "demo_nodes_py", "executable": "talker"},
    "listener-py": {"description": "Demo listener node (Python)", "package": "demo_nodes_py", "executable": "listener"},
    "teleop-keyboard": {
        "description": "Keyboard teleoperation for turtlesim",
        "package": "turtlesim",
        "executable": "turtle_teleop_key",
    },
    "joy": {"description": "Joystick driver node", "package": "joy", "executable": "joy_node"},
    "robot-state-publisher": {
        "description": "Robot state publisher",
        "package": "robot_state_publisher",
        "executable": "robot_state_publisher",
        "node_args": ["--ros-args", "-p", "use_sim_time:=false"],
    },
    "static-transform": {
        "description": "Static transform publisher",
        "package": "tf2_ros",
        "executable": "static_transform_publisher",
        "node_args": ["0", "0", "0", "0", "0", "0", "map", "base_link"],
    },
}


def get_template(template_name: str) -> dict:
    """
    Get a service template by name.

    Args:
        template_name: Name of the template

    Returns:
        Template dictionary or None if not found
    """
    return TEMPLATES.get(template_name)


def list_templates() -> list:
    """
    List all available templates.

    Returns:
        List of template names
    """
    return list(TEMPLATES.keys())


def get_template_info(template_name: str) -> str:
    """
    Get information about a template.

    Args:
        template_name: Name of the template

    Returns:
        Template information string
    """
    template = TEMPLATES.get(template_name)
    if not template:
        return f"Template '{template_name}' not found"

    info = [
        f"Template: {template_name}",
        f"Description: {template['description']}",
        f"Package: {template['package']}",
        f"Executable: {template['executable']}",
    ]

    if template.get("node_args"):
        info.append(f"Arguments: {' '.join(template['node_args'])}")

    if template.get("env_vars"):
        env_str = ", ".join(f"{k}={v}" for k, v in template["env_vars"].items())
        info.append(f"Environment: {env_str}")

    return "\n".join(info)
