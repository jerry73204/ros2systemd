from ros2_systemd.api.systemd_manager import SystemdServiceManager
from ros2_systemd.templates import get_template, get_template_info, list_templates
from ros2_systemd.verb import VerbExtension


class TemplateVerb(VerbExtension):
    """Create a service from a template or list available templates."""

    def add_arguments(self, parser, cli_name):
        # Add subcommands
        subparsers = parser.add_subparsers(dest="subcommand", help="Template operations")

        # List templates subcommand
        subparsers.add_parser("list", help="List available templates")

        # Info subcommand
        info_parser = subparsers.add_parser("info", help="Show template information")
        info_parser.add_argument("template_name", help="Name of the template")

        # Create subcommand
        create_parser = subparsers.add_parser("create", help="Create service from template")
        create_parser.add_argument("service_name", help="Name for the service")
        create_parser.add_argument("template_name", help="Template to use")
        create_parser.add_argument(
            "--system", action="store_true", help="Create system service instead of user service"
        )
        create_parser.add_argument("--env", nargs="*", help="Additional environment variables in KEY=VALUE format")

    def main(self, *, args):
        if not hasattr(args, "subcommand") or args.subcommand is None:
            print("Usage: ros2 systemd template [list|info|create] ...")
            return 1

        if args.subcommand == "list":
            return self._list_templates()
        elif args.subcommand == "info":
            return self._show_template_info(args.template_name)
        elif args.subcommand == "create":
            return self._create_from_template(args)

        return 1

    def _list_templates(self):
        """List all available templates."""
        templates = list_templates()
        if not templates:
            print("No templates available")
            return 0

        print("Available service templates:")
        print("-" * 40)
        for name in sorted(templates):
            template = get_template(name)
            print(f"  {name:<20} - {template['description']}")

        print("\nUse 'ros2 systemd template info <name>' for details")
        print("Use 'ros2 systemd template create <service_name> <template>' to create a service")
        return 0

    def _show_template_info(self, template_name):
        """Show information about a template."""
        info = get_template_info(template_name)
        print(info)

        if not get_template(template_name):
            print("\nAvailable templates:")
            for name in sorted(list_templates()):
                print(f"  - {name}")
            return 1

        return 0

    def _create_from_template(self, args):
        """Create a service from a template."""
        template = get_template(args.template_name)
        if not template:
            print(f"Error: Template '{args.template_name}' not found")
            print("\nAvailable templates:")
            for name in sorted(list_templates()):
                print(f"  - {name}")
            return 1

        # Prepare environment variables
        env_vars = template.get("env_vars", {}).copy()
        if args.env:
            for env_var in args.env:
                if "=" in env_var:
                    key, value = env_var.split("=", 1)
                    env_vars[key] = value

        # Create the service
        manager = SystemdServiceManager(user_mode=not args.system)
        success = manager.create_node_service(
            service_name=args.service_name,
            package=template["package"],
            executable=template["executable"],
            node_args=template.get("node_args"),
            env_vars=env_vars if env_vars else None,
            description=f"{template['description']} (from template: {args.template_name})",
        )

        if success:
            print(f"Successfully created service 'ros2-{args.service_name}' from template '{args.template_name}'")
            print(f"Use 'ros2 systemd start {args.service_name}' to start the service")
            return 0
        else:
            print("Failed to create service from template")
            return 1
