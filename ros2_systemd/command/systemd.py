from ros2cli.command import CommandExtension, add_subparsers_on_demand


class SystemdCommand(CommandExtension):
    """Manage ROS2 launches and nodes as systemd services."""

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        add_subparsers_on_demand(parser, cli_name, "_verb", "ros2_systemd.verb", required=False)

    def main(self, *, parser, args):
        if not hasattr(args, "_verb"):
            self._subparser.print_help()
            return 0
        extension = getattr(args, "_verb")
        return extension.main(args=args)
