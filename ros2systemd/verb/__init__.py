from ros2cli.plugin_system import PLUGIN_SYSTEM_VERSION, satisfies_version


class VerbExtension:
    """Base class for ros2systemd verb extensions."""

    NAME = None
    EXTENSION_POINT_VERSION = "0.1"

    def __init__(self):
        super(VerbExtension, self).__init__()
        satisfies_version(PLUGIN_SYSTEM_VERSION, "^0.1")

    def add_arguments(self, parser, cli_name):
        pass

    def main(self, *, args):
        raise NotImplementedError()
