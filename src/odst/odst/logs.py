from launch.actions import LogInfo


def print_arguments(context):
    """
    Helper function to build additional debug info when using launch files.
    This info will be printed to screen.

    Args:
        context (rclcpp::Context): context passed via ROS2

    Returns:
        list: List of additional info to print
    """
    stuff_to_log = []

    stuff_to_log.append(LogInfo(msg="=====Launch Configuration Values ====="))

    for var, value in context.launch_configurations.items():
        stuff_to_log.append(LogInfo(msg=f'\'{var}\' has value \'{value}\''))

    return stuff_to_log


class LogDebug():
    """
    Simple class made to be able to log something only if the 'debug' level is specified.
    Preferred as workaround instead of using global variables/constants.
    """
    loglevel: str

    def __init__(self):
        self.loglevel = 'info'

    def print(self, msg_content):
        """
        Print the message on screen.

        Args:
            msg_content (str): String to print
        """
        if self.loglevel == 'debug':
            print(f'[DEBUG] {msg_content}')

    def setLevel(self, l: str):
        """
        Set the Logger level.

        Args:
            l (str): level to set
        """
        self.loglevel = l


    def getLevel(self):
        """
        Returns the current level of the Logger.

        Returns:
            str: current level of the Logger
        """
        return self.loglevel
