from launch.actions import LogInfo


def print_arguments(context):
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

    def print(self, msg_content: str):
        if self.loglevel == 'debug':
            print(f'[DEBUG] {msg_content}')

    def setLevel(self, l: str):
        self.loglevel = l
