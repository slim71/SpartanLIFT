from launch.actions import LogInfo

def print_argument(context):
    stuff_to_log = []

    stuff_to_log.append(LogInfo(msg="=====Launch Configuration Values ====="))

    for var, value in context.launch_configurations.items():
        stuff_to_log.append(LogInfo(msg=f'\'{var}\' has value \'{value}\''))

    return stuff_to_log
