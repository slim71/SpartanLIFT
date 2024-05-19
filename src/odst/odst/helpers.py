from odst.logs import LaunchfileLogger

helpers_logger = LaunchfileLogger()
helpers_logger.set_level("custom")


def print_launch_configuration(context):
    """
    Helper function to build additional debug info when using launch files.
    This info will be printed to screen.

    Args:
        context (rclcpp::Context): context passed via ROS2
    """
    helpers_logger.print("===== Argument Values =====")

    for var, value in context.launch_configurations.items():
        helpers_logger.print(f"'{var}' has value '{value}'")
