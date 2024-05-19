class LaunchfileLogger:
    """
    Simple class made to be able to log something only if the 'custom' level is specified.
    Preferred as workaround instead of using global variables/constants.
    """

    _loglevel: str

    def __init__(self):
        self._loglevel = ""

    def print(self, msg_content):
        """
        Print the message on screen.

        Args:
            msg_content (str): String to print
        """
        if self._loglevel == "debug":
            print(f"[TRACE] {msg_content}")

    def set_level(self, level: str):
        """
        Set the Logger level.

        Args:
            level (str): level to set
        """
        self._loglevel = level

    def get_level(self):
        """
        Returns the current level of the Logger.

        Returns:
            str: current level of the Logger
        """
        return self._loglevel
