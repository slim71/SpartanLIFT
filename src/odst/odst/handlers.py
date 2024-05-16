import os
from launch.actions import EmitEvent, LogInfo
from launch.events import Shutdown

def shutdown_func_with_echo_side_effect(event, context):
    return [
        LogInfo(msg=f"Shutdown callback was called for reason {event.reason}"),
        # EmitEvent(event=Shutdown(reason="Terminating..."))
        ]
