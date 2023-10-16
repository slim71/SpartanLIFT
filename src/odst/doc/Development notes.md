# Development notes
In some launch files there's this import:
```python
from odst.logs import print_arguments
```
which is highlighted as an error by VSCode.

This actually works when the package is launched and run, because `colcon` puts the python files into `workspace/install/<ros_package_name>lib/python<version>/site-packages/<python_package_name>`, so the import statement is valid.
This is explained in [this post](https://answers.ros.org/question/367793/including-a-python-module-in-a-ros2-package/) from the (now read-only) ROS-answers forum.

---
The optional dictionary `launch_arguments` for `IncludeLaunchDescription` is only valid to pass arguments to a target launch file that defines arguments with `LaunchConfiguration`s.

For node executables, use a list `arguments`, depending on the executable's source.

---
To correctly spawn everything in the same window with `gnome-terminal`, I had to use a temporary file. That's because, even when the `--tab` option is used, launching it from another application always opens a new window instead of a tab.

As reference, see [this comment](https://github.com/GNS3/gns3-gui/issues/3449#issuecomment-1532133451) on GitHub.

---
The empty file in the temp folder is needed because it will be copied over to the share folder of the package and used to structure a bash script.

---
TODO: add ros_gz_sim as package?
TODO: create Micro-XRCE as own package

---
The overall launch with all applications is mostly done via Bash to have more flexibility. \
As an example, starting Gazebo as done in other files (including the launch description of the `ros_gz_sim` package) is equally valid,
but this prevented me from changing the content of the `GZ_SIM_RESOURCE_PATH` environment variable to allow the simulator to find all PX4 models/worlds. Instead of doing it in the `.bashrc` file of the local machine, I opted to highlight this need in the project itself.

---
When launching Gazebo, an error usually appears on screen
```shell
libEGL warning: egl: failed to create dri2 screen
libEGL warning: egl: failed to create dri2 screen
```
This is only a warning and does not affect Gazebo's behavior.\
More info [here](https://github.com/gazebosim/gz-rendering/issues/587).
