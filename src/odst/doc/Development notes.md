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
