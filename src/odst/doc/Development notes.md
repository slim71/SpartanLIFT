# Development notes
In some launch files there's this import:
```python
from odst.logs import print_argument
```
which is highlighted as an error by VSCode.

This actually works when the package is launched and run, because `colcon` puts the python files into `workspace/install/<ros_package_name>lib/python<version>/site-packages/<python_package_name>`, so the import statement is valid.
This is explained in [this post](https://answers.ros.org/question/367793/including-a-python-module-in-a-ros2-package/) from the (now read-only) ROS-answers forum.

---
The optional dictionary `launch_arguments` for `IncludeLaunchDescription` is only valid to pass arguments to a target launch file that defines arguments with `LaunchConfiguration`s.

For node executables, use a list `arguments`, depending on the executable's source.
