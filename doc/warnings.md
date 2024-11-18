
```shell
$ colcon build
Starting >>> px4_msgs
Starting >>> comms
Starting >>> cargo
Starting >>> microxrcedds_agent
Starting >>> odst
Starting >>> reach
Finished <<< cargo [2.08s]  
Finished <<< reach [2.50s]  
--- stderr: odst  
/home/slim71/.local/lib/python3.10/site-packages/setuptools/_distutils/cmd.py:66: SetuptoolsDeprecationWarning: setup.py install is deprecated.
!!

        ********************************************************************************
        Please avoid running ``setup.py`` directly.
        Instead, use pypa/build, pypa/installer, pypa/build or
        other standards-based tools.

        See https://blog.ganssle.io/articles/2021/10/setup-py-deprecated.html for details.
        ********************************************************************************

!!
  self.initialize_options()
---
Finished <<< odst [3.02s]
Finished <<< comms [13.9s]  
[Processing: microxrcedds_agent, px4_msgs]  
[Processing: microxrcedds_agent, px4_msgs]  
[Processing: microxrcedds_agent, px4_msgs]  
--- stderr: microxrcedds_agent  
Cloning into 'spdlog'...
HEAD is now at 7e635fca Fixed #2724 by excluding bin_to_hex sink if using std::format
CMake Warning (dev) at /usr/share/cmake-3.22/Modules/FindPackageHandleStandardArgs.cmake:438 (message):
  The package name passed to `find_package_handle_standard_args` (tinyxml2)
  does not match the name of the calling package (TinyXML2).  This can lead
  to problems in calling code that expects `find_package` result variables
  (e.g., `_FOUND`) to follow a certain pattern.
Call Stack (most recent call first):
  cmake/modules/FindTinyXML2.cmake:40 (find_package_handle_standard_args)
  /opt/ros/humble/share/fastrtps/cmake/fastrtps-config.cmake:51 (find_package)
  CMakeLists.txt:153 (find_package)
This warning is for project developers.  Use -Wno-dev to suppress it.

---
Finished <<< microxrcedds_agent [1min 53s]
[Processing: px4_msgs]  
[Processing: px4_msgs]  
[Processing: px4_msgs]  
[Processing: px4_msgs]  
[Processing: px4_msgs]  
[Processing: px4_msgs]  
[Processing: px4_msgs]  
[Processing: px4_msgs]  
[Processing: px4_msgs]  
[Processing: px4_msgs]  
Finished <<< px4_msgs [7min 10s]  
Starting >>> pelican
Starting >>> px4_ros_com
Finished <<< px4_ros_com [20.8s]  
[Processing: pelican]  
--- stderr: pelican  
/home/slim71/Documents/git/SpartanLIFT/src/pelican/src/PelicanModule/pelican.cpp: In static member function ‘static void Pelican::signalHandler(int)’:
/home/slim71/Documents/git/SpartanLIFT/src/pelican/src/PelicanModule/pelican.cpp:84:33: warning: unused parameter ‘signum’ [-Wunused-parameter]
   84 | void Pelican::signalHandler(int signum) {
      |                             ~~~~^~~~~~
/home/slim71/Documents/git/SpartanLIFT/src/pelican/src/PelicanModule/pelican.cpp: In static member function ‘static void Pelican::signalHandler(int)’:
/home/slim71/Documents/git/SpartanLIFT/src/pelican/src/PelicanModule/pelican.cpp:84:33: warning: unused parameter ‘signum’ [-Wunused-parameter]
   84 | void Pelican::signalHandler(int signum) {
      |                             ~~~~^~~~~~
/home/slim71/Documents/git/SpartanLIFT/src/pelican/test/src/fixtures.cpp: In lambda function:
/home/slim71/Documents/git/SpartanLIFT/src/pelican/test/src/fixtures.cpp:68:69: warning: unused parameter ‘msg’ [-Wunused-parameter]
   68 |         [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
      |                ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^~~
/home/slim71/Documents/git/SpartanLIFT/src/pelican/test/src/fixtures.cpp: In lambda function:
/home/slim71/Documents/git/SpartanLIFT/src/pelican/test/src/fixtures.cpp:79:55: warning: unused parameter ‘msg’ [-Wunused-parameter]
   79 |         [this](const comms::msg::Heartbeat::SharedPtr msg) {
      |                ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^~~
/home/slim71/Documents/git/SpartanLIFT/src/pelican/test/src/fixtures.cpp: In lambda function:
/home/slim71/Documents/git/SpartanLIFT/src/pelican/test/src/fixtures.cpp:90:53: warning: unused parameter ‘msg’ [-Wunused-parameter]
   90 |         [this](const comms::msg::Datapad::SharedPtr msg) {
      |                ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^~~
/home/slim71/Documents/git/SpartanLIFT/src/pelican/src/ElectionModule/election.cpp: In member function ‘void ElectionModule::serveVoteRequest(comms::msg::RequestVoteRPC) const’:
/home/slim71/Documents/git/SpartanLIFT/src/pelican/src/ElectionModule/election.cpp:192:72: warning: unused parameter ‘msg’ [-Wunused-parameter]
  192 | void ElectionModule::serveVoteRequest(const comms::msg::RequestVoteRPC msg) const {
      |                                       ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^~~
/home/slim71/Documents/git/SpartanLIFT/src/pelican/src/PelicanModule/pelican.cpp: In static member function ‘static void Pelican::signalHandler(int)’:
/home/slim71/Documents/git/SpartanLIFT/src/pelican/src/PelicanModule/pelican.cpp:84:33: warning: unused parameter ‘signum’ [-Wunused-parameter]
   84 | void Pelican::signalHandler(int signum) {
      |                             ~~~~^~~~~~
/home/slim71/Documents/git/SpartanLIFT/src/pelican/src/ElectionModule/election.cpp: In member function ‘void ElectionModule::serveVoteRequest(comms::msg::RequestVoteRPC) const’:
/home/slim71/Documents/git/SpartanLIFT/src/pelican/src/ElectionModule/election.cpp:192:72: warning: unused parameter ‘msg’ [-Wunused-parameter]
  192 | void ElectionModule::serveVoteRequest(const comms::msg::RequestVoteRPC msg) const {
      |                                       ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^~~
---
Finished <<< pelican [1min 20s]

Summary: 8 packages finished [8min 31s]
  3 packages had stderr output: microxrcedds_agent odst pelican

```
