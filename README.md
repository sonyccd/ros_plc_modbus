# ros_plc_modbus
ROS driver for modbus for use with PLCs or anything else modbus

## Before you begin
You will need to install the c++ mobus library from [here](http://libmodbus.org/download/)  You can either use the prebuilt ones for Ubuntu or install from source, I would recomend installing the latest from source.

##Usage
Simply download this repo into your workspace and you are on your way.  There is a simple example file to get you started.

```bash
cd <workspace>/src
git clone https://github.com/sonyccd/ros_plc_modbus.git
cd <workspace>
catkin_make
source devel/setup.bash
roslaunch plc_modbus_node example.launch
```
