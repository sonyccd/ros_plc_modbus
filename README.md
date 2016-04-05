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

##How it works

##What is Modbus?
![Alt Text](http://www.controlsystemworks.com/i/Features/Modbus.jpg)
>Modbus is a serial communications protocol originally published by Modicon (now Schneider Electric) in 1979 for use with its programmable logic controllers (PLCs). Modbus enables communication among many devices connected to the same network, for example a system that measures temperature and humidity and communicates the results to a computer. In simple terms, it is a method used for transmitting information over serial lines between electronic devices. The device requesting the information is called the Modbus Master and the devices supplying information are Modbus Slaves. In a standard Modbus network, there is one Master and up to 247 Slaves, each with a unique Slave Address from 1 to 247. The Master can also write information to the Slaves.
