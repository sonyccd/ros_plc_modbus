# ros_plc_modbus
ROS driver for modbus for use with PLCs or anything else modbus. Only tested on ROS Indigo.

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

You give a list of address to be used for coils and registers.  When you read or write back over the topics you will do so with an array that is mapped to the address you gave in the parameters.  
Example:  
If you have a set of address [3,56,32,4,7] then when you read from the topic or write you will give an array such that the first element maps to address 3 and the second to 56 and so on. You can not change the address once the node is started, for now.

##Parameters

|Parameter|Default|Definition|
|-----|----------|-------|
|ip|192.168.0.100|IP address of the modbus device|
|port|502|Port for communication with the modbus device|
|spin_rate|30|The rate ROS will refresh the coil and register read topics in Hz|
|regs_addr|NULL|An array of register addresses to be written and read from|
|coils_addr|NULL|An array of coil addresses to be written and read from|

##Topics

###Subscribed

/modbus/regs_read [(std_msgs/UInt16MultiArray)](http://docs.ros.org/api/std_msgs/html/msg/UInt16MultiArray.html)  
This is the current value of the registers.  
/modbus/coils_read [(std_msgs/ByteMultiArray)](http://docs.ros.org/api/std_msgs/html/msg/ByteMultiArray.html)  
This is the current value of the coils (TRUE or FALSE).

###Published

/modbus/regs_write [(std_msgs/UInt16MultiArray)](http://docs.ros.org/api/std_msgs/html/msg/UInt16MultiArray.html)  
What to set the given register address to.  
/modbus/coils_write [(std_msgs/ByteMultiArray)](http://docs.ros.org/api/std_msgs/html/msg/ByteMultiArray.html)  
What to set the given coil address to (TRUE or FALSE).  

##What is Modbus?

![Alt Text](http://www.controlsystemworks.com/i/Features/Modbus.jpg)
>Modbus is a serial communications protocol originally published by Modicon (now Schneider Electric) in 1979 for use with its programmable logic controllers (PLCs). Modbus enables communication among many devices connected to the same network, for example a system that measures temperature and humidity and communicates the results to a computer. In simple terms, it is a method used for transmitting information over serial lines between electronic devices. The device requesting the information is called the Modbus Master and the devices supplying information are Modbus Slaves. In a standard Modbus network, there is one Master and up to 247 Slaves, each with a unique Slave Address from 1 to 247. The Master can also write information to the Slaves.

#IF SOMETHING IS BROEKN:
Please file an issue, it makes it far easier to keep track of what needs to be fixed. It also allows others that might have solved the problem to contribute.  If you are confused feel free to email me, I might have overlooked something in my readme.
