//
// Created by Brad Bazemore on 10/29/15.
//
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <modbus/modbus.h>

class Modbus_manager{
public:
    Modbus_manager();
private:
    ros::NodeHandle node;
    ros::Publisher mod_out;
    ros::Subscriber mod_in;
    uint16_t reg_in[10];
    uint16_t reg_out[10];
    std_msgs::UInt16MultiArray reg;
    modbus_t *plc;

    void callBack(const std_msgs::UInt16MultiArray::ConstPtr &reg_data);
};

Modbus_manager::Modbus_manager() {
    mod_out = node.advertise<std_msgs::UInt16MultiArray>("modbus/out",100);
    mod_in = node.subscribe<std_msgs::UInt16MultiArray>("modbus/in", 100, &Modbus_manager::callBack,this);
    plc = modbus_new_tcp("192.168.1.2",502);
    modbus_connect(plc);
    ros::Rate loop_rate(100);
    for(int i=0;i<sizeof(reg_out);i++){reg_out[i]=0;}
    while(ros::ok()){
        reg.data.clear();
        modbus_read_registers(plc,0,10, reg_in);
        for(int i=0;i<10;i++) {
            reg.data.push_back(reg_in[i]);
        }
        mod_out.publish(reg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    modbus_close(plc);
    modbus_free(plc);
    return;
}

void Modbus_manager::callBack(const std_msgs::UInt16MultiArray::ConstPtr &reg_data){
    for(int i=0;i<reg_data->data.size();i++){
        reg_out[i]=reg_data->data.at(i);
        ROS_INFO("reg_out[%d]:%u",i,reg_out[i]);
    }
    if(modbus_write_registers(plc,0,10,reg_out)==-1){
        ROS_WARN("Modbus write failed");
    } else{
        for(int i=0;i<sizeof(reg_out);i++){reg_out[i]=0;}
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv,"ros_modbus");
    Modbus_manager mm;
    return 0;
}
