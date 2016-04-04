//
// Created by Brad Bazemore on 10/29/15.
//
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <modbus/modbus.h>

class plc_modbus_manager{
public:
    plc_modbus_manager();
private:
    ros::NodeHandle node;

    ros::Publisher regs_read;
    ros::Subscriber regs_write;
    ros::Publisher coils_read;
    ros::Subscriber coils_write;

    std::vector<int> regs_addr;
    std::vector<int> coils_addr;

    std_msgs::UInt16MultiArray regs_val;
    std_msgs::ByteMultiArray coils_val;

    modbus_t *plc;

    std::string ip_address;
    int port;

    void regs_callBack(const std_msgs::UInt16MultiArray::ConstPtr &regs_data);
    void coils_callBack(const std_msgs::ByteMultiArray::ConstPtr &coils_data);
};

plc_modbus_manager::plc_modbus_manager() {

    regs_read = node.advertise<std_msgs::UInt16MultiArray>("modbus/regs_read",100);
    regs_write = node.subscribe<std_msgs::UInt16MultiArray>("modbus/regs_write", 100, &plc_modbus_manager::regs_callBack, this);
    coils_read = node.advertise<std_msgs::ByteMultiArray>("modbus/coils_read",100);
    coils_write = node.subscribe<std_msgs::ByteMultiArray>("modbus/coils_write",100, &plc_modbus_manager::coils_callBack, this);

    node.param<std::string>("modbus/ip",ip_address,"192.168.0.100");
    node.param("modbus/port",port,502);

    if(!node.getParam("plc_modbus_node/regs_addr",regs_addr)){
        ROS_WARN("No reg addrs given!");
    }
    if(!node.getParam("plc_modbus_node/coils_addr",coils_addr)){
        ROS_WARN("No coil addrs given!");
    }

    ROS_INFO("Connecting to modbus device on %s/%d",ip_address.c_str(),port);
    plc = modbus_new_tcp(ip_address.c_str(),port);
    if (plc == NULL) {
        ROS_FATAL("Unable to allocate libmodbus context\n");
        return;
    }
    if(modbus_connect(plc)==-1){
        ROS_FATAL("Failed to connect to modbus device!!!");
        ROS_FATAL("%s",modbus_strerror(errno));
        modbus_free(plc);
        return;
    }else{
        ROS_INFO("Connection to modbus device established");
    }

    ros::Rate loop_rate(10);

    while(ros::ok()){
        regs_val.data.clear();
        coils_val.data.clear();

        for(int i=0;i<regs_addr.size();i++){
            if(modbus_read_registers(plc, regs_addr.at(i), 1, (uint16_t *) regs_val.data.at(i)) == -1){
                ROS_ERROR("Unable to read reg addr:%d",regs_addr.at(i));
                ROS_ERROR("%s",modbus_strerror(errno));
            }
        }
        if(regs_val.data.size()>0) {
            regs_read.publish(regs_val);
        }

        for(int i=0;i<coils_addr.size();i++){
            uint8_t temp[1]={0};
            if(modbus_read_bits(plc, coils_addr.at(i), 1, temp)==-1){
                ROS_ERROR("Unable to read coil addr:%d",regs_addr.at(i));
                ROS_ERROR("%s",modbus_strerror(errno));
            }else{
                coils_val.data.push_back(temp[0]);
            }
        }
        if(coils_val.data.size()>0) {
            coils_read.publish(coils_val);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    modbus_close(plc);
    modbus_free(plc);
    return;
}

void plc_modbus_manager::regs_callBack(const std_msgs::UInt16MultiArray::ConstPtr &regs_data){
    if(regs_data->data.size()!=regs_addr.size()){
        ROS_ERROR("Number of regs to write to number of values provided do not match");
        return;
    }
    for(int i=0;i<regs_data->data.size();i++){
        ROS_DEBUG("regs_out[%d]:%u",i,regs_data->data.at(i));
        if(modbus_write_registers(plc, regs_addr.at(i), 1, (const uint16_t *) regs_data->data.at(i)) == -1){
            ROS_ERROR("Modbus reg write failed at addr:%d with value:%u",regs_addr.at(i),regs_data->data.at(i));
            ROS_ERROR("%s",modbus_strerror(errno));
        }
    }
}

void plc_modbus_manager::coils_callBack(const std_msgs::ByteMultiArray::ConstPtr &coils_data){
    if(coils_data->data.size()!=coils_addr.size()){
        ROS_ERROR("Number of coils to write to number of values provided do not match");
        return;
    }
    for(int i=0;i<coils_data->data.size();i++){
        ROS_DEBUG("regs_out[%d]:%u",i,coils_data->data.at(i));
        if(modbus_write_bits(plc, coils_addr.at(i), 1, (const uint8_t *) coils_data->data.at(i)) == -1){
            ROS_ERROR("Modbus coil write failed at addr:%d with value:%u",coils_addr.at(i),coils_data->data.at(i));
            ROS_ERROR("%s",modbus_strerror(errno));
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv,"ros_plc_modbus");
    plc_modbus_manager mm;
    return 0;
}
