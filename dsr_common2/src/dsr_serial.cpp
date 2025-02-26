#include "../include/dsr_serial.h"
#include <sstream>

Serial_comm::Serial_comm(){
    set_port("/dev/ttyUSB0");
    set_baudrate(115200);
    set_timeout(1000);
    set_byte_size(8);
    set_parity(0);
    set_stop_bits(1);
}

Serial_comm::~Serial_comm(){
    ser.close();
    ROS_INFO("serial_comm disconnected!!!!!!!!!!");
    ROS_INFO("serial_comm disconnected!!!!!!!!!!");
    ROS_INFO("serial_comm disconnected!!!!!!!!!!");
    ROS_INFO("serial_comm disconnected!!!!!!!!!!");
}

void Serial_comm::write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

void Serial_comm::read_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Read Data" << msg->data);
}

void Serial_comm::set_port(std::string _port){
    port = _port;
    ser.setPort(port);
}

void Serial_comm::set_timeout(int _time){
    timeout = serial::Timeout::simpleTimeout(_time);
    ser.setTimeout(timeout);
}

void Serial_comm::set_baudrate(int _baudrate){
    baudrate = _baudrate;
    ser.setBaudrate(baudrate);
}

void Serial_comm::set_byte_size(int _byte_size){
    byte_size = (serial::bytesize_t)_byte_size;
    ser.setBytesize(byte_size);
}

void Serial_comm::set_parity(int _parity_size){
    parity_size = (serial::parity_t)_parity_size;
    ser.setParity(parity_size);
}   

void Serial_comm::set_stop_bits(int _stop_bits){
    stop_bits = (serial::stopbits_t)_stop_bits;
    ser.setStopbits(stop_bits);
}

int Serial_comm::ser_open(){
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("/serial_write", 1000, &Serial_comm::write_callback, this);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("/serial_read", 1000);

    try{
        ///you can edit configuration of serial
        ser.open();
    }
    catch(serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port111");
        return -1;
    }

    return 1;
}

int Serial_comm::ser_close(){
    ser.close();
    return 1;
}
/*
void serial_comm::ser_write(unsigned char* ask, int size){
    ser.write(ask, size);
}
*/
/*
void Serial_comm::Activation(){
    unsigned char ask[16] = {0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x30};
    ser.write(ask, 16);

    ros::Duration(0.1).sleep();

    //Set Activation
    unsigned char ask[16] = {0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x72, 0xE1};
    ser.write(ask, 16);
}

void Serial_comm::Open(){
    unsigned char ask[16] = {0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x09, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x72, 0x19};
    ser.write(ask, 16);
}   

void Serial_comm::Close(){
    //Gipper Close(Full speed, Full Force)
    unsigned char ask[16] = {0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, 0x09, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x42, 0x29};
    ser.write(ask, 16);
}

void Serial_comm::Move(int width){
    ///Not Yet
}
*/