/*********************************************************************
 *
 * Author: Minsoo Song (minsoo.song@doosan.com)
 * Copyright (c) 2025 Doosan Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *********************************************************************/


/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    //ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("serial_write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("serial_read", 1000);
    ROS_INFO("start serial_node !");
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.setBytesize((serial::bytesize_t)8);
        ser.setParity((serial::parity_t)0);
        ser.setStopbits((serial::stopbits_t)1);
        ser.open();
    }
    catch (serial::IOException& e)  ///check your port`s permission ///ex) sudo chmod 666 /dev/ttyUSB0
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            ROS_INFO_STREAM("ser.available: " << ser.available());
            result.data = ser.read(ser.available());
            
            const char* read_data = result.data.c_str();
            ROS_INFO_STREAM("Read: " << atoi(read_data));
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}

