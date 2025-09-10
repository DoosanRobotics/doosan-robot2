/*********************************************************************
 *
 * dsr_common2
 * Author: Minsoo Song (minsoo.song@doosan.com)
 * 
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


#ifndef DR_SERIAL_H
#define DR_SERIAL_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

class Serial_comm{

public:
        Serial_comm();
        Serial_comm(std::string _port, int _timeout);
        ~Serial_comm();

        void set_port(std::string _port);
        void set_baudrate(int _baudrate);
        void set_byte_size(int _byte_size);
        void set_parity(int _parity_size);
        void set_stop_bits(int _stop_bits);
        void set_timeout(int time);
        int ser_open();
        //void ser_write(unsigned char* ask, int size);
        int ser_close();
        //void write_callback(const std_msgs::String::ConstPtr& msg);
        void write_callback(const std_msgs::String::ConstPtr& msg);
        void read_callback(const std_msgs::String::ConstPtr& msg);
        /////for Robotiq Modbus RTU(FullSpeed/FullForce)

        void Activation();
        void Open();
        void Close();
        void Move(int width);
        serial::Serial ser;
    private:
        std::string port;
        int baudrate;
        serial::bytesize_t byte_size;
        serial::parity_t parity_size;
        serial::stopbits_t stop_bits;
        serial::Timeout timeout;
};



#endif