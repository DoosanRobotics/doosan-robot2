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

#include "rclcpp/rclcpp.hpp"

#include "dsr_realtime_control/rt_shutdown.hpp"

#include <chrono>
#include <atomic>
#include <memory>
#include <thread>

RtShutdownNode::RtShutdownNode()    :    Node("RtShutdown")
{
    client1_ = this->create_client<dsr_msgs2::srv::StopRtControl>("/dsr01/realtime/stop_rt_control");
    client1_thread_ = std::thread(std::bind(&RtShutdownNode::StopRtControlClient, this));
    client2_ = this->create_client<dsr_msgs2::srv::DisconnectRtControl>("/dsr01/realtime/disconnect_rt_control");
    client2_thread_ = std::thread(std::bind(&RtShutdownNode::DisconnectRtControlClient, this));
}

RtShutdownNode::~RtShutdownNode()
{
    if(client1_thread_.joinable())
    {
        client1_thread_.join(); 
        RCLCPP_INFO(this->get_logger(), "client1_thread_.joined");
    }
    if(client2_thread_.joinable())
    {
        client2_thread_.join();
        RCLCPP_INFO(this->get_logger(), "client2_thread_.joined");
    }
}

void RtShutdownNode::StopRtControlClient()
{
    rclcpp::Rate rate(0.5);
    while(rclcpp::ok() && rt_started)
    {
        rate.sleep();
        if (!client1_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "thread1: Waiting for the server to be up...");
            continue;
        }
        auto request = std::make_shared<dsr_msgs2::srv::StopRtControl::Request>();
        auto future = client1_->async_send_request(request);
        try
        {
            auto response = future.get();
            if(response->success)
            {
                rt_started=false;
            }
            RCLCPP_INFO(this->get_logger(), "RT stopped");
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
}

void RtShutdownNode::DisconnectRtControlClient()
{
    rclcpp::Rate rate(0.5);
    while(rclcpp::ok() && rt_connected)
    {
        rate.sleep();
        if (!rt_started)
        {
            if (!client2_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "thread1: Waiting for the server to be up...");
                continue;
            }
            auto request = std::make_shared<dsr_msgs2::srv::DisconnectRtControl::Request>();
            auto future = client2_->async_send_request(request);
            try
            {
                auto response = future.get();
                if(response->success)
                {
                    rt_connected=false;
                }
                RCLCPP_INFO(this->get_logger(), "RT disconnected");
            }
            catch(const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
        }
    }
}



int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RtShutdownNode>();
    while(rclcpp::ok() && rt_connected)
    {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
