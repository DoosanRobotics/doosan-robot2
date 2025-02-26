#include "rclcpp/rclcpp.hpp"

#include "dsr_realtime_control/rt_init.hpp"

#include <chrono>
#include <atomic>
#include <memory>
#include <thread>

RtInitNode::RtInitNode()    :    Node("RtInit")
{
    client1_ = this->create_client<dsr_msgs2::srv::ConnectRtControl>("/dsr01/realtime/connect_rt_control");
    client1_thread_ = std::thread(std::bind(&RtInitNode::ConnectRtControlClient, this));
    client2_ = this->create_client<dsr_msgs2::srv::SetRtControlOutput>("/dsr01/realtime/set_rt_control_output");
    client2_thread_ = std::thread(std::bind(&RtInitNode::SetRtControlOutputClient, this));
    client3_ = this->create_client<dsr_msgs2::srv::StartRtControl>("/dsr01/realtime/start_rt_control");
    client3_thread_ = std::thread(std::bind(&RtInitNode::StartRtControlClient, this));
}

RtInitNode::~RtInitNode()
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
    if(client3_thread_.joinable())
    {
        client3_thread_.join();
        RCLCPP_INFO(this->get_logger(), "client3_thread_.joined");
    }
}

void RtInitNode::ConnectRtControlClient()
{
    rclcpp::Rate rate(0.5);
    while(rclcpp::ok() && !rt_connected)
    {
        rate.sleep();
        if (!client1_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "thread1: Waiting for the server to be up...");
            continue;
        }
        auto request = std::make_shared<dsr_msgs2::srv::ConnectRtControl::Request>();
        request->ip_address  =   "192.168.137.100";
        request->port       =   12347;
        auto future = client1_->async_send_request(request);
        try
        {
            auto response = future.get();
            if(response->success)
            {
                rt_connected=true;
            }
            RCLCPP_INFO(this->get_logger(), "RT connected");
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
}

void RtInitNode::SetRtControlOutputClient()
{
    rclcpp::Rate rate(0.5);
    while(rclcpp::ok() && !rt_output_set)
    {
        rate.sleep();
        if (rt_connected)
        {
            if (!client2_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "thread2: Waiting for the server to be up...");
                continue;
            }
            auto request = std::make_shared<dsr_msgs2::srv::SetRtControlOutput::Request>();
            request->version    =   "v1.0";
            request->period     =   0.001;
            request->loss       =   4;
            auto future = client2_->async_send_request(request);
            try
            {
                auto response = future.get();
                if(response->success)
                {
                    rt_output_set=true;
                }
                RCLCPP_INFO(this->get_logger(), "RT control output set");
            }
            catch(const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
        }
    }
}

void RtInitNode::StartRtControlClient()
{
    rclcpp::Rate rate(0.5);
    while(rclcpp::ok() && !rt_started)
    {
        rate.sleep();
        if (rt_connected && rt_output_set)
        {
            if (!client3_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "thread3: Waiting for the server to be up...");
                continue;
            }
            auto request = std::make_shared<dsr_msgs2::srv::StartRtControl::Request>();
            auto future = client3_->async_send_request(request);
            try
            {
                auto response = future.get();
                if(response->success)
                {
                    rt_started=true;
                }
                RCLCPP_INFO(this->get_logger(), "RT started");
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
    auto node = std::make_shared<RtInitNode>();
    while(rclcpp::ok() && !rt_started)
    {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
