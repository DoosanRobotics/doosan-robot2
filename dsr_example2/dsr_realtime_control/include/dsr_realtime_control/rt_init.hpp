#include "rclcpp/rclcpp.hpp"

#include "dsr_msgs2/srv/connect_rt_control.hpp"
#include "dsr_msgs2/srv/set_rt_control_output.hpp"
#include "dsr_msgs2/srv/start_rt_control.hpp"

#include <chrono>
#include <atomic>
#include <memory>
#include <thread>

std::atomic_bool rt_connected   =   false;
std::atomic_bool rt_output_set  =   false;
std::atomic_bool rt_started     =   false;

class RtInitNode : public rclcpp::Node
{
public:
    explicit RtInitNode();
    virtual ~RtInitNode();

    void ConnectRtControlClient();
    void SetRtControlOutputClient();
    void StartRtControlClient();

private:  
    std::thread client1_thread_;
    std::thread client2_thread_;
    std::thread client3_thread_;

    rclcpp::Client<dsr_msgs2::srv::ConnectRtControl>::SharedPtr client1_;
    rclcpp::Client<dsr_msgs2::srv::SetRtControlOutput>::SharedPtr client2_;
    rclcpp::Client<dsr_msgs2::srv::StartRtControl>::SharedPtr client3_;
};