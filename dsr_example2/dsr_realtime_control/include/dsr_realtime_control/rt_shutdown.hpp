#include "rclcpp/rclcpp.hpp"

#include "dsr_msgs2/srv/disconnect_rt_control.hpp"
#include "dsr_msgs2/srv/stop_rt_control.hpp"

#include <chrono>
#include <atomic>
#include <memory>
#include <thread>

std::atomic_bool rt_connected   =   true;
std::atomic_bool rt_started     =   true;

class RtShutdownNode : public rclcpp::Node
{
public:
    explicit RtShutdownNode();
    virtual ~RtShutdownNode();

    void DisconnectRtControlClient();
    void StopRtControlClient();

private:  
    std::thread client1_thread_;
    std::thread client2_thread_;

    rclcpp::Client<dsr_msgs2::srv::StopRtControl>::SharedPtr client1_;
    rclcpp::Client<dsr_msgs2::srv::DisconnectRtControl>::SharedPtr client2_;

};