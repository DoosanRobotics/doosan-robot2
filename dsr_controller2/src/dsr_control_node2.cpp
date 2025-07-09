/*
 * dsr_control_node2 
 * Author: Minsoo Song (minsoo.song@doosan.com)
 *
 * Copyright (c) 2024 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

#include <signal.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"  //ROS2
#include "rclcpp_components/register_node_macro.hpp"

#include <iostream>
#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>
// #include "dsr_hardware2/dsr_hw_interface2.h"

// extern void* get_s_node_();
// void* node_ptr = get_s_node_();
// rclcpp::Node::SharedPtr s_node_ = get_s_node_();
// rclcpp::Node::SharedPtr s_node_ = *static_cast<rclcpp::Node::SharedPtr*>(node_ptr);


class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        // 파라미터 선언
        declare_parameter("name", "dsr01");
        declare_parameter("rate", 100);
        declare_parameter("standby", 5000);
        declare_parameter("command", true);
        declare_parameter("host", "127.0.0.1");
        declare_parameter("port", 12345);
        declare_parameter("mode", "virtual");
        declare_parameter("model", "m1013");
        declare_parameter("gripper", "none");
        declare_parameter("mobile", "none");

        // 파라미터 읽기
        get_parameter("mobile", my_parameter_);
        RCLCPP_INFO(this->get_logger(), "mobile: %s", my_parameter_.c_str());

        timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&MyNode::saveParametersToYAML, this));
    }
private:
    std::string my_parameter_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 파라미터를 YAML 파일로 저장하는 메서드
    void saveParametersToYAML() {
        // 파라미터 서비스를 통해 현재 파라미터 값 가져오기
        YAML::Node yaml_node;
        yaml_node["name"] = get_parameter("name").as_string();
        yaml_node["rate"] = get_parameter("rate").as_int();
        yaml_node["standby"] = get_parameter("standby").as_int();
        yaml_node["command"] = get_parameter("command").as_bool();
        yaml_node["host"] = get_parameter("host").as_string();
        yaml_node["port"] = get_parameter("port").as_int();
        yaml_node["mode"] = get_parameter("mode").as_string();
        yaml_node["model"] = get_parameter("model").as_string();
        yaml_node["gripper"] = get_parameter("gripper").as_string();
        yaml_node["mobile"] = get_parameter("mobile").as_string();
        std::filesystem::path absolute_path = std::filesystem::absolute(__FILE__);
        // YAML 파일로 저장
        std::cout << "현재 실행 중인 파일의 절대 경로: " << absolute_path << std::endl;
    
        std::ofstream fout("parameters.yaml");
        fout << yaml_node;
        fout.close();

        RCLCPP_INFO(this->get_logger(), "Parameters saved to parameters.yaml");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
