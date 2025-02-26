#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "dsr_realtime_control/realtime_api.hpp"

#include <pthread.h>
#include <string>

RT_STATE g_stRTState;
std::mutex mtx;
std::atomic_bool first_get(false);

// </-------------------- constant variables --------------------> //
Matrix6f J_m{
    {0.0004956, 0, 0, 0, 0, 0},
    {0, 0.0004956, 0, 0, 0, 0},
    {0, 0, 0.0001839, 0, 0, 0},
    {0, 0, 0, 0.00009901, 0, 0},
    {0, 0, 0, 0, 0.00009901, 0},
    {0, 0, 0, 0, 0, 0.00009901},
};
Matrix6f Gear_Ratio{
    {100, 0, 0, 0, 0, 0},
    {0, 100, 0, 0, 0, 0},
    {0, 0, 100, 0, 0, 0},
    {0, 0, 0, 80, 0, 0},
    {0, 0, 0, 0, 80, 0},
    {0, 0, 0, 0, 0, 80},
};

Matrix6f M_d{
    {2.5, 0, 0, 0, 0, 0},
    {0, 2.5, 0, 0, 0, 0},
    {0, 0, 2.5, 0, 0, 0},
    {0, 0, 0, 2.5, 0, 0},
    {0, 0, 0, 0, 2.5, 0},
    {0, 0, 0, 0, 0, 2.5},
};
Matrix6f D_d{
    {15, 0, 0, 0, 0, 0},
    {0, 15, 0, 0, 0, 0},
    {0, 0, 15, 0, 0, 0},
    {0, 0, 0, 15, 0, 0},
    {0, 0, 0, 0, 15, 0},
    {0, 0, 0, 0, 0, 15},
};
Matrix6f K_d{
    {100, 0, 0, 0, 0, 0},
    {0, 100, 0, 0, 0, 0},
    {0, 0, 100, 0, 0, 0},
    {0, 0, 0, 100, 0, 0},
    {0, 0, 0, 0, 100, 0},
    {0, 0, 0, 0, 0, 100},
};
Matrix6f K_o{
    {0.1, 0, 0, 0, 0, 0},
    {0, 0.1, 0, 0, 0, 0},
    {0, 0, 0.1, 0, 0, 0},
    {0, 0, 0, 0.1, 0, 0},
    {0, 0, 0, 0, 0.1, 0},
    {0, 0, 0, 0, 0, 0.1},
};

Vector6f deg_q_d{0,0,0,0,0,0};
Vector6f deg_q_dot_d{0,0,0,0,0,0};
Vector6f deg_q_ddot_d{0,0,0,0,0,0};

Vector6f q_d{0,0,0,0,0,0};
Vector6f q_dot_d{0,0,0,0,0,0};
Vector6f q_ddot_d{0,0,0,0,0,0};
// <-------------------- constant variables --------------------/> //

// </-------------------- update variables --------------------> //
Matrix6f M_hat;
Matrix6f C_hat;

Vector6f trq_g;
Vector6f trq_j;
Vector6f trq_m;
Vector6f trq_e;

Vector6f trq_c;
Vector6f trq_f_hat{0,0,0,0,0,0};

Vector6f trq;

Vector6f deg_q;
Vector6f deg_q_dot;

Vector6f q;
Vector6f q_dot;
Vector6f q_dot_prev{0,0,0,0,0,0};
Vector6f q_ddot;
// <-------------------- update variables --------------------/> //

using namespace DRAFramework;
CDRFLEx Drfl;

SetOnRtMonitoringDataNode::SetOnRtMonitoringDataNode() : Node("SetOnRtMonitoringData")
{
    Drfl.set_on_rt_monitoring_data(SetOnRtMonitoringDataNode::OnRtMonitoringData);
}

ReadDataRtNode::ReadDataRtNode() : Node("ReadDataRt")
{
    timer_      = this->create_wall_timer(std::chrono::microseconds(333),std::bind(&ReadDataRtNode::ReadDataRtAPI,this));
    
    // auto timer_callback = [this]() -> void 
    // {
    //     auto context_switches = context_switches_counter.get();
    //     if (context_switches > 0L) 
    //     {
    //       RCLCPP_WARN(this->get_logger(), "Involuntary context switches: '%lu'", context_switches);
    //     } 
    //     else 
    //     {
    //       RCLCPP_INFO(this->get_logger(), "Involuntary context switches: '%lu'", context_switches);
    //     }
    // };
    // context_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
}

TorqueRtNode::TorqueRtNode() : Node("TorqueRt")
{
    timer_      = this->create_wall_timer(std::chrono::microseconds(1000),std::bind(&TorqueRtNode::TorqueRtAPI,this));

    // auto timer_callback = [this]() -> void 
    // {
    //     auto context_switches = context_switches_counter.get();
    //     if (context_switches > 0L) 
    //     {
    //       RCLCPP_WARN(this->get_logger(), "Involuntary context switches: '%lu'", context_switches);
    //     } 
    //     else 
    //     {
    //       RCLCPP_INFO(this->get_logger(), "Involuntary context switches: '%lu'", context_switches);
    //     }
    // };
    // context_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
}

SetOnRtMonitoringDataNode::~SetOnRtMonitoringDataNode()
{
    RCLCPP_INFO(this->get_logger(), "SetOnRtMonitoringData shut down");
}
ReadDataRtNode::~ReadDataRtNode()
{
    RCLCPP_INFO(this->get_logger(), "ReadDataRt caller shut down");
}
TorqueRtNode::~TorqueRtNode()
{
    RCLCPP_INFO(this->get_logger(), "TorqueRt publisher shut down");
}

void SetOnRtMonitoringDataNode::OnRtMonitoringData(const LPRT_OUTPUT_DATA_LIST tData)
{
    g_stRTState.time_stamp = tData->time_stamp;
    
    for(int i = 0; i < 6; i++) {
        g_stRTState.actual_joint_position[i] = tData->actual_joint_position[i];
        g_stRTState.actual_joint_position_abs[i] = tData->actual_joint_position_abs[i];
        g_stRTState.actual_joint_velocity[i] = tData->actual_joint_velocity[i];
        g_stRTState.actual_joint_velocity_abs[i] = tData->actual_joint_velocity_abs[i];
        g_stRTState.actual_tcp_position[i] = tData->actual_tcp_position[i];
        g_stRTState.actual_tcp_velocity[i] = tData->actual_tcp_velocity[i];
        g_stRTState.actual_flange_position[i] = tData->actual_flange_position[i];
        g_stRTState.actual_flange_velocity[i] = tData->actual_flange_velocity[i];
        g_stRTState.actual_motor_torque[i] = tData->actual_motor_torque[i];
        g_stRTState.actual_joint_torque[i] = tData->actual_joint_torque[i];
        g_stRTState.raw_joint_torque[i] = tData->raw_joint_torque[i];
        g_stRTState.raw_force_torque[i] = tData->raw_force_torque[i];
        g_stRTState.external_joint_torque[i] = tData->external_joint_torque[i];
        g_stRTState.external_tcp_force[i] = tData->external_tcp_force[i];
        g_stRTState.target_joint_position[i] = tData->target_joint_position[i];
        g_stRTState.target_joint_velocity[i] = tData->target_joint_velocity[i];
        g_stRTState.target_joint_acceleration[i] = tData->target_joint_acceleration[i];
        g_stRTState.target_motor_torque[i] = tData->target_motor_torque[i];
        g_stRTState.target_tcp_position[i] = tData->target_tcp_position[i];
        g_stRTState.target_tcp_velocity[i] = tData->target_tcp_velocity[i];
        g_stRTState.gravity_torque[i] = tData->gravity_torque[i];
        g_stRTState.joint_temperature[i] = tData->joint_temperature[i];
        g_stRTState.goal_joint_position[i] = tData->goal_joint_position[i];
        g_stRTState.goal_tcp_position[i] = tData->goal_tcp_position[i];
        g_stRTState.goal_joint_position[i] = tData->goal_joint_position[i];
        g_stRTState.goal_tcp_position[i] = tData->goal_tcp_position[i];
    }

    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            g_stRTState.mass_matrix[i][j] = tData->mass_matrix[i][j];
            g_stRTState.coriolis_matrix[i][j] = tData->coriolis_matrix[i][j];
            g_stRTState.jacobian_matrix[i][j] = tData->jacobian_matrix[i][j];
        }
    }

    g_stRTState.solution_space = tData->solution_space;
    g_stRTState.singularity = tData->singularity;
    g_stRTState.operation_speed_rate = tData->operation_speed_rate;
    g_stRTState.controller_digital_input = tData->controller_digital_input;
    g_stRTState.controller_digital_output = tData->controller_digital_output;

    for(int i=0; i<2; i++){
        g_stRTState.controller_analog_input_type[i] = tData->controller_analog_input_type[i];
        g_stRTState.controller_analog_input[i] = tData->controller_analog_input[i];
        g_stRTState.controller_analog_output_type[i] = tData->controller_analog_output_type[i];
        g_stRTState.controller_analog_output[i] = tData->controller_analog_output[i];
        g_stRTState.external_encoder_strobe_count[i] = tData->external_encoder_strobe_count[i];
        g_stRTState.external_encoder_count[i] = tData->external_encoder_count[i];
    }

    g_stRTState.flange_digital_input = tData->flange_digital_input;
    g_stRTState.flange_digital_output = tData->flange_digital_output;

    for(int i=0; i<4; i++){
        g_stRTState.flange_analog_input[i] = tData->flange_analog_input[i];
    }
    g_stRTState.robot_mode = tData->robot_mode;
    g_stRTState.robot_state = tData->robot_state;
    g_stRTState.control_mode = tData->control_mode;
    
    for(int i=0; i<6; i++)
    {
        g_stRTState.actual_joint_position[i] = tData->actual_joint_position[i];
        g_stRTState.actual_joint_velocity[i] = tData->actual_joint_velocity[i];
        g_stRTState.gravity_torque[i] = tData->gravity_torque[i];
    }
    RCLCPP_INFO(rclcpp::get_logger("OnRTMonitoringData"),"time_stamp:%f",tData->time_stamp);
}

void ReadDataRtNode::ReadDataRtAPI()
{
    // RCLCPP_INFO(this->get_logger(), "call ReadDataRt API");
    LPRT_OUTPUT_DATA_LIST tData = Drfl.read_data_rt();
    // RCLCPP_INFO(this->get_logger(), "Received ReadDataRt");

    g_stRTState.time_stamp=tData->time_stamp;
    for(int i=0; i<6; i++)
    {
        trq_g(i) = tData->gravity_torque[i];
        trq_j(i) = tData->actual_joint_torque[i];
        trq_m(i) = tData->actual_motor_torque[i];
        trq_e(i) = tData->external_joint_torque [i];

        deg_q(i) = tData->actual_joint_position_abs[i];
        deg_q_dot(i) = tData->actual_joint_velocity_abs[i];
        // RCLCPP_INFO(this->get_logger(), "[0]%f[1]%f[2]%f[3]%f[4]%f[5]%f",trq_e[0],trq_e[1],trq_e[2],trq_e[3],trq_e[4],trq_e[5]);    
    }
    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            M_hat(i,j) = tData->mass_matrix[i][j];
            C_hat(i,j) = tData->coriolis_matrix[i][j];
        }
    }

    if(!first_get)
    {
        first_get=true;
        RCLCPP_INFO(this->get_logger(), "data updated");        
    }
    // RCLCPP_INFO(this->get_logger(), "g_stRTState updated about %f",tData->time_stamp);
}

void TorqueRtNode::TorqueRtAPI()
{
    if(!first_get)
    {
        RCLCPP_INFO(this->get_logger(), "data not updated yet");        
        return;
    }
    trq = TorqueRtNode::GravityCompensation();
    // trq = TorqueRtNode::ExternalForceResist();
    
    for(int i=0; i<6; i++)
    {
        trq_d[i] = trq(i);  
    }
    Drfl.torque_rt(trq_d,0);
    RCLCPP_INFO(this->get_logger(), "trq_d[0]%f[1]%f[2]%f[3]%f[4]%f[5]%f",trq_d[0],trq_d[1],trq_d[2],trq_d[3],trq_d[4],trq_d[5]);
}
Vector6f TorqueRtNode::GravityCompensation()
{
    mtx.lock();
    //control input
    trq_c = trq_g;
    mtx.unlock();

    return trq_c;
}
Vector6f TorqueRtNode::ExternalForceResist()
{
    mtx.lock();
    //control input
    trq_c = trq_j;
    mtx.unlock();

    return trq_c;
}

int main(int argc, char **argv)
{
    // --------------------cpu affinity set-------------------- //

    // Pin the main thread to CPU 3
    // int cpu_id = std::thread::hardware_concurrency()-1;
    // cpu_set_t cpuset;
    // CPU_ZERO(&cpuset);
    // CPU_SET(cpu_id, &cpuset);
    // Pin the main thread to CPU 3 //

    // Pin the main thread to CPUs 2 and 3
    uint32_t cpu_bit_mask = 0b1100;
    cpu_set_t cpuset;
    uint32_t cpu_cnt = 0U;
    CPU_ZERO(&cpuset);
    while (cpu_bit_mask > 0U) 
    {
        if ((cpu_bit_mask & 0x1U) > 0) 
        {
        CPU_SET(cpu_cnt, &cpuset);
        }
        cpu_bit_mask = (cpu_bit_mask >> 1U);
        cpu_cnt++;
    }
    auto ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (ret>0)
    {
        std::cerr << "Couldn't set CPU affinity. Error code" << strerror(errno) << std::endl;
        return EXIT_FAILURE;
    }
    ret = pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (ret<0)
    {
        std::cerr << "Coudln't get CPU affinity. Error code" << strerror(errno) << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Pinned CPUs:"<< std::endl;
    for (int i=0; i < CPU_SETSIZE; i++)
    {
        if(CPU_ISSET(i,&cpuset))
        {
            std::cout << "  CPU" << std::to_string(i) << std::endl;
        }
    }
    // Pin the main thread to CPUs 2 and 3 //

    // -------------------- cpu affinity set    -------------------- //

    // -------------------- process scheduling  -------------------- //
    auto options_reader = SchedOptionsReader();
    if (!options_reader.read_options(argc, argv)) 
    {
        options_reader.print_usage();
        return 0;
    }
    auto options = options_reader.get_options();

    // -------------------- RT Initalize -------------------- // 
    assert(Drfl.connect_rt_control("192.168.137.100",12347));
    Drfl.set_rt_control_output("v1.0",0.001,4);
    Drfl.start_rt_control();
    // -------------------- RT Initalize -------------------- // 

    // </-------------------- realtime executor scheduling -------------------- > //
    rclcpp::init(argc,argv);

    // </---------- if you want to acquire data using SetOnRtMonitoringData API, Uncomment this block ---------- > 
    // auto node = std::make_shared<SetOnRtMonitoringDataNode>();
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node);
    // auto executor_thread  = std::thread([&](){executor.spin();});
    // set_thread_scheduling(executor_thread.native_handle(), options.policy, options.priority);
    // executor_thread.join();
    // < ---------- if you want to acquire data using SetOnRtMonitoringData API, Uncomment this block ----------/>

    auto node1= std::make_shared<ReadDataRtNode>();
    rclcpp::executors::SingleThreadedExecutor executor1;
    executor1.add_node(node1);
    auto executor1_thread = std::thread([&](){executor1.spin();});
    set_thread_scheduling(executor1_thread.native_handle(), options.policy, options.priority);

    auto node2= std::make_shared<TorqueRtNode>();
    rclcpp::executors::SingleThreadedExecutor executor2;
    executor2.add_node(node2);
    auto executor2_thread = std::thread([&](){executor2.spin();});
    set_thread_scheduling(executor2_thread.native_handle(), options.policy, options.priority);
    
    // executor_thread.join();
    executor1_thread.join();
    executor2_thread.join();
    rclcpp::shutdown();
    // < -------------------- realtime executor scheduling --------------------/> //

    // </-------------------- RT Shutdown -------------------- > // 
    Drfl.stop_rt_control();
    Drfl.disconnect_rt_control();
    // < -------------------- RT Shutdown --------------------/> // 
    return 0;
}
            
// </----------scheduling command example---------- >//
// $ ros2 run dsr_realtime_control realtime_control --sched SCHED_FIFO --priority 80
// $ ros2 run dsr_realtime_control realtime_control --sched SCHED_RR --priority 80
// $ ps -C realtime_control -L -o tid,comm,rtprio,cls,psr
// < ----------scheduling command example----------/>//