#include "rclcpp/rclcpp.hpp"
#include "dsr_msgs2/srv/read_data_rt.hpp"
#include "dsr_msgs2/msg/torque_rt_stream.hpp"
#include "dsr_msgs2/msg/servol_rt_stream.hpp"
#include "dsr_msgs2/msg/servoj_rt_stream.hpp"

#include "dsr_realtime_control/rusage_utils.hpp"
#include "dsr_realtime_control/sched_utils.hpp"
#include "dsr_realtime_control/command_line_options.hpp"
#include "dsr_realtime_control/burn_cpu_cycles.hpp"

#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

#include "../../dsr_common2/include/DRFLEx.h"

typedef struct {
    /* timestamp at the data of data acquisition */
    double                      time_stamp;
    /* actual joint position from incremental encoder at motor side(used for control) [deg] */
    float                       actual_joint_position[NUMBER_OF_JOINT];
    /* actual joint position from absolute encoder at link side (used for exact link position) [deg] */
    float                       actual_joint_position_abs[NUMBER_OF_JOINT];
    /* actual joint velocity from incremental encoder at motor side [deg/s] */
    float                       actual_joint_velocity[NUMBER_OF_JOINT];
    /* actual joint velocity from absolute encoder at link side [deg/s] */
    float                       actual_joint_velocity_abs[NUMBER_OF_JOINT];
    /* actual robot tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] */
    float                       actual_tcp_position[NUMBER_OF_TASK];
    /* actual robot tcp velocity w.r.t. base coordinates [mm, deg/s] */
    float                       actual_tcp_velocity[NUMBER_OF_TASK];
    /* actual robot flange position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] */
    float                       actual_flange_position[NUMBER_OF_TASK];
    /* robot flange velocity w.r.t. base coordinates [mm, deg/s] */
    float                       actual_flange_velocity[NUMBER_OF_TASK];
    /* actual motor torque applying gear ratio = gear_ratio * current2torque_constant * motor current [Nm] */
    float                       actual_motor_torque[NUMBER_OF_JOINT];
    /* estimated joint torque by robot controller [Nm] */
    float                       actual_joint_torque[NUMBER_OF_JOINT];
    /* calibrated joint torque sensor data [Nm] */
    float                       raw_joint_torque[NUMBER_OF_JOINT];
    /* calibrated force torque sensor data w.r.t. flange coordinates [N, Nm] */
    float                       raw_force_torque[NUMBER_OF_JOINT];
    /* estimated external joint torque [Nm] */
    float                       external_joint_torque[NUMBER_OF_JOINT];
    /* estimated tcp force w.r.t. base coordinates [N, Nm] */
    float                       external_tcp_force[NUMBER_OF_TASK];
    /* target joint position [deg] */
    float                       target_joint_position[NUMBER_OF_JOINT];
    /* target joint velocity [deg/s] */
    float                       target_joint_velocity[NUMBER_OF_JOINT];
    /* target joint acceleration [deg/s^2] */
    float                       target_joint_acceleration[NUMBER_OF_JOINT];
    /* target motor torque [Nm] */
    float                       target_motor_torque[NUMBER_OF_JOINT];
    /* target tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] */
    float                       target_tcp_position[NUMBER_OF_TASK];
    /* target tcp velocity w.r.t. base coordinates [mm, deg/s] */
    float                       target_tcp_velocity[NUMBER_OF_TASK];
    /* jacobian matrix=J(q) w.r.t. base coordinates */
    float                       jacobian_matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT];
    /* gravity torque=g(q) [Nm] */
    float                       gravity_torque[NUMBER_OF_JOINT];
    /* coriolis matrix=C(q,q_dot)  */
    float                       coriolis_matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT];
    /* mass matrix=M(q) */
    float                       mass_matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT];
    /* robot configuration */
    unsigned short              solution_space;
    /* minimum singular value */
    float                       singularity;
    /* current operation speed rate(1~100 %) */
    float                       operation_speed_rate;
    /* joint temperature(celsius) */
    float                       joint_temperature[NUMBER_OF_JOINT];
    /* controller digital input(16 channel) */
    unsigned short              controller_digital_input;
    /* controller digital output(16 channel) */
    unsigned short              controller_digital_output;
    /* controller analog input type(2 channel) */
    unsigned char               controller_analog_input_type[2];
    /* controller analog input(2 channel) */
    float                       controller_analog_input[2];
    /* controller analog output type(2 channel) */
    unsigned char               controller_analog_output_type[2];
    /* controller analog output(2 channel) */
    float                       controller_analog_output[2];
    /* flange digital input(A-Series: 2 channel, M/H-Series: 6 channel) */
    unsigned char               flange_digital_input;
    /* flange digital output(A-Series: 2 channel, M/H-Series: 6 channel) */
    unsigned char               flange_digital_output;
    /* flange analog input(A-Series: 2 channel, M/H-Series: 4 channel) */
    float                       flange_analog_input[4];
    /* strobe count(increased by 1 when detecting setting edge) */
    unsigned char               external_encoder_strobe_count[2];
    /* external encoder count */
    unsigned int                external_encoder_count[2];
    /* final goal joint position (reserved) */
    float                       goal_joint_position[NUMBER_OF_JOINT];
    /* final goal tcp position (reserved) */
    float                       goal_tcp_position[NUMBER_OF_TASK];
    /* ROBOT_MODE_MANUAL(0), ROBOT_MODE_AUTONOMOUS(1), ROBOT_MODE_MEASURE(2) */
    unsigned char               robot_mode;
    /* STATE_INITIALIZING(0), STATE_STANDBY(1), STATE_MOVING(2), STATE_SAFE_OFF(3), STATE_TEACHING(4), STATE_SAFE_STOP(5), STATE_EMERGENCY_STOP, STATE_HOMMING, STATE_RECOVERY, STATE_SAFE_STOP2, STATE_SAFE_OFF2, */
    unsigned char               robot_state;
    /* position control mode, torque mode */
    unsigned short              control_mode;
    /* Reserved */
    unsigned char               reserved[256];
    //---------------------------------------------------------------------------------

} RT_STATE, *LPRT_STATE;

static ContextSwitchesCounter context_switches_counter(RUSAGE_THREAD);

class ReadDataRtNode : public rclcpp::Node
{
public:
    explicit ReadDataRtNode();
    virtual ~ReadDataRtNode();

    void ReadDataRtClient();

private:  
    rclcpp::Client<dsr_msgs2::srv::ReadDataRt>::SharedPtr client_;
    std::thread client_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr context_timer_;
};

class TorqueRtNode : public rclcpp::Node
{
public:
    explicit TorqueRtNode();
    virtual ~TorqueRtNode();

    void TorqueRtStreamPublisher();

private:  
    rclcpp::Publisher<dsr_msgs2::msg::TorqueRtStream>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr context_timer_;
    float q[NUMBER_OF_JOINT]={0,0,};
    float q_dot[NUMBER_OF_JOINT]={0,0,};
    float q_d[NUMBER_OF_JOINT]={0,0,90,0,90,0};
    float q_dot_d[NUMBER_OF_JOINT]={0,0,};
    float trq_g[NUMBER_OF_JOINT]={0,0,};
    float trq_d[NUMBER_OF_JOINT]={0,0,};

    float kp[NUMBER_OF_JOINT]={0.01,0.01,0.08,0.01,0.01,0.01};
    float kd[NUMBER_OF_JOINT]={0.00,0.00,0.00,0.00,0.00,0.00};
};

class ServojRtNode : public rclcpp::Node
{
public:
    explicit ServojRtNode();
    virtual ~ServojRtNode();

    void ServojRtStreamPublisher();

private:  
    rclcpp::Publisher<dsr_msgs2::msg::ServojRtStream>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr context_timer_;
    float pos_d[NUMBER_OF_JOINT]={0,0,};
    float vel_d[NUMBER_OF_JOINT]={0,0,};
    float acc_d[NUMBER_OF_JOINT]={0,0,};
    float time_d=0.0;
    
};

class ServolRtNode : public rclcpp::Node
{
public:
    explicit ServolRtNode();
    virtual ~ServolRtNode();

    void ServolRtStreamPublisher();

private:  
    rclcpp::Publisher<dsr_msgs2::msg::ServolRtStream>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr context_timer_;
    float pos_d[NUMBER_OF_JOINT]={0,0,};
    float vel_d[NUMBER_OF_JOINT]={0,0,};
    float acc_d[NUMBER_OF_JOINT]={0,0,};
    float time_d=0.0;
};