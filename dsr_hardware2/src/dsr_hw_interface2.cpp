// /*
//  *  Inferfaces for doosan robot controllor 
//   * Author: Minsoo Song(minsoo.song@doosan.com)
//  *
//  * Copyright (c) 2024 Doosan Robotics
//  * Use of this source code is governed by the BSD, see LICENSE
// */

#include "dsr_hardware2/dsr_hw_interface2.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

// #include "dsr_hardware2/dsr_connection_node2.h"
#include <boost/thread/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <iostream>
#include <chrono>

#include <unistd.h>     
#include <math.h>
#include "../../common2/include/DRFLEx.h"
using namespace std;
using namespace chrono;
using namespace DRAFramework;
rclcpp::Node::SharedPtr s_node_ = nullptr;

CDRFLEx Drfl;
//TODO Serial_comm ser_comm;

bool g_bIsEmulatorMode = FALSE;
bool g_bHasControlAuthority = FALSE;
bool g_bTpInitailizingComplted = FALSE;
bool g_bHommingCompleted = FALSE;

DR_STATE    g_stDrState;
DR_ERROR    g_stDrError;

int m_nVersionDRCF;
bool init_state=TRUE;


int nDelay = 5000;
#define STABLE_BAND_JNT     0.05

void* get_drfl(){
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[DRFL address] %p", &Drfl);
    return &Drfl;
}
void* get_s_node_(){
    return &s_node_;
}
auto startTime = steady_clock::now(); // 프로그램 시작 시간 기록
bool init_check = true;

void threadFunction() {
    s_node_ = rclcpp::Node::make_shared("dsr_hw_interface2");
    
    std::string param_name = std::string(s_node_->get_namespace()) + "_parameters.yaml";
    std::string package_directory = ament_index_cpp::get_package_share_directory("dsr_hardware2");
    std::string yaml_file_path = package_directory + "/config" + param_name;

    std::ifstream fin(yaml_file_path);
    if (!fin) {
        RCLCPP_ERROR(s_node_->get_logger(), "Failed to open YAML file: %s", yaml_file_path.c_str());
        return;
    }

    // YAML 파일 파싱
    YAML::Node yaml_node = YAML::Load(fin);
    fin.close();
    
    // 파싱된 YAML 노드에서 파라미터 읽기
    if (yaml_node["name"]) {
        m_name = yaml_node["name"].as<std::string>();
        RCLCPP_INFO(s_node_->get_logger(), "name: %s", m_name.c_str());
    }
    if (yaml_node["rate"]) {
        m_rate = yaml_node["rate"].as<int>();
        RCLCPP_INFO(s_node_->get_logger(), "rate: %d", m_rate);
    }
    if (yaml_node["standby"]) {
        m_standby = yaml_node["standby"].as<int>();
        RCLCPP_INFO(s_node_->get_logger(), "standby: %d", m_standby);
    }
    if (yaml_node["command"]) {
        m_command = yaml_node["command"].as<bool>();
        RCLCPP_INFO(s_node_->get_logger(), "command: %s", m_command ? "true" : "false");
    }
    if (yaml_node["host"]) {
        m_host = yaml_node["host"].as<std::string>();
        RCLCPP_INFO(s_node_->get_logger(), "host: %s", m_host.c_str());
    }
    if (yaml_node["port"]) {
        m_port = yaml_node["port"].as<int>();
        RCLCPP_INFO(s_node_->get_logger(), "port: %d", m_port);
    }
    if (yaml_node["mode"]) {
        m_mode = yaml_node["mode"].as<std::string>();
        RCLCPP_INFO(s_node_->get_logger(), "mode: %s", m_mode.c_str());
    }
    if (yaml_node["model"]) {
        m_model = yaml_node["model"].as<std::string>();
        RCLCPP_INFO(s_node_->get_logger(), "model: %s", m_model.c_str());
    }
    if (yaml_node["gripper"]) {
        m_gripper = yaml_node["gripper"].as<std::string>();
        RCLCPP_INFO(s_node_->get_logger(), "gripper: %s", m_gripper.c_str());
    }
    if (yaml_node["mobile"]) {
        m_mobile = yaml_node["mobile"].as<std::string>();
        RCLCPP_INFO(s_node_->get_logger(), "mobile: %s", m_mobile.c_str());
    }
}


namespace dsr_hardware2{


CallbackReturn DRHWInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    // TODO(leeminju531) Workaround sleep. replace with the event come out from drcf.
    sleep(8);

    // robot has 6 joints and 2 interfaces
    joint_position_.assign(6, 0);
    joint_velocities_.assign(6, 0);
    joint_position_command_.assign(6, 0);
    joint_velocities_command_.assign(6, 0);

    if(6 != info_.joints.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), 
            "[on_init] Hardware joint size : %zu, expected : 6", info.joints.size());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), 
            "[on_init] Hardware name : %s, type : %s, class type : %s",
            info_.name.c_str(), info_.type.c_str(), info_.hardware_class_type.c_str());

    for (const auto & joint : info_.joints)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("dsr_hw_interface2"), 
            "[on_init] joint name : %s, type : %s,",
            joint.name.c_str(), joint.type.c_str());
        for (const auto & interface : joint.state_interfaces)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("dsr_hw_interface2"), 
                "[on_init] joint state interface name : %s ", 
                interface.name.c_str());
						if(interface.name == "effort") {
							RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), 
								"[on_init] Not Implemented effort interface.. ignored");
							continue;
						}
            joint_interfaces[interface.name].push_back(joint.name);
        }
        for (const auto & interface : joint.command_interfaces)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("dsr_hw_interface2"), 
                "[on_init] joint command_interfaces name : %s ", 
                interface.name.c_str());
							if(interface.name == "effort") {
								RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), 
										"[on_init] Not Implemented effort interface.. ignored");
								continue;
							}
            joint_comm_interfaces[interface.name].push_back(joint.name);
        }
    }

    // TODO(Song-ms, leeminju): This thread is present for parameter reading... 
    //? Parameter concept is proper for hardware interface ? (Interface doesn't inherit node.
    std::thread t(threadFunction);
    t.join(); // need to make sure termination of the thread.

//-----------------------------------------------------------------------------------------------------
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    INITAILIZE");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");
    //--- doosan API's call-back fuctions : Only work within 50msec in call-back functions
    Drfl.set_on_tp_initializing_completed(DSRInterface::OnTpInitializingCompletedCB);
    Drfl.set_on_homming_completed(DSRInterface::OnHommingCompletedCB);
    Drfl.set_on_program_stopped(DSRInterface::OnProgramStoppedCB);
    Drfl.set_on_monitoring_modbus(DSRInterface::OnMonitoringModbusCB);
    Drfl.set_on_monitoring_data(DSRInterface::OnMonitoringDataCB);           // Callback function in M2.4 and earlier
    Drfl.set_on_monitoring_ctrl_io(DSRInterface::OnMonitoringCtrlIOCB);       // Callback function in M2.4 and earlier
    Drfl.set_on_monitoring_state(DSRInterface::OnMonitoringStateCB);
    Drfl.set_on_monitoring_access_control(DSRInterface::OnMonitoringAccessControlCB);
    Drfl.set_on_log_alarm(DSRInterface::OnLogAlarm);
    
    //------------------------------------------------------------------------------
    // await for values from ros parameters
    while(m_host == "") //! attached handling with above thread.
    {
        usleep(nDelay);
    }

    if(!Drfl.open_connection(m_host, m_port))
    {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"),"    DSRInterface::init() DRCF connecting ERROR!!!");
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n"); 
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    OPEN CONNECTION");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");   

    //--- connect Emulator ? ------------------------------    
    if(m_host == "127.0.0.1") g_bIsEmulatorMode = true; 
    else                    g_bIsEmulatorMode = false;

    //--- Get version -------------------------------------            
    SYSTEM_VERSION tSysVerion = {'\0', };
    assert(Drfl.get_system_version(&tSysVerion));

    //--- Get DRCF version & convert to integer  ----------            
    m_nVersionDRCF = 0; 
    int k=0;
    for(int i=strlen(tSysVerion._szController); i>0; i--)
        if(tSysVerion._szController[i]>='0' && tSysVerion._szController[i]<='9')
            m_nVersionDRCF += (tSysVerion._szController[i]-'0')*pow(10.0,k++);
    if(m_nVersionDRCF < 100000) m_nVersionDRCF += 100000; 

    if(g_bIsEmulatorMode) RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Emulator Mode");
    else                  RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Real Robot Mode");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    DRCF version = %s",tSysVerion._szController);
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    DRFL version = %s",Drfl.get_library_version());
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    m_nVersionDRCF = %d", m_nVersionDRCF);  //ex> M2.40 = 120400, M2.50 = 120500  
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");   

        if(m_nVersionDRCF >= 120500 && m_nVersionDRCF < 3000000)    //M2.5 or later        
        {
            Drfl.set_on_monitoring_data_ex(DSRInterface::OnMonitoringDataExCB);      //Callback function in version 2.5 and higher
            // Drfl.set_on_monitoring_ctrl_io_ex(DSRInterface::OnMonitoringCtrlIOExCB);  //Callback function in version 2.5 and higher                     
            Drfl.setup_monitoring_version(1);                        //Enabling extended monitoring functions 
        }
        else if(m_nVersionDRCF >= 3000000)
        {
            Drfl.set_on_monitoring_data_ex(DSRInterface::OnMonitoringDataExCB);      //Callback function in version 2.5 and higher
            // Drfl.set_on_monitoring_ctrl_io_ex(DSRInterface::OnMonitoringCtrlIOEx2CB);  //Callback function in version 2.5 and higher                     
            Drfl.setup_monitoring_version(1);                        //Enabling extended monitoring functions 
        }

    //--- Check Robot State : STATE_STANDBY ---               
    while ((Drfl.GetRobotState() != STATE_STANDBY)){
        usleep(nDelay);
    }

    //--- Set Robot mode : MANUAL or AUTO
    if(!Drfl.SetRobotMode(ROBOT_MODE_AUTONOMOUS)) {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "ROBOT_MODE_AUTONOMOUS Setting Failure !!"); 
        return CallbackReturn::ERROR;
    }

    //--- Set Robot mode : virual or real 
    ROBOT_SYSTEM eTargetSystem = ROBOT_SYSTEM_VIRTUAL;
    if(m_mode == "real") eTargetSystem = ROBOT_SYSTEM_REAL;
    if(!Drfl.SetRobotSystem(eTargetSystem)) {
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"), "SetRobotSystem {%s} Setting Failure !!",
            m_mode.c_str()); 
        return CallbackReturn::ERROR;
    }

    Drfl.set_auto_servo_off(0, 5.0);
    // Drfl.connect_rt_control();
    // string version = "v1.0";
    // float period = 0.001; // 1 msec
    // int losscount = 4;
    // Drfl.set_rt_control_output(version, period, losscount);
    // usleep(nDelay);
    // Drfl.start_rt_control();

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DRHWInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
	
	for(size_t i=0; i<joint_interfaces["position"].size(); i++) {
		state_interfaces.emplace_back(joint_interfaces["position"][i], "position", &joint_position_[i]);
	}
	// TODO(songms, leeminju) support velocity control.
    for(size_t i=0; i<joint_interfaces["velocity"].size(); i++) {
		state_interfaces.emplace_back(joint_interfaces["velocity"][i], "velocity", &joint_velocities_[i]);
	}
	// TODO(songms, leeminju) support effort control.
	for(size_t i=0; i<joint_interfaces["effort"].size(); i++) {
		state_interfaces.emplace_back(joint_interfaces["effort"][i], "effort", &joint_effort_[i]);
	}
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DRHWInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
    pre_joint_position_command_ = joint_position_command_;
	for(size_t i=0; i<joint_comm_interfaces["position"].size(); i++) {
		command_interfaces.emplace_back(joint_comm_interfaces["position"][i], "position", &joint_position_command_[i]);
	}
	for(size_t i=0; i<joint_comm_interfaces["velocity"].size(); i++) {
		command_interfaces.emplace_back(joint_comm_interfaces["velocity"][i], "velocity", &joint_velocities_command_[i]);
	}
	// TODO(songms, leeminju) support effort control.
	for(size_t i=0; i<joint_comm_interfaces["effort"].size(); i++) {
		command_interfaces.emplace_back(joint_comm_interfaces["effort"][i], "effort", &joint_effort_command_[i]);
	}
  return command_interfaces;
}


return_type DRHWInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

    LPROBOT_POSE pose = Drfl.GetCurrentPose();
    if(nullptr == pose) {
      RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"),
				"[read] GetCurrentPose retrieves nullptr");
      return return_type::ERROR; //? what effection of this to control node 
    }
    for(int i=0;i<6;i++){
      joint_position_[i] = deg2rad(pose->_fPosition[i]);
    //   RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), " [READ] joint_pos[%d] : %.3f ", i, joint_position_[i]);
    }
  return return_type::OK;
}


std::vector<float> previous_joint_position_command_float(6, std::numeric_limits<float>::quiet_NaN()); // 초기값 NaN

bool positionCommandRunning(const std::vector<double>& lhs, const std::vector<double>& rhs) {
    double var = 0;
    for(int i=0; i<lhs.size(); i++) {
        var += abs(lhs[i] - rhs[i]);
    }
    return var >= 0.0001;
}

vector<vector<float>> joint_position_commands;
bool done = false;
return_type DRHWInterface::write(const rclcpp::Time &, const rclcpp::Duration &dt)
{
    if(positionCommandRunning(pre_joint_position_command_, joint_position_command_)) {
        // RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"), "Updated ");
        Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS,SAFETY_MODE_EVENT_MOVE);
        float pos[6];
        float limitVel[6] = {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};
        float limitAcc[6] = {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};
        for(int i=0;i<6;i++) {
            pos[i] = static_cast<float>(joint_position_command_[i]* (180.0f / M_PI));
        }
        // RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "[WRITE] pos  : {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f}"
        //     ,pos[0] * ( M_PI / 180.0f)
        //     ,pos[1] * ( M_PI / 180.0f)
        //     ,pos[2] * ( M_PI / 180.0f)
        //     ,pos[3] * ( M_PI / 180.0f)
        //     ,pos[4] * ( M_PI / 180.0f)
        //     ,pos[5] * ( M_PI / 180.0f)
        // );
        if(joint_position_commands.size() < 3) {
            joint_position_commands.emplace_back(std::vector<float> {pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]});
            pre_joint_position_command_ = joint_position_command_;
            done = true;
            return return_type::OK;
        }
        else{
            if(joint_position_commands.size() == 3 && done) {
                
                for(auto cmd : joint_position_commands) {
                    // RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"), "[WRITE] cmd  : {%.3f, %.3f, %.3f, %.3f, %.3f, %.3f}"
                    //     ,cmd[0] * ( M_PI / 180.0f)
                    //     ,cmd[1] * ( M_PI / 180.0f)
                    //     ,cmd[2] * ( M_PI / 180.0f)
                    //     ,cmd[3] * ( M_PI / 180.0f)
                    //     ,cmd[4] * ( M_PI / 180.0f)
                    //     ,cmd[5] * ( M_PI / 180.0f)
                    // );
                    if(m_nVersionDRCF >= 3000000) {
                        Drfl.amovej(pos, limitVel, limitVel); // Workaround. needed updated.
                    }
                    else {
                        // Drfl.servoj(cmd.data(), limitVel, limitAcc, float(dt.seconds())*1.7, DR_SERVO_OVERRIDE);
                        Drfl.servoj(cmd.data(), limitVel, limitAcc, float(dt.seconds()), DR_SERVO_QUEUE);
                    }
                    this_thread::sleep_for(chrono::milliseconds(1));
                    done = false;
                }
            }
            if(m_nVersionDRCF >= 3000000) {
                Drfl.amovej(pos, limitVel, limitVel); // Workaround. needed updated.
            }
            else {
                // Drfl.servoj(cmd.data(), limitVel, limitAcc, float(dt.seconds())*1.7, DR_SERVO_OVERRIDE);
                Drfl.servoj(pos, limitVel, limitAcc, float(dt.seconds()), DR_SERVO_QUEUE);
            }
        }
        pre_joint_position_command_ = joint_position_command_;
        return return_type::OK;
    }

    joint_position_commands.clear();
    done = false;
    // RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"), "NOT Updated ");
    pre_joint_position_command_ = joint_position_command_;
    return return_type::OK;
}

DRHWInterface::~DRHWInterface()
{
    Drfl.close_connection();

    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n"); 
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    CONNECTION IS CLOSED");
    RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n"); 
}

}  


const char* GetRobotStateString(int nState)
{
    switch(nState)
    {
    case STATE_INITIALIZING:    return "(0) INITIALIZING";
    case STATE_STANDBY:         return "(1) STANDBY";
    case STATE_MOVING:          return "(2) MOVING";
    case STATE_SAFE_OFF:        return "(3) SAFE_OFF";
    case STATE_TEACHING:        return "(4) TEACHING";
    case STATE_SAFE_STOP:       return "(5) SAFE_STOP";
    case STATE_EMERGENCY_STOP:  return "(6) EMERGENCY_STOP";
    case STATE_HOMMING:         return "(7) HOMMING";
    case STATE_RECOVERY:        return "(8) RECOVERY";
    case STATE_SAFE_STOP2:      return "(9) SAFE_STOP2";
    case STATE_SAFE_OFF2:       return "(10) SAFE_OFF2";
    case STATE_RESERVED1:       return "(11) RESERVED1";
    case STATE_RESERVED2:       return "(12) RESERVED2";
    case STATE_RESERVED3:       return "(13) RESERVED3";
    case STATE_RESERVED4:       return "(14) RESERVED4";
    case STATE_NOT_READY:       return "(15) NOT_READY";

    default:                  return "UNKNOWN";
    }
    return "UNKNOWN";
}

int IsInposition(double dCurPosDeg[], double dCmdPosDeg[])
{
    int cnt=0;
    double dError[NUM_JOINT] ={0.0, };

    for(int i=0;i<NUM_JOINT;i++)
    {
        dError[i] = dCurPosDeg[i] - dCmdPosDeg[i];
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    <inpos> %f = %f -%f",dError[i], dCurPosDeg[i], dCmdPosDeg[i]);
        if(fabs(dError[i]) < STABLE_BAND_JNT)
            cnt++;
    }
    if(NUM_JOINT == cnt)
        return true;
    else 
        return false;
}

//----- register the call-back functions ----------------------------------------
void DSRInterface::OnTpInitializingCompletedCB()
{
    // request control authority after TP initialized
    // cout << "[callback OnTpInitializingCompletedCB] tp initializing completed" << endl;
    g_bTpInitailizingComplted = TRUE;
    //Drfl.manage_access_control(MANAGE_ACCESS_CONTROL_REQUEST);
    Drfl.manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);

    g_stDrState.bTpInitialized = TRUE;
}


void DSRInterface::OnHommingCompletedCB()
{
    g_bHommingCompleted = TRUE;
    // Only work within 50msec
    // cout << "[callback OnHommingCompletedCB] homming completed" << endl;

    g_stDrState.bHommingCompleted = TRUE;
}

void DSRInterface::OnProgramStoppedCB(const PROGRAM_STOP_CAUSE iStopCause)
{
    // cout << "[callback OnProgramStoppedCB] Program Stop: " << (int)iStopCause << endl;
    g_stDrState.bDrlStopped = TRUE;
}
// M2.4 or lower
void DSRInterface::OnMonitoringCtrlIOCB (const LPMONITORING_CTRLIO pCtrlIO)
{
    for (int i = 0; i < NUM_DIGITAL; i++){
        if(pCtrlIO){  
            g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];  
            g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];  
        }
    }
}

// M3.0 or higher
// void DSRInterface::OnMonitoringCtrlIOEx2CB (const LPMONITORING_CTRLIO_EX2 pCtrlIO) 
// {
//     //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DSRInterface::OnMonitoringCtrlIOExCB");

//     for (int i = 0; i < NUM_DIGITAL; i++){
//         if(pCtrlIO){  
//             g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];  
//             g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];  
//         }
//     }

//     //----- In M3.0 version or higher The following variables were added -----
//     for (int i = 0; i < 3; i++)
//         g_stDrState.bActualSW[i] = pCtrlIO->_tInput._iActualSW[i];

//     for (int i = 0; i < 2; i++){
//         g_stDrState.bActualSI[i] = pCtrlIO->_tInput._iActualSI[i];
//         g_stDrState.fActualAI[i] = pCtrlIO->_tInput._fActualAI[i];
//         g_stDrState.iActualAT[i] = pCtrlIO->_tInput._iActualAT[i];
//         g_stDrState.fTargetAO[i] = pCtrlIO->_tOutput._fTargetAO[i];
//         g_stDrState.iTargetAT[i] = pCtrlIO->_tOutput._iTargetAT[i];
//         g_stDrState.bActualES[i] = pCtrlIO->_tEncoder._iActualES[i];
//         g_stDrState.iActualED[i] = pCtrlIO->_tEncoder._iActualED[i];
//         g_stDrState.bActualER[i] = pCtrlIO->_tEncoder._iActualER[i];
//     }  
//     //-------------------------------------------------------------------------
// }

// M2.5 or higher
void DSRInterface::OnMonitoringCtrlIOExCB (const LPMONITORING_CTRLIO_EX pCtrlIO) 
{
    //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DSRInterface::OnMonitoringCtrlIOExCB");

    for (int i = 0; i < NUM_DIGITAL; i++){
        if(pCtrlIO){  
            g_stDrState.bCtrlBoxDigitalOutput[i] = pCtrlIO->_tOutput._iTargetDO[i];  
            g_stDrState.bCtrlBoxDigitalInput[i]  = pCtrlIO->_tInput._iActualDI[i];  
        }
    }

    //----- In M2.5 version or higher The following variables were added -----
    for (int i = 0; i < 3; i++)
        g_stDrState.bActualSW[i] = pCtrlIO->_tInput._iActualSW[i];

    for (int i = 0; i < 2; i++){
        g_stDrState.bActualSI[i] = pCtrlIO->_tInput._iActualSI[i];
        g_stDrState.fActualAI[i] = pCtrlIO->_tInput._fActualAI[i];
        g_stDrState.iActualAT[i] = pCtrlIO->_tInput._iActualAT[i];
        g_stDrState.fTargetAO[i] = pCtrlIO->_tOutput._fTargetAO[i];
        g_stDrState.iTargetAT[i] = pCtrlIO->_tOutput._iTargetAT[i];
        g_stDrState.bActualES[i] = pCtrlIO->_tEncoder._iActualES[i];
        g_stDrState.iActualED[i] = pCtrlIO->_tEncoder._iActualED[i];
        g_stDrState.bActualER[i] = pCtrlIO->_tEncoder._iActualER[i];
    }  
    //-------------------------------------------------------------------------
}

// M2.4 or lower
void DSRInterface::OnMonitoringDataCB(const LPMONITORING_DATA pData)
{
    // This function is called every 100 msec
    // Only work within 50msec
    //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DSRInterface::OnMonitoringDataCB");

    g_stDrState.nActualMode  = pData->_tCtrl._tState._iActualMode;                  // position control: 0, torque control: 1 ?????
    g_stDrState.nActualSpace = pData->_tCtrl._tState._iActualSpace;                 // joint space: 0, task space: 1    

    for (int i = 0; i < NUM_JOINT; i++){
        if(pData){  
            // joint         
            g_stDrState.fCurrentPosj[i] = pData->_tCtrl._tJoint._fActualPos[i];     // Position Actual Value in INC     
            g_stDrState.fCurrentVelj[i] = pData->_tCtrl._tJoint._fActualVel[i];     // Velocity Actual Value
            g_stDrState.fJointAbs[i]    = pData->_tCtrl._tJoint._fActualAbs[i];     // Position Actual Value in ABS
            g_stDrState.fJointErr[i]    = pData->_tCtrl._tJoint._fActualErr[i];     // Joint Error
            g_stDrState.fTargetPosj[i]  = pData->_tCtrl._tJoint._fTargetPos[i];     // Target Position
            g_stDrState.fTargetVelj[i]  = pData->_tCtrl._tJoint._fTargetVel[i];     // Target Velocity
            // task
            g_stDrState.fCurrentPosx[i]     = pData->_tCtrl._tTask._fActualPos[0][i];   //????? <---------이것 2개다 확인할 것  
            g_stDrState.fCurrentToolPosx[i] = pData->_tCtrl._tTask._fActualPos[1][i];   //????? <---------이것 2개다 확인할 것  
            g_stDrState.fCurrentVelx[i] = pData->_tCtrl._tTask._fActualVel[i];      // Velocity Actual Value
            g_stDrState.fTaskErr[i]     = pData->_tCtrl._tTask._fActualErr[i];      // Task Error
            g_stDrState.fTargetPosx[i]  = pData->_tCtrl._tTask._fTargetPos[i];      // Target Position
            g_stDrState.fTargetVelx[i]  = pData->_tCtrl._tTask._fTargetVel[i];      // Target Velocity
            // Torque
            g_stDrState.fDynamicTor[i]  = pData->_tCtrl._tTorque._fDynamicTor[i];   // Dynamics Torque
            g_stDrState.fActualJTS[i]   = pData->_tCtrl._tTorque._fActualJTS[i];    // Joint Torque Sensor Value
            g_stDrState.fActualEJT[i]   = pData->_tCtrl._tTorque._fActualEJT[i];    // External Joint Torque
            g_stDrState.fActualETT[i]   = pData->_tCtrl._tTorque._fActualETT[i];    // External Task Force/Torque

            g_stDrState.nActualBK[i]    = pData->_tMisc._iActualBK[i];              // brake state     
            g_stDrState.fActualMC[i]    = pData->_tMisc._fActualMC[i];              // motor input current
            g_stDrState.fActualMT[i]    = pData->_tMisc._fActualMT[i];              // motor current temperature
        }
    }
    g_stDrState.nSolutionSpace  = pData->_tCtrl._tTask._iSolutionSpace;             // Solution Space
    g_stDrState.dSyncTime       = pData->_tMisc._dSyncTime;                         // inner clock counter  

    for (int i = 5; i < NUM_BUTTON; i++){
        if(pData){
            g_stDrState.nActualBT[i]    = pData->_tMisc._iActualBT[i];              // robot button state
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(pData){
                g_stDrState.fRotationMatrix[j][i] = pData->_tCtrl._tTask._fRotationMatrix[j][i];    // Rotation Matrix
            }
        }
    }

    for (int i = 0; i < NUM_FLANGE_IO; i++){
        if(pData){
            g_stDrState.bFlangeDigitalInput[i]  = pData->_tMisc._iActualDI[i];      // Digital Input data             
            g_stDrState.bFlangeDigitalOutput[i] = pData->_tMisc._iActualDO[i];      // Digital output data
        }
    }
}

// M2.5 or higher    
void DSRInterface::OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData)
{
    // This function is called every 100 msec
    // Only work within 50msec
    // RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    DSRInterface::OnMonitoringDataExCB");

    g_stDrState.nActualMode  = pData->_tCtrl._tState._iActualMode;                  // position control: 0, torque control: 1 ?????
    g_stDrState.nActualSpace = pData->_tCtrl._tState._iActualSpace;                 // joint space: 0, task space: 1    

    for (int i = 0; i < NUM_JOINT; i++){
        if(pData){  
            // joint         
            g_stDrState.fCurrentPosj[i] = pData->_tCtrl._tJoint._fActualPos[i];     // Position Actual Value in INC     
            g_stDrState.fCurrentVelj[i] = pData->_tCtrl._tJoint._fActualVel[i];     // Velocity Actual Value
            g_stDrState.fJointAbs[i]    = pData->_tCtrl._tJoint._fActualAbs[i];     // Position Actual Value in ABS
            g_stDrState.fJointErr[i]    = pData->_tCtrl._tJoint._fActualErr[i];     // Joint Error
            g_stDrState.fTargetPosj[i]  = pData->_tCtrl._tJoint._fTargetPos[i];     // Target Position
            g_stDrState.fTargetVelj[i]  = pData->_tCtrl._tJoint._fTargetVel[i];     // Target Velocity
            // task
            g_stDrState.fCurrentPosx[i]     = pData->_tCtrl._tTask._fActualPos[0][i];   //????? <---------이것 2개다 확인할 것  
            g_stDrState.fCurrentToolPosx[i] = pData->_tCtrl._tTask._fActualPos[1][i];   //????? <---------이것 2개다 확인할 것  
            g_stDrState.fCurrentVelx[i] = pData->_tCtrl._tTask._fActualVel[i];      // Velocity Actual Value
            g_stDrState.fTaskErr[i]     = pData->_tCtrl._tTask._fActualErr[i];      // Task Error
            g_stDrState.fTargetPosx[i]  = pData->_tCtrl._tTask._fTargetPos[i];      // Target Position
            g_stDrState.fTargetVelx[i]  = pData->_tCtrl._tTask._fTargetVel[i];      // Target Velocity
            // Torque
            g_stDrState.fDynamicTor[i]  = pData->_tCtrl._tTorque._fDynamicTor[i];   // Dynamics Torque
            g_stDrState.fActualJTS[i]   = pData->_tCtrl._tTorque._fActualJTS[i];    // Joint Torque Sensor Value
            g_stDrState.fActualEJT[i]   = pData->_tCtrl._tTorque._fActualEJT[i];    // External Joint Torque
            g_stDrState.fActualETT[i]   = pData->_tCtrl._tTorque._fActualETT[i];    // External Task Force/Torque

            g_stDrState.nActualBK[i]    = pData->_tMisc._iActualBK[i];              // brake state     
            g_stDrState.fActualMC[i]    = pData->_tMisc._fActualMC[i];              // motor input current
            g_stDrState.fActualMT[i]    = pData->_tMisc._fActualMT[i];              // motor current temperature
        }
    }
    g_stDrState.nSolutionSpace  = pData->_tCtrl._tTask._iSolutionSpace;             // Solution Space
    g_stDrState.dSyncTime       = pData->_tMisc._dSyncTime;                         // inner clock counter  

    for (int i = 5; i < NUM_BUTTON; i++){
        if(pData){
            g_stDrState.nActualBT[i]    = pData->_tMisc._iActualBT[i];              // robot button state
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(pData){
                g_stDrState.fRotationMatrix[j][i] = pData->_tCtrl._tTask._fRotationMatrix[j][i];    // Rotation Matrix
            }
        }
    }

    for (int i = 0; i < NUM_FLANGE_IO; i++){
        if(pData){
            g_stDrState.bFlangeDigitalInput[i]  = pData->_tMisc._iActualDI[i];      // Digital Input data             
            g_stDrState.bFlangeDigitalOutput[i] = pData->_tMisc._iActualDO[i];      // Digital output data
        }
    }

    //----- In M2.5 version or higher The following variables were added -----
    for (int i = 0; i < NUM_JOINT; i++){
        g_stDrState.fActualW2B[i] = pData->_tCtrl._tWorld._fActualW2B[i];
        g_stDrState.fCurrentVelW[i] = pData->_tCtrl._tWorld._fActualVel[i];
        g_stDrState.fWorldETT[i] = pData->_tCtrl._tWorld._fActualETT[i];
        g_stDrState.fTargetPosW[i] = pData->_tCtrl._tWorld._fTargetPos[i];
        g_stDrState.fTargetVelW[i] = pData->_tCtrl._tWorld._fTargetVel[i];
        g_stDrState.fCurrentVelU[i] = pData->_tCtrl._tWorld._fActualVel[i];
        g_stDrState.fUserETT[i] = pData->_tCtrl._tUser._fActualETT[i];
        g_stDrState.fTargetPosU[i] = pData->_tCtrl._tUser._fTargetPos[i];
        g_stDrState.fTargetVelU[i] = pData->_tCtrl._tUser._fTargetVel[i];
    }    

    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 6; j++){
            g_stDrState.fCurrentPosW[i][j] = pData->_tCtrl._tWorld._fActualPos[i][j];
            g_stDrState.fCurrentPosU[i][j] = pData->_tCtrl._tUser._fActualPos[i][j];
        }
    }

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            g_stDrState.fRotationMatrixWorld[j][i] = pData->_tCtrl._tWorld._fRotationMatrix[j][i];
            g_stDrState.fRotationMatrixUser[j][i] = pData->_tCtrl._tUser._fRotationMatrix[j][i];
        }
    }

    g_stDrState.iActualUCN = pData->_tCtrl._tUser._iActualUCN;
    g_stDrState.iParent    = pData->_tCtrl._tUser._iParent;
    //-------------------------------------------------------------------------
}

void DSRInterface::OnMonitoringModbusCB (const LPMONITORING_MODBUS pModbus)
{
    g_stDrState.nRegCount = pModbus->_iRegCount;
    for (int i = 0; i < pModbus->_iRegCount; i++){
        // cout << "[callback OnMonitoringModbusCB] " << pModbus->_tRegister[i]._szSymbol <<": " << pModbus->_tRegister[i]._iValue<< endl;
        g_stDrState.strModbusSymbol[i] = pModbus->_tRegister[i]._szSymbol;
        g_stDrState.nModbusValue[i]    = pModbus->_tRegister[i]._iValue;
    }
}

void DSRInterface::OnMonitoringStateCB(const ROBOT_STATE eState)
{
    //This function is called when the state changes.
    //RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"DSRInterface::OnMonitoringStateCB");    
    // Only work within 50msec
    
    switch((unsigned char)eState)
    {
#if 0 // TP initializing logic, Don't use in API level. (If you want to operate without TP, use this logic)       
    case eSTATE_NOT_READY:
    if (g_bHasControlAuthority) Drfl.set_robot_control(CONTROL_INIT_CONFIG);
        break;
    case eSTATE_INITIALIZING:
        // add initalizing logic
        if (g_bHasControlAuthority) Drfl.set_robot_control(CONTROL_ENABLE_OPERATION);
        break;
#endif      
    case STATE_EMERGENCY_STOP:
        // popup
        break;
    case STATE_STANDBY:
    case STATE_MOVING:
    case STATE_TEACHING:
        break;
    case STATE_SAFE_STOP:
        if (g_bHasControlAuthority) {
            Drfl.set_safe_stop_reset_type(SAFE_STOP_RESET_TYPE_DEFAULT);
            Drfl.set_robot_control(CONTROL_RESET_SAFET_STOP);
            // Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS,SAFETY_MODE_EVENT_MOVE);
        }
        break;
    case STATE_SAFE_OFF:
        if (g_bHasControlAuthority){
            if (init_state){
            Drfl.set_robot_control(CONTROL_SERVO_ON);
            Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);   //Idle Servo Off 후 servo on 하는 상황 발생 시 set_robot_mode 명령을 전송해 manual 로 전환. add 2020/04/28
            // Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS,SAFETY_MODE_EVENT_MOVE);
            init_state = FALSE;
            }
        } 
        break;
    case STATE_SAFE_STOP2:
        if (g_bHasControlAuthority) Drfl.set_robot_control(CONTROL_RECOVERY_SAFE_STOP);
        break;
    case STATE_SAFE_OFF2:
        if (g_bHasControlAuthority) {
            Drfl.set_robot_control(CONTROL_RECOVERY_SAFE_OFF);
        }
        break;
    case STATE_RECOVERY:
        Drfl.set_robot_control(CONTROL_RESET_RECOVERY);
        break;
    default:
        break;
    }

    // cout << "[callback OnMonitoringStateCB] current state: " << GetRobotStateString((int)eState) << endl;
    g_stDrState.nRobotState = (int)eState;
    strncpy(g_stDrState.strRobotState, GetRobotStateString((int)eState), MAX_SYMBOL_SIZE); 
}

void DSRInterface::OnMonitoringAccessControlCB(const MONITORING_ACCESS_CONTROL eAccCtrl)
{
    // Only work within 50msec

    // cout << "[callback OnMonitoringAccessControlCB] eAccCtrl: " << eAccCtrl << endl;
    switch(eAccCtrl)
    {
    case MONITORING_ACCESS_CONTROL_REQUEST:
        Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO);
        //Drfl.TransitControlAuth(MANaGE_ACCESS_CONTROL_RESPONSE_YES);
        break;
    case MONITORING_ACCESS_CONTROL_GRANT:
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");   
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Access control granted ");
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");   
        g_bHasControlAuthority = TRUE;
        OnMonitoringStateCB(Drfl.GetRobotState());
        break;
    case MONITORING_ACCESS_CONTROL_DENY:
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");   
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"    Access control deny ");
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"_______________________________________________\n");   
        break;
    case MONITORING_ACCESS_CONTROL_LOSS:
        g_bHasControlAuthority = FALSE;
        if (g_bTpInitailizingComplted) {
            Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST);
        }
        break;
    default:
        break;
    }
    g_stDrState.nAccessControl = (int)eAccCtrl;
}

void DSRInterface::OnLogAlarm(LPLOG_ALARM pLogAlarm)
{
    //This function is called when an error occurs.
    auto PubRobotError = s_node_->create_publisher<dsr_msgs2::msg::RobotError>("error", 100);
    dsr_msgs2::msg::RobotError msg;

    switch(pLogAlarm->_iLevel)
    {
    case LOG_LEVEL_SYSINFO:
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2"),"[callback OnLogAlarm]");
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," level : %d",(unsigned int)pLogAlarm->_iLevel);
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," group : %d",(unsigned int)pLogAlarm->_iGroup);
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," index : %d", pLogAlarm->_iIndex);
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[0] );
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[1] );
        RCLCPP_INFO(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[2] );
        break;
    case LOG_LEVEL_SYSWARN:
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2"),"[callback OnLogAlarm]");
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," level : %d",(unsigned int)pLogAlarm->_iLevel);
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," group : %d",(unsigned int)pLogAlarm->_iGroup);
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," index : %d", pLogAlarm->_iIndex);
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[0] );
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[1] );
        RCLCPP_WARN(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[2] );
        break;
    case LOG_LEVEL_SYSERROR:
    default:
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2"),"[callback OnLogAlarm]");
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," level : %d",(unsigned int)pLogAlarm->_iLevel);
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," group : %d",(unsigned int)pLogAlarm->_iGroup);
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," index : %d", pLogAlarm->_iIndex);
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[0] );
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[1] );
        RCLCPP_ERROR(rclcpp::get_logger("dsr_hw_interface2")," param : %s", pLogAlarm->_szParam[2] );
        break;
    }

    g_stDrError.nLevel=(unsigned int)pLogAlarm->_iLevel;
    g_stDrError.nGroup=(unsigned int)pLogAlarm->_iGroup;
    g_stDrError.nCode=pLogAlarm->_iIndex;
    strncpy(g_stDrError.strMsg1, pLogAlarm->_szParam[0], MAX_STRING_SIZE);
    strncpy(g_stDrError.strMsg2, pLogAlarm->_szParam[1], MAX_STRING_SIZE);
    strncpy(g_stDrError.strMsg3, pLogAlarm->_szParam[2], MAX_STRING_SIZE);

    msg.level=g_stDrError.nLevel;
    msg.group=g_stDrError.nGroup;
    msg.code=g_stDrError.nCode;
    msg.msg1=g_stDrError.strMsg1;
    msg.msg2=g_stDrError.strMsg2;
    msg.msg3=g_stDrError.strMsg3;

    PubRobotError->publish(msg);
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dsr_hardware2::DRHWInterface, hardware_interface::SystemInterface)
      

