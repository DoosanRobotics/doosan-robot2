/*
 * class of doosan robot control 
 * Author: Kab Kyoum Kim (kabkyoum.kim@doosan.com)
 *
 * Copyright (c) 2019 Doosan Robotics
 * Use of this source code is governed by the BSD, see LICENSE
*/

#include "../include/dsr_robot.h"

using namespace DSR_Robot;

int CDsrRobot::stop(int nMode/*=STOP_TYPE_QUICK*/)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Publisher pubRobotStop = node->advertise<dsr_msgs2::RobotStop>(m_strSrvNamePrefix + "/stop",100);   
    dsr_msgs2::RobotStop msg;
    
    msg.stop_mode  = nMode;
    pubRobotStop.publish(msg);
    return 0;
}

int CDsrRobot::set_robot_mode(int robot_mode){
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetRobotMode = node->serviceClient<dsr_msgs2::SetRobotMode>(m_strSrvNamePrefix + "/system/set_robot_mode");
    dsr_msgs2::SetRobotMode srv;

    srv.request.robot_mode = robot_mode;

    if(srvSetRobotMode.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_robot_mode\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::get_robot_mode(){
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetRobotMode = node->serviceClient<dsr_msgs2::GetRobotMode>(m_strSrvNamePrefix + "/system/get_robot_mode");
    dsr_msgs2::GetRobotMode srv;

    if(srvGetRobotMode.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.robot_mode);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_robot_mode\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::set_robot_system(int robot_system)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetRobotSystem = node->serviceClient<dsr_msgs2::SetRobotSystem>(m_strSrvNamePrefix + "/system/set_robot_system");
    dsr_msgs2::SetRobotSystem srv;

    if(srvSetRobotSystem.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_robot_system\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::get_robot_system()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetRobotSystem = node->serviceClient<dsr_msgs2::GetRobotSystem>(m_strSrvNamePrefix + "/system/get_robot_system");
    dsr_msgs2::GetRobotSystem srv;

    if(srvGetRobotSystem.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.robot_system);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_robot_system\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::set_robot_speed_mode(int speed_mode)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetRobotSpeedMode = node->serviceClient<dsr_msgs2::SetRobotSpeedMode>(m_strSrvNamePrefix + "/system/set_robot_speed_mode");
    dsr_msgs2::SetRobotSpeedMode srv;

    if(srvSetRobotSpeedMode.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_robot_speed_mode\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}


int CDsrRobot::get_robot_speed_mode()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetRobotSpeedMode = node->serviceClient<dsr_msgs2::GetRobotSpeedMode>(m_strSrvNamePrefix + "/system/get_robot_speed_mode");
    dsr_msgs2::GetRobotSpeedMode srv;

    if(srvGetRobotSpeedMode.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.speed_mode);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_robot_speed_mode\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::set_safe_stop_reset_type(int reset_type)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetSafeStopResetType = node->serviceClient<dsr_msgs2::SetSafeStopResetType>(m_strSrvNamePrefix + "/system/set_safe_stop_reset_type");
    dsr_msgs2::SetSafeStopResetType srv;

    if(srvSetSafeStopResetType.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_safe_stop_reset_type\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::get_current_pose(int space_type)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetCurrentPose = node->serviceClient<dsr_msgs2::GetCurrentPose>(m_strSrvNamePrefix + "/system/get_current_pose");
    dsr_msgs2::GetCurrentPose srv;

    if(srvGetCurrentPose.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return 1;
        //return (srv.response.pos);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_current_pose\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}


int CDsrRobot::get_current_solution_space()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetCurrentSolutionSpace = node->serviceClient<dsr_msgs2::GetCurrentSolutionSpace>(m_strSrvNamePrefix + "/system/get_current_solution_space");
    dsr_msgs2::GetCurrentSolutionSpace srv;

    if(srvGetCurrentSolutionSpace.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.solution_space);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_current_solution_space\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}
/*
int CDsrRobot::get_last_alarm()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetLastAlarm = node->serviceClient<dsr_msgs2::GetLastAlarm>(m_strSrvNamePrefix + "/system/get_last_alarm");
    dsr_msgs2::GetLastAlarm srv;

    if(srvGetLastAlarm.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return 1;
        //return (srv.response.solution_space);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_current_solution_space\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}
*/

int CDsrRobot::movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime/*=0.f*/, float fBlendingRadius/*=0.f*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/, int nBlendingType/*=BLENDING_SPEED_TYPE_DUPLICATE*/)
{
    return _movej(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveMode, nBlendingType, 0);
}
int CDsrRobot::amovej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime/*=0.f*/, float fBlendingRadius/*=0.f*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/, int nBlendingType/*=BLENDING_SPEED_TYPE_DUPLICATE*/)
{
    return _movej(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveMode, nBlendingType, 1);
}
int CDsrRobot::_movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveMode, int nBlendingType, int nSyncType) 
{  
    //ros::ServiceClient srvMoveJoint = nh.serviceClient<dsr_msgs2::MoveJoint>(m_strSrvNamePrefix + "/motion/move_joint");
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvMoveJoint = node->serviceClient<dsr_msgs2::MoveJoint>(m_strSrvNamePrefix + "/motion/move_joint");

    dsr_msgs2::MoveJoint srv;

    for(int i=0; i<NUM_JOINT; i++)
        srv.request.pos[i] = fTargetPos[i];
    srv.request.vel = fTargetVel;
    srv.request.acc = fTargetAcc;
    srv.request.time = fTargetTime;
    srv.request.radius = fBlendingRadius;
    srv.request.mode = nMoveMode;
    srv.request.blendType = nBlendingType; 
    srv.request.syncType = nSyncType;
    ROS_INFO("service call: %s/motion/move_joint",m_strSrvNamePrefix.c_str());
    ROS_INFO("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0],srv.request.pos[1],srv.request.pos[2],srv.request.pos[3],srv.request.pos[4],srv.request.pos[5]);
    ROS_INFO("  <vel> %7.3f , <acc> %7.3f, <time> %7.3f",srv.request.vel, srv.request.acc, srv.request.time);
    ROS_INFO("  <mode> %d , <radius> %7.3f, <blendType> %d",srv.request.mode, srv.request.radius, srv.request.blendType);

    if(srvMoveJoint.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : move_joint\n");
        ros::shutdown();  
        return -1;
    }

    return 0; 
}


int CDsrRobot::movel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime/*=0.f*/, float fBlendingRadius/*=0.f*/, int nMoveReference/*=MOVE_REFERENCE_BASE*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/, int nBlendingType/*=BLENDING_SPEED_TYPE_DUPLICATE*/)
{
    return _movel(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, nBlendingType, 0); 
}
int CDsrRobot::amovel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime/*=0.f*/, float fBlendingRadius/*=0.f*/, int nMoveReference/*=MOVE_REFERENCE_BASE*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/, int nBlendingType/*=BLENDING_SPEED_TYPE_DUPLICATE*/)
{
    return _movel(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, nBlendingType, 1); 
}
int CDsrRobot::_movel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType)             
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveLine = node->serviceClient<dsr_msgs2::MoveLine>(m_strSrvNamePrefix + "/motion/move_line");
    dsr_msgs2::MoveLine srv;

    for(int i=0; i<NUM_JOINT; i++)
        srv.request.pos[i] = fTargetPos[i];
    for(int i=0; i<2; i++){
        srv.request.vel[i] = fTargetVel[i];
        srv.request.acc[i] = fTargetAcc[i];
    }
    srv.request.time = fTargetTime;
    srv.request.radius = fBlendingRadius;
    srv.request.ref  = nMoveReference;
    srv.request.mode = nMoveMode;
    srv.request.blendType = nBlendingType; 
    srv.request.syncType = nSyncType;

    //ROS_INFO("service call: %s/motion/move_line",m_strSrvNamePrefix.c_str());
    //ROS_INFO("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0],srv.request.pos[1],srv.request.pos[2],srv.request.pos[3],srv.request.pos[4],srv.request.pos[5]);
    //ROS_INFO("  <vel> %7.3f,%7.3f <acc> %7.3f,%7.3f <time> %7.3f",srv.request.vel[0],srv.request.vel[1],srv.request.acc[0],srv.request.acc[1], srv.request.time);
    //ROS_INFO("  <mode> %d, <ref> %d, <radius> %7.3f, <blendType> %d",srv.request.mode,srv.request.ref, srv.request.radius, srv.request.blendType);

    if(srvMoveLine.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {     
        ROS_ERROR("Failed to call service dr_control_service : move_line\n");
        ros::shutdown();
        return -1;
    }
    return 0; 
}


int CDsrRobot::movejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime/*=0.f*/, float fBlendingRadius/*=0.f*/, int nMoveReference/*=MOVE_REFERENCE_BASE*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/, int nBlendingType/*=BLENDING_SPEED_TYPE_DUPLICATE*/, int nSolSpace/*=0*/)
{
    return _movejx(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, nBlendingType, nSolSpace, 0);
}
int CDsrRobot::amovejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime/*=0.f*/, float fBlendingRadius/*=0.f*/, int nMoveReference/*=MOVE_REFERENCE_BASE*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/, int nBlendingType/*=BLENDING_SPEED_TYPE_DUPLICATE*/, int nSolSpace/*=0*/)
{
    return _movejx(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, nBlendingType, nSolSpace, 1);
}
int CDsrRobot::_movejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSolSpace, int nSyncType)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveJointx = node->serviceClient<dsr_msgs2::MoveJointx>(m_strSrvNamePrefix + "/motion/move_jointx");
    dsr_msgs2::MoveJointx srv; 

    for(int i=0; i<NUM_TASK; i++)
        srv.request.pos[i] = fTargetPos[i];
    srv.request.vel = fTargetVel;
    srv.request.acc = fTargetAcc;
    srv.request.time = fTargetTime;
    srv.request.radius = fBlendingRadius;
    srv.request.ref = nMoveReference;
    srv.request.mode = nMoveMode;
    srv.request.blendType = nBlendingType;
    srv.request.sol = nSolSpace;

    //ROS_INFO("service call: %s/motion/move_jointx",m_strSrvNamePrefix.c_str());
    //ROS_INFO("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0],srv.request.pos[1],srv.request.pos[2],srv.request.pos[3],srv.request.pos[4],srv.request.pos[5]);
    //ROS_INFO("  <vel> %7.3f <acc> %7.3f <time> %7.3f",srv.request.vel,srv.request.acc, srv.request.time);
    //ROS_INFO("  <mode> %d, <ref> %d, <radius> %7.3f, <blendType> %d, <sol> %d",srv.request.mode,srv.request.ref, srv.request.radius, srv.request.blendType, srv.request.sol);

    if(srvMoveJointx.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {     
        ROS_ERROR("Failed to call service dr_control_service : move_jointx\n");
        ros::shutdown();
        return -1;
    }

    return 0; 
}


int CDsrRobot::movec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime/*=0.f*/, float fBlendingRadius/*=0.f*/, int nMoveReference/*=MOVE_REFERENCE_BASE*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/, int nBlendingType/*=BLENDING_SPEED_TYPE_DUPLICATE*/)
{
    return _movec(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, nBlendingType, 0);
}
int CDsrRobot::amovec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime/*=0.f*/, float fBlendingRadius/*=0.f*/, int nMoveReference/*=MOVE_REFERENCE_BASE*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/, int nBlendingType/*=BLENDING_SPEED_TYPE_DUPLICATE*/)
{
    return _movec(fTargetPos, fTargetVel, fTargetAcc, fTargetTime, fBlendingRadius, nMoveReference, nMoveMode, nBlendingType, 1); 
}
int CDsrRobot::_movec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveCircle = node->serviceClient<dsr_msgs2::MoveCircle>(m_strSrvNamePrefix + "/motion/move_circle");

    dsr_msgs2::MoveCircle srv; 
    std::vector<std_msgs::Float64MultiArray> poses;
    std_msgs::Float64MultiArray pos;
    for(int i = 0; i < 2; i++){
        pos.data.clear();
        for(int j = 0; j < NUM_TASK; j++){
            pos.data.push_back(fTargetPos[i][j]);
        }
        poses.push_back(pos);
    }

    //ROS_INFO("  <xxx pos1> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[0][0],fTargetPos[0][1],fTargetPos[0][2],fTargetPos[0][3],fTargetPos[0][4],fTargetPos[0][5]);
    //ROS_INFO("  <xxx pos2> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",fTargetPos[1][0],fTargetPos[1][1],fTargetPos[1][2],fTargetPos[1][3],fTargetPos[1][4],fTargetPos[1][5]);
    //ROS_INFO("  <pos1> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",poses[0].data[0],poses[0].data[1],poses[0].data[2],poses[0].data[3],poses[0].data[4],poses[0].data[5]);
    //ROS_INFO("  <pos2> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",poses[1].data[0],poses[1].data[1],poses[1].data[2],poses[1].data[3],poses[1].data[4],poses[1].data[5]);

    srv.request.pos = poses;
    for(int i=0; i<2; i++){
        srv.request.vel[i] = fTargetVel[i];
        srv.request.acc[i] = fTargetAcc[i];
    }
    srv.request.time = fTargetTime;
    srv.request.radius = fBlendingRadius;
    srv.request.ref  = nMoveReference;
    srv.request.mode = nMoveMode;
    srv.request.blendType = nBlendingType;
    srv.request.syncType = nSyncType;

    //ROS_INFO("service call: %s/motion/move_circle",m_strSrvNamePrefix.c_str());
    //ROS_INFO("  <pos1> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0].data[0],srv.request.pos[0].data[1],srv.request.pos[0].data[2],srv.request.pos[0].data[3],srv.request.pos[0].data[4],srv.request.pos[0].data[5]);
    //ROS_INFO("  <pos2> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[1].data[0],srv.request.pos[1].data[1],srv.request.pos[1].data[2],srv.request.pos[1].data[3],srv.request.pos[1].data[4],srv.request.pos[1].data[5]);
    //ROS_INFO("  <vel> %7.3f,%7.3f <acc> %7.3f,%7.3f <time> %7.3f",srv.request.vel[0],srv.request.vel[1],srv.request.acc[0],srv.request.acc[1], srv.request.time);
    //ROS_INFO("  <mode> %d, <ref> %d, <radius> %7.3f, <blendType> %d",srv.request.mode,srv.request.ref, srv.request.radius, srv.request.blendType);

    if(srvMoveCircle.call(srv))
    {
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service dr_control_service : move_circle\n");
        ros::shutdown();
        return -1;
    }

    return 0;
}


int CDsrRobot::movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime/*=0.f*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/)
{
    return _movesj(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, nMoveMode, 0);
}
int CDsrRobot::amovesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime/*=0.f*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/)
{
    return _movesj(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, nMoveMode, 1);
}
int CDsrRobot::_movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime, int nMoveMode, int nSyncType)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveSplineJoint = node->serviceClient<dsr_msgs2::MoveSplineJoint>(m_strSrvNamePrefix + "/motion/move_spline_joint");
    dsr_msgs2::MoveSplineJoint srv;
    std::vector<std_msgs::Float64MultiArray> poses;
    std_msgs::Float64MultiArray pos;

    for(int i = 0; i < MAX_SPLINE_POINT; i++){
        pos.data.clear();
        for(int j = 0; j < NUM_JOINT; j++){
            pos.data.push_back(fTargetPos[i][j]);
        }
        poses.push_back(pos);
    }
    srv.request.pos = poses;
    srv.request.posCnt = nPosCount;

    srv.request.vel = fTargetVel;
    srv.request.acc = fTargetAcc;
    srv.request.time = fTargetTime;
    srv.request.mode = nMoveMode;
    srv.request.syncType = nSyncType;

    if(srvMoveSplineJoint.call(srv))
    {
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service dr_control_service : move_spline_joint\n");
        ros::shutdown();
        return -1;
    }

    return 0; 
}


int CDsrRobot::movesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime/*=0.f*/, int nMoveReference/*=MOVE_REFERENCE_BASE*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/, int nVelOpt/*=SPLINE_VELOCITY_OPTION_DEFAULT*/)
{
    return _movesx(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, nMoveReference, nMoveMode, nVelOpt, 0);
}
int CDsrRobot::amovesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime/*=0.f*/, int nMoveReference/*=MOVE_REFERENCE_BASE*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/, int nVelOpt/*=SPLINE_VELOCITY_OPTION_DEFAULT*/)
{
    return _movesx(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, nMoveReference, nMoveMode, nVelOpt, 1);
}
int CDsrRobot::_movesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nMoveReference, int nMoveMode, int nVelOpt, int nSyncType)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveSplineTask = node->serviceClient<dsr_msgs2::MoveSplineTask>(m_strSrvNamePrefix + "/motion/move_spline_task");
    dsr_msgs2::MoveSplineTask srv;
    std::vector<std_msgs::Float64MultiArray> poses;
    std_msgs::Float64MultiArray pos;

    for(int i = 0; i < MAX_SPLINE_POINT; i++){
        pos.data.clear();
        for(int j = 0; j < NUM_TASK; j++){
            pos.data.push_back(fTargetPos[i][j]);
        }
        poses.push_back(pos);
    }
    srv.request.pos = poses;
    srv.request.posCnt = nPosCount;

    for(int i=0; i<2; i++){
        srv.request.vel[i] = fTargetVel[i];
        srv.request.acc[i] = fTargetAcc[i];
    }
    srv.request.time = fTargetTime;
    srv.request.ref = nMoveReference;
    srv.request.mode = nMoveMode;
    srv.request.opt = nVelOpt;
    srv.request.syncType = nSyncType;

    if(srvMoveSplineTask.call(srv))
    {
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service dr_control_service : move_spline_task\n");
        ros::shutdown();
        return -1;
    }

    return 0; 
}


int CDsrRobot::moveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime/*=0.f*/, int nMoveReference/*=MOVE_REFERENCE_BASE*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/)
{
    return _moveb(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, nMoveReference, nMoveMode, 0);
}
int CDsrRobot::amoveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime/*=0.f*/, int nMoveReference/*=MOVE_REFERENCE_BASE*/, int nMoveMode/*=MOVE_MODE_ABSOLUTE*/)
{
    return _moveb(fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, nMoveReference, nMoveMode, 1);
}
int CDsrRobot::_moveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nMoveReference, int nMoveMode, int nSyncType)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveBlending = node->serviceClient<dsr_msgs2::MoveBlending>(m_strSrvNamePrefix + "/motion/move_blending");
    dsr_msgs2::MoveBlending srv;
    std::vector<std_msgs::Float64MultiArray> segments;
    std_msgs::Float64MultiArray segment;

    for(int i=0; i<nPosCount; i++){
        segment.data.clear();

        for(int j=0; j<NUM_TASK; j++)
            segment.data.push_back( fTargetPos[i]._fTargetPos[0][j]);
        for(int j=0; j<NUM_TASK; j++)
            segment.data.push_back( fTargetPos[i]._fTargetPos[1][j]);

        segment.data.push_back( fTargetPos[i]._iBlendType );
        segment.data.push_back( fTargetPos[i]._fBlendRad  );  

        segments.push_back(segment);
    }
    srv.request.segment = segments;
    srv.request.posCnt = nPosCount;    

    for(int i=0; i<2; i++){
        srv.request.vel[i] = fTargetVel[i];
        srv.request.acc[i] = fTargetAcc[i];
    }
    srv.request.time = fTargetTime;    
    srv.request.ref = nMoveReference;
    srv.request.mode = nMoveMode;
    srv.request.syncType = nSyncType;

    if(srvMoveBlending.call(srv))
    {
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service dr_control_service : move_spline_blending\n");
        ros::shutdown();
        return -1;
    }

    return 0; 
}


int CDsrRobot::move_spiral(float fRevolution, float fMaxRadius, float fMaxLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime/*=0.f*/, int nTaskAxis/*=TASK_AXIS_Z*/, int nMoveReference/*=MOVE_REFERENCE_TOOL*/)
{
    return _move_spiral(fRevolution, fMaxRadius, fMaxLength, fTargetVel, fTargetAcc, fTargetTime, nTaskAxis, nMoveReference, 0);
}
int CDsrRobot::amove_spiral(float fRevolution, float fMaxRadius, float fMaxLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime/*=0.f*/, int nTaskAxis/*=TASK_AXIS_Z*/, int nMoveReference/*=MOVE_REFERENCE_TOOL*/)
{
    return _move_spiral(fRevolution, fMaxRadius, fMaxLength, fTargetVel, fTargetAcc, fTargetTime, nTaskAxis, nMoveReference, 1);
}
int CDsrRobot::_move_spiral(float fRevolution, float fMaxRadius, float fMaxLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nTaskAxis, int nMoveReference, int nSyncType)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMoveSpiral = node->serviceClient<dsr_msgs2::MoveSpiral>(m_strSrvNamePrefix + "/motion/move_spiral");
    dsr_msgs2::MoveSpiral srv;

    srv.request.revolution = fRevolution;
    srv.request.maxRadius = fMaxRadius;
    srv.request.maxLength = fMaxLength;
    for(int i=0; i<2; i++){
        srv.request.vel[i] = fTargetVel[i];
        srv.request.acc[i] = fTargetAcc[i];
    }
    srv.request.time = fTargetTime;
    srv.request.taskAxis = nTaskAxis;
    srv.request.ref = nMoveReference;      
    srv.request.syncType = nSyncType;

    if(srvMoveSpiral.call(srv)){
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else{
        ROS_ERROR("Failed to call service dr_control_service : move_spiral\n");
        ros::shutdown();
        return -1;
    }
    
    return 0;  
}


int CDsrRobot::move_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime/*=0.f*/, int nRepeat/*=1*/, int nMoveReference/*=MOVE_REFERENCE_TOOL*/)
{
    return _move_periodic(fAmplitude, fPeriodic, fAccelTime, nRepeat, nMoveReference, 0);
}
int CDsrRobot::amove_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime/*=0.f*/, int nRepeat/*=1*/, int nMoveReference/*=MOVE_REFERENCE_TOOL*/)
{
    return _move_periodic(fAmplitude, fPeriodic, fAccelTime, nRepeat, nMoveReference, 1);
}
int CDsrRobot::_move_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, int nRepeat, int nMoveReference, int nSyncType)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();  
    ros::ServiceClient srvMovePeriodic = node->serviceClient<dsr_msgs2::MovePeriodic>(m_strSrvNamePrefix + "/motion/move_periodic");
    dsr_msgs2::MovePeriodic srv;

    for(int i=0; i<NUM_TASK; i++){
        srv.request.amp[i] = fAmplitude[i];
        srv.request.periodic[i] = fPeriodic[i];
    }
    srv.request.acc = fAccelTime;
    srv.request.repeat = nRepeat;
    srv.request.ref = nMoveReference;
    srv.request.syncType = nSyncType;

    if(srvMovePeriodic.call(srv))
    {
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service dr_control_service : move_periodic\n");
        ros::shutdown();
        return -1;
    }

    return 0;
}


int CDsrRobot::move_wait()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvMoveWait = node->serviceClient<dsr_msgs2::MoveWait>(m_strSrvNamePrefix + "/motion/move_wait");
    dsr_msgs2::MoveWait srv;

    
    if(srvMoveWait.call(srv))
    {
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service dr_control_service : move_wait\n");
        ros::shutdown();
        return -1;
    }

    return 0; 
}

int CDsrRobot::jog(int jog_axis, int move_reference, int speed)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvJog = node->serviceClient<dsr_msgs2::Jog>(m_strSrvNamePrefix + "/motion/jog");

    dsr_msgs2::Jog srv;

    srv.request.jog_axis = jog_axis;
    srv.request.move_reference = move_reference;
    srv.request.speed = speed;

    if(srvJog.call(srv))
    {         
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : jog\n");
        ros::shutdown();  
        return -1;
    }

    return 0; 
}

int CDsrRobot::jog_multi(float jog_axis[NUM_TASK], int move_reference, int speed)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvJogMulti = node->serviceClient<dsr_msgs2::JogMulti>(m_strSrvNamePrefix + "/motion/jog_multi");

    dsr_msgs2::JogMulti srv;

    for(int i=0; i<NUM_TASK; i++)
        srv.request.jog_axis[i] = jog_axis[i];
    srv.request.move_reference = move_reference;
    srv.request.speed = speed;

    if(srvJogMulti.call(srv))
    {         
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : jog_multi\n");
        ros::shutdown();  
        return -1;
    }

    return 0; 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int CDsrRobot::config_create_tcp(string strName, float fTargetPos[NUM_TASK])
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvConfigCreateTcp = node->serviceClient<dsr_msgs2::ConfigCreateTcp>(m_strSrvNamePrefix + "/tcp/config_create_tcp");
    dsr_msgs2::ConfigCreateTcp srv;

    srv.request.name = strName;
    for(int i=0; i<NUM_TASK; i++){
        srv.request.pos[i] = fTargetPos[i];
    }

    if(srvConfigCreateTcp.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : config_create_tcp\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::config_delete_tcp(string strName)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvConfigDeleteTcp = node->serviceClient<dsr_msgs2::ConfigDeleteTcp>(m_strSrvNamePrefix + "/tcp/config_delete_tcp");
    dsr_msgs2::ConfigDeleteTcp srv;

    srv.request.name = strName;

    if(srvConfigDeleteTcp.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : config_delete_tcp\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::set_current_tcp(string strName)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetCurrentTcp = node->serviceClient<dsr_msgs2::SetCurrentTcp>(m_strSrvNamePrefix + "/tcp/set_current_tcp");
    dsr_msgs2::SetCurrentTcp srv;

    srv.request.name = strName;
    ROS_INFO("set current tcp name is : %s", strName.c_str());
    if(srvSetCurrentTcp.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_current_tcp\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

string CDsrRobot::get_current_tcp()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetCurrentTcp = node->serviceClient<dsr_msgs2::GetCurrentTcp>(m_strSrvNamePrefix + "/tcp/get_current_tcp");
    dsr_msgs2::GetCurrentTcp srv;

    if(srvGetCurrentTcp.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %s\n", (srv.response.info).c_str());
        return (srv.response.info);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_current_tcp\n");
        ros::shutdown();  
        return NULL;
    }
    return NULL;
}

//-----------------------------------------------------------------------------------------------------------------------------------

int CDsrRobot::config_create_tool(string strName,
                       float fTargetWeight,
                       float fTargetCog[3],
                       float fTargetInertia[NUM_TASK])
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvConfigCreateTool = node->serviceClient<dsr_msgs2::ConfigCreateTool>(m_strSrvNamePrefix + "/tool/config_create_tool");
    dsr_msgs2::ConfigCreateTool srv;

    srv.request.name = strName;
    srv.request.weight = fTargetWeight;
    for(int i=0; i<3; i++){
        srv.request.cog[i] = fTargetCog[i];
    }
    for(int i=0; i<NUM_TASK; i++){
        srv.request.inertia[i] = fTargetInertia[i];
    }

    if(srvConfigCreateTool.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : config_create_tool\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::config_delete_tool(string strName)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvConfigDeleteTool = node->serviceClient<dsr_msgs2::ConfigDeleteTool>(m_strSrvNamePrefix + "/tool/config_delete_tool");
    dsr_msgs2::ConfigDeleteTool srv;

    srv.request.name = strName;
    if(srvConfigDeleteTool.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : config_delete_tool\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::set_current_tool(string strName)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetCurrentTool = node->serviceClient<dsr_msgs2::SetCurrentTool>(m_strSrvNamePrefix + "/tool/set_current_tool");
    dsr_msgs2::SetCurrentTool srv;

    srv.request.name = strName;
    ROS_INFO("set current tool name is : %s", strName.c_str());
    if(srvSetCurrentTool.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_current_tool\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

string CDsrRobot::get_current_tool()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetCurrentTool = node->serviceClient<dsr_msgs2::GetCurrentTool>(m_strSrvNamePrefix + "/tool/get_current_tool");
    dsr_msgs2::GetCurrentTool srv;

    if(srvGetCurrentTool.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %s\n", srv.response.info.c_str());
        return (srv.response.info);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_current_tool\n");
        ros::shutdown();
        return NULL;
    }
    return NULL;
}

//-----------------------------------------------------------------------------------------------------------------------------------

int CDsrRobot::set_digital_output(int nGpioIndex, bool bGpioValue)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetCtrlBoxDigitalOutput = node->serviceClient<dsr_msgs2::SetCtrlBoxDigitalOutput>(m_strSrvNamePrefix + "/io/set_digital_output");
    dsr_msgs2::SetCtrlBoxDigitalOutput srv;

    srv.request.index = nGpioIndex;
    srv.request.value = bGpioValue;
    
    if(srvSetCtrlBoxDigitalOutput.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_digital_output\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::get_digital_input(int nGpioIndex)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetCtrlBoxDigitalInput = node->serviceClient<dsr_msgs2::GetCtrlBoxDigitalInput>(m_strSrvNamePrefix + "/io/get_digital_input");
    dsr_msgs2::GetCtrlBoxDigitalInput srv;

    srv.request.index = nGpioIndex;

    if(srvGetCtrlBoxDigitalInput.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.value);
        return (srv.response.value);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_digital_input\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::set_tool_digital_output(int nGpioIndex, bool bGpioValue)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetToolDigitalOutput = node->serviceClient<dsr_msgs2::SetToolDigitalOutput>(m_strSrvNamePrefix + "/io/set_tool_digital_output");
    dsr_msgs2::SetToolDigitalOutput srv;

    srv.request.index = nGpioIndex;
    srv.request.value = bGpioValue;

    if(srvSetToolDigitalOutput.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_tool_digital_output\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::get_tool_digital_input(int nGpioIndex)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetToolDigitalInput = node->serviceClient<dsr_msgs2::GetToolDigitalInput>(m_strSrvNamePrefix + "/io/get_tool_digital_input");
    dsr_msgs2::GetToolDigitalInput srv;

    srv.request.index = nGpioIndex;

    if(srvGetToolDigitalInput.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.value);
        return (srv.response.value);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_tool_digital_input\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::set_analog_output(int nGpioChannel, float fGpioValue)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetCtrlBoxAnalogOutput = node->serviceClient<dsr_msgs2::SetCtrlBoxAnalogOutput>(m_strSrvNamePrefix + "/io/set_analog_output");
    dsr_msgs2::SetCtrlBoxAnalogOutput srv;

    srv.request.channel = nGpioChannel;
    srv.request.value = fGpioValue;

    if(srvSetCtrlBoxAnalogOutput.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_analog_output\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::get_analog_input(int nGpioChannel)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetCtrlBoxAnalogInput = node->serviceClient<dsr_msgs2::GetCtrlBoxDigitalInput>(m_strSrvNamePrefix + "/io/get_analog_input");
    dsr_msgs2::GetCtrlBoxAnalogInput srv;

    srv.request.channel = nGpioChannel;

    if(srvGetCtrlBoxAnalogInput.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.value);
        return (srv.response.value);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_analog_input\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::set_analog_output_type(int nGpioChannel, int nGpioMode)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetCtrlBoxAnalogOutputType = node->serviceClient<dsr_msgs2::SetCtrlBoxAnalogOutputType>(m_strSrvNamePrefix + "/io/set_analog_output_type");
    dsr_msgs2::SetCtrlBoxAnalogOutputType srv;

    srv.request.channel = nGpioChannel;
    srv.request.mode = nGpioMode;

    if(srvSetCtrlBoxAnalogOutputType.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_analog_output_type\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::set_analog_input_type(int nGpioChannel, int nGpioMode)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetCtrlBoxAnalogInputType = node->serviceClient<dsr_msgs2::SetCtrlBoxAnalogInputType>(m_strSrvNamePrefix + "/io/set_analog_input_type");
    dsr_msgs2::SetCtrlBoxAnalogInputType srv;

    srv.request.channel = nGpioChannel;
    srv.request.mode = nGpioMode;

    if(srvSetCtrlBoxAnalogInputType.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_analog_input_type\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

//-----------------------------------------------------------------------------------------------------------------------------------

int CDsrRobot::config_create_modbus(string strName, 
                       string strIP, 
                       int nPort, 
                       int nRegType, 
                       int nRegIndex, 
                       int nRegValue,/* = 0*/
                       int nSlaveID/* = 255*/
                       )
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvConfigCreateModbus = node->serviceClient<dsr_msgs2::ConfigCreateModbus>(m_strSrvNamePrefix + "/modbus/config_create_modbus");
    dsr_msgs2::ConfigCreateModbus srv;

    srv.request.name = strName;
    srv.request.ip = strIP;
    srv.request.port = nPort;
    srv.request.reg_type = nRegType;
    srv.request.index = nRegIndex;
    srv.request.value = nRegValue;
    srv.request.slave_id = nSlaveID;

    if(srvConfigCreateModbus.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : config_create_modbus\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::config_delete_modbus(string strName)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvConfigDeleteModbus = node->serviceClient<dsr_msgs2::ConfigDeleteModbus>(m_strSrvNamePrefix + "/modbus/config_delete_modbus");
    dsr_msgs2::ConfigDeleteModbus srv;

    srv.request.name = strName;

    if(srvConfigDeleteModbus.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : config_delete_modbus\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::set_modbus_output(string strName,
                      int nValue)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvSetModbusOutput = node->serviceClient<dsr_msgs2::SetModbusOutput>(m_strSrvNamePrefix + "/modbus/set_modbus_output");
    dsr_msgs2::SetModbusOutput srv;

    srv.request.name = strName;
    srv.request.value = nValue;

    if(srvSetModbusOutput.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : set_modbus_output\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::get_modbus_input(string strName)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetModbusInput = node->serviceClient<dsr_msgs2::GetModbusInput>(m_strSrvNamePrefix + "/modbus/get_modbus_input");
    dsr_msgs2::GetModbusInput srv;

    srv.request.name = strName;
    if(srvGetModbusInput.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.value);
        return (srv.response.value);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_modbus_input\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

//-----------------------------------------------------------------------------------------------------------------------------------

int CDsrRobot::drl_start(int nRobotSystem, string strCode)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvDrlStart = node->serviceClient<dsr_msgs2::DrlStart>(m_strSrvNamePrefix + "/drl/drl_start");
    dsr_msgs2::DrlStart srv;

    srv.request.robotSystem = nRobotSystem;
    srv.request.code = strCode;

    if(srvDrlStart.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : drl_start\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::drl_stop(int nStopMode/* = STOP_TYPE_QUICK*/)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvDrlStop = node->serviceClient<dsr_msgs2::DrlStop>(m_strSrvNamePrefix + "/drl/drl_stop");
    dsr_msgs2::DrlStop srv;

    srv.request.stop_mode = nStopMode;

    if(srvDrlStop.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : drl_stop\n");
        ros::shutdown();  
        return -1;
    }
    return 0;    
}

int CDsrRobot::drl_pause()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvDrlPause = node->serviceClient<dsr_msgs2::DrlPause>(m_strSrvNamePrefix + "/drl/drl_pause");
    dsr_msgs2::DrlPause srv;

    if(srvDrlPause.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : drl_pause\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::drl_resume()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvDrlResume = node->serviceClient<dsr_msgs2::DrlResume>(m_strSrvNamePrefix + "/drl/drl_resume");
    dsr_msgs2::DrlResume srv;

    if(srvDrlResume.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.success);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : drl_resume\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

int CDsrRobot::get_drl_state(){
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetDrlState = node->serviceClient<dsr_msgs2::GetDrlState>(m_strSrvNamePrefix + "/drl/get_drl_state");
    dsr_msgs2::GetDrlState srv;

    if(srvGetDrlState.call(srv))
    {         
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        return (srv.response.drl_state);
    }
    else
    {    
        ROS_ERROR("Failed to call service dr_control_service : get_drl_state\n");
        ros::shutdown();  
        return -1;
    }
    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 사용자 node에서 수신 하도록 변경
void msgRobotState_cb(const dsr_msgs2::RobotState::ConstPtr& msg)
{
    static int sn_cnt =0;
    
    sn_cnt++;
    if(0==(sn_cnt % 100))
    {  
        ROS_INFO("________ ROBOT STATUS ________");
        ROS_INFO("  robot_state       : %d", msg->robot_state);
        ROS_INFO("  robot_state_str   : %s", msg->robot_state_str.c_str());
        ROS_INFO("  current_posj      :  %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",msg->current_posj[0] ,msg->current_posj[1] ,msg->current_posj[2]
                                                                             ,msg->current_posj[3] ,msg->current_posj[4] ,msg->current_posj[5] );
        ROS_INFO("  io_control_box    : %d", msg->io_control_box);
        //ROS_INFO("  io_modbus         : %d", msg->io_modbus);
        //ROS_INFO("  error             : %d", msg->error);
        ROS_INFO("  access_control    : %d", msg->access_control);
        ROS_INFO("  homming_completed : %d", msg->homming_completed);
        ROS_INFO("  tp_initialized    : %d", msg->tp_initialized);
        ROS_INFO("  speed             : %d", msg->speed);
        ROS_INFO("  mastering_need    : %d", msg->mastering_need);
        ROS_INFO("  drl_stopped       : %d", msg->drl_stopped);
        ROS_INFO("  disconnected      : %d", msg->disconnected);
    }
} 

void thread_subscriber()
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Subscriber subRobotState = node->subscribe(m_strTopicNamePrefix + "/state", 100, msgRobotState_cb);
    ///ros::spin();
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
}
*/

CDsrRobot::CDsrRobot(ros::NodeHandle nh, std::string robotID, std::string robotModel)
{
    // spawn another thread
    ///m_thread_sub = boost::thread(thread_subscriber);

    ROS_INFO("CDsrRobot:: %s, %s",robotID.c_str(),robotModel.c_str() );
    ROS_INFO("CDsrRobot:: %s, %s",robotID.c_str(),robotModel.c_str() );
    ROS_INFO("CDsrRobot:: %s, %s",robotID.c_str(),robotModel.c_str() );
    m_strSrvNamePrefix   = "/" + robotID + robotModel;
    m_strTopicNamePrefix = m_strSrvNamePrefix;

    ROS_INFO("CDsrRobot:: m_strSrvNamePrefix   = %s",m_strSrvNamePrefix.c_str() );
    ROS_INFO("CDsrRobot:: m_strTopicNamePrefix = %s",m_strTopicNamePrefix.c_str() );

}

CDsrRobot::~CDsrRobot()   
{
    ROS_INFO("CDsrRobot::~CDsrRobot()");
    ROS_INFO("CDsrRobot::~CDsrRobot()");
    ROS_INFO("CDsrRobot::~CDsrRobot()");

    // wait the second thread to finish
    ///m_thread_sub.join();
}
