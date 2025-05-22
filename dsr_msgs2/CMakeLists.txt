cmake_minimum_required(VERSION 3.5)
project(dsr_msgs2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  
  "msg/LogAlarm.msg"
  "msg/ModbusState.msg"
  "msg/RobotError.msg"
  "msg/RobotState.msg"
  "msg/RobotStop.msg"
  "msg/JogMultiAxis.msg"
  "msg/AlterMotionStream.msg"
  "msg/ServojStream.msg"
  "msg/ServolStream.msg"
  "msg/SpeedjStream.msg"
  "msg/SpeedlStream.msg"
  "msg/RobotDisconnection.msg"
  "msg/RobotStateRt.msg"
  "msg/ServojRtStream.msg"
  "msg/ServolRtStream.msg"
  "msg/SpeedjRtStream.msg"
  "msg/SpeedlRtStream.msg"
  "msg/TorqueRtStream.msg"

  "srv/system/SetRobotMode.srv"
  "srv/system/GetRobotMode.srv"
  "srv/system/SetRobotSystem.srv"
  "srv/system/GetRobotSystem.srv"
  "srv/system/SetRobotSpeedMode.srv"
  "srv/system/GetRobotSpeedMode.srv"
  "srv/system/GetCurrentPose.srv"
  "srv/system/SetSafeStopResetType.srv"
  "srv/system/GetLastAlarm.srv"
  "srv/system/GetRobotState.srv"
  "srv/system/ServoOff.srv"
  "srv/system/SetRobotControl.srv"
  "srv/system/ChangeCollisionSensitivity.srv"
  "srv/system/SetSafetyMode.srv"

  "srv/motion/MoveJoint.srv"
  "srv/motion/MoveLine.srv"
  "srv/motion/MoveJointx.srv"
  "srv/motion/MoveCircle.srv"
  "srv/motion/MoveSplineJoint.srv"
  "srv/motion/MoveSplineTask.srv"
  "srv/motion/MoveBlending.srv"
  "srv/motion/MoveSpiral.srv"
  "srv/motion/MovePeriodic.srv"
  "srv/motion/MoveWait.srv"
  "srv/motion/Jog.srv"
  "srv/motion/JogMulti.srv"
  "srv/motion/MoveStop.srv"
  "srv/motion/MovePause.srv"
  "srv/motion/MoveResume.srv"
  "srv/motion/Trans.srv"
  "srv/motion/Fkin.srv"
  "srv/motion/Ikin.srv"
  "srv/motion/SetRefCoord.srv"
  "srv/motion/MoveHome.srv"
  "srv/motion/CheckMotion.srv"
  "srv/motion/ChangeOperationSpeed.srv"
  "srv/motion/EnableAlterMotion.srv"
  "srv/motion/AlterMotion.srv"
  "srv/motion/DisableAlterMotion.srv"
  "srv/motion/SetSingularityHandling.srv"

  "srv/aux_control/GetControlMode.srv"              
  "srv/aux_control/GetControlSpace.srv"             
  "srv/aux_control/GetCurrentPosj.srv"             
  "srv/aux_control/GetDesiredPosj.srv"
  "srv/aux_control/GetCurrentVelj.srv"              
  "srv/aux_control/GetDesiredVelj.srv"              
  "srv/aux_control/GetCurrentPosx.srv"              
  "srv/aux_control/GetCurrentToolFlangePosx.srv"    
  "srv/aux_control/GetCurrentVelx.srv"              
  "srv/aux_control/GetDesiredPosx.srv"
  "srv/aux_control/GetDesiredVelx.srv"             
  "srv/aux_control/GetCurrentSolutionSpace.srv"     
  "srv/aux_control/GetCurrentRotm.srv"              
  "srv/aux_control/GetJointTorque.srv"              
  "srv/aux_control/GetExternalTorque.srv"          
  "srv/aux_control/GetToolForce.srv"                
  "srv/aux_control/GetSolutionSpace.srv"
  "srv/aux_control/GetOrientationError.srv"

  "srv/force/ParallelAxis1.srv"
  "srv/force/ParallelAxis2.srv"
  "srv/force/AlignAxis1.srv"
  "srv/force/AlignAxis2.srv"
  "srv/force/IsDoneBoltTightening.srv"
  "srv/force/ReleaseComplianceCtrl.srv"
  "srv/force/TaskComplianceCtrl.srv"
  "srv/force/SetStiffnessx.srv"
  "srv/force/CalcCoord.srv"
  "srv/force/SetUserCartCoord1.srv"
  "srv/force/SetUserCartCoord2.srv"
  "srv/force/SetUserCartCoord3.srv"
  "srv/force/OverwriteUserCartCoord.srv"
  "srv/force/GetUserCartCoord.srv"
  "srv/force/SetDesiredForce.srv"
  "srv/force/ReleaseForce.srv"
  "srv/force/CheckPositionCondition.srv"
  "srv/force/CheckForceCondition.srv"
  "srv/force/CheckOrientationCondition1.srv"
  "srv/force/CheckOrientationCondition2.srv"
  "srv/force/CoordTransform.srv"
  "srv/force/GetWorkpieceWeight.srv"
  "srv/force/ResetWorkpieceWeight.srv"
    
  "srv/tool/ConfigCreateTool.srv"
  "srv/tool/ConfigDeleteTool.srv"
  "srv/tool/SetCurrentTool.srv"
  "srv/tool/GetCurrentTool.srv"
  "srv/tool/SetToolShape.srv"

  "srv/tcp/ConfigCreateTcp.srv"
  "srv/tcp/ConfigDeleteTcp.srv"
  "srv/tcp/SetCurrentTcp.srv"
  "srv/tcp/GetCurrentTcp.srv"

  "srv/io/SetToolDigitalOutput.srv"
  "srv/io/GetToolDigitalOutput.srv"
  "srv/io/GetToolDigitalInput.srv"
  "srv/io/SetCtrlBoxDigitalOutput.srv"
  "srv/io/GetCtrlBoxDigitalOutput.srv" 
  "srv/io/GetCtrlBoxDigitalInput.srv"
  "srv/io/SetCtrlBoxAnalogInputType.srv"
  "srv/io/SetCtrlBoxAnalogOutputType.srv"
  "srv/io/SetCtrlBoxAnalogOutput.srv"
  "srv/io/GetCtrlBoxAnalogInput.srv"
    
  "srv/modbus/ConfigCreateModbus.srv"
  "srv/modbus/ConfigDeleteModbus.srv"
  "srv/modbus/SetModbusOutput.srv"
  "srv/modbus/GetModbusInput.srv"

  "srv/drl/DrlStart.srv"
  "srv/drl/DrlStop.srv"
  "srv/drl/DrlPause.srv"
  "srv/drl/DrlResume.srv"
  "srv/drl/GetDrlState.srv"

  "srv/gripper/Robotiq2FClose.srv"
  "srv/gripper/Robotiq2FOpen.srv"
  "srv/gripper/Robotiq2FMove.srv"
  "srv/gripper/SerialSendData.srv"
  

  "srv/realtime/ConnectRtControl.srv"
  "srv/realtime/DisconnectRtControl.srv"
  "srv/realtime/GetRtControlInputDataList.srv"
  "srv/realtime/GetRtControlInputVersionList.srv"
  "srv/realtime/GetRtControlOutputDataList.srv"
  "srv/realtime/GetRtControlOutputVersionList.srv"
  "srv/realtime/ReadDataRt.srv"
  "srv/realtime/SetAccjRt.srv"
  "srv/realtime/SetAccxRt.srv"
  "srv/realtime/SetRtControlInput.srv"
  "srv/realtime/SetRtControlOutput.srv"
  "srv/realtime/SetVeljRt.srv"
  "srv/realtime/SetVelxRt.srv"
  "srv/realtime/StartRtControl.srv"
  "srv/realtime/StopRtControl.srv"
  "srv/realtime/WriteDataRt.srv"

  DEPENDENCIES builtin_interfaces std_msgs  
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
