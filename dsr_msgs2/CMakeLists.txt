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

  "msg/RobotStateRt.msg"
  "msg/ServojRtStream.msg"
  "msg/ServolRtStream.msg"
  "msg/SpeedjRtStream.msg"
  "msg/SpeedlRtStream.msg"
  "msg/TorqueRtStream.msg"

  "srv/SetRobotMode.srv"
  "srv/GetRobotMode.srv"
  "srv/SetRobotSystem.srv"
  "srv/GetRobotSystem.srv"
  "srv/SetRobotSpeedMode.srv"
  "srv/GetRobotSpeedMode.srv"
  "srv/GetCurrentPose.srv"
  "srv/SetSafeStopResetType.srv"
  "srv/GetLastAlarm.srv"
  "srv/GetRobotState.srv"
  "srv/ServoOff.srv"
  "srv/SetRobotControl.srv"
  "srv/ChangeCollisionSensitivity.srv"
  "srv/SetSafetyMode.srv"

  "srv/MoveJoint.srv"
  "srv/MoveLine.srv"
  "srv/MoveJointx.srv"
  "srv/MoveCircle.srv"
  "srv/MoveSplineJoint.srv"
  "srv/MoveSplineTask.srv"
  "srv/MoveBlending.srv"
  "srv/MoveSpiral.srv"
  "srv/MovePeriodic.srv"
  "srv/MoveWait.srv"
  "srv/Jog.srv"
  "srv/JogMulti.srv"
  "srv/MoveStop.srv"
  "srv/MovePause.srv"
  "srv/MoveResume.srv"
  "srv/Trans.srv"
  "srv/Fkin.srv"
  "srv/Ikin.srv"
  "srv/SetRefCoord.srv"
  "srv/MoveHome.srv"
  "srv/CheckMotion.srv"
  "srv/ChangeOperationSpeed.srv"
  "srv/EnableAlterMotion.srv"
  "srv/AlterMotion.srv"
  "srv/DisableAlterMotion.srv"
  "srv/SetSingularityHandling.srv"

  "srv/GetControlMode.srv"              
  "srv/GetControlSpace.srv"             
  "srv/GetCurrentPosj.srv"             
  "srv/GetDesiredPosj.srv"
  "srv/GetCurrentVelj.srv"              
  "srv/GetDesiredVelj.srv"              
  "srv/GetCurrentPosx.srv"              
  "srv/GetCurrentToolFlangePosx.srv"    
  "srv/GetCurrentVelx.srv"              
  "srv/GetDesiredPosx.srv"
  "srv/GetDesiredVelx.srv"             
  "srv/GetCurrentSolutionSpace.srv"     
  "srv/GetCurrentRotm.srv"              
  "srv/GetJointTorque.srv"              
  "srv/GetExternalTorque.srv"          
  "srv/GetToolForce.srv"                
  "srv/GetSolutionSpace.srv"
  "srv/GetOrientationError.srv"

  "srv/ParallelAxis1.srv"
  "srv/ParallelAxis2.srv"
  "srv/AlignAxis1.srv"
  "srv/AlignAxis2.srv"
  "srv/IsDoneBoltTightening.srv"
  "srv/ReleaseComplianceCtrl.srv"
  "srv/TaskComplianceCtrl.srv"
  "srv/SetStiffnessx.srv"
  "srv/CalcCoord.srv"
  "srv/SetUserCartCoord1.srv"
  "srv/SetUserCartCoord2.srv"
  "srv/SetUserCartCoord3.srv"
  "srv/OverwriteUserCartCoord.srv"
  "srv/GetUserCartCoord.srv"
  "srv/SetDesiredForce.srv"
  "srv/ReleaseForce.srv"
  "srv/CheckPositionCondition.srv"
  "srv/CheckForceCondition.srv"
  "srv/CheckOrientationCondition1.srv"
  "srv/CheckOrientationCondition2.srv"
  "srv/CoordTransform.srv"
  "srv/GetWorkpieceWeight.srv"
  "srv/ResetWorkpieceWeight.srv"
    
  "srv/ConfigCreateTool.srv"
  "srv/ConfigDeleteTool.srv"
  "srv/SetCurrentTool.srv"
  "srv/GetCurrentTool.srv"
  "srv/SetToolShape.srv"

  "srv/ConfigCreateTcp.srv"
  "srv/ConfigDeleteTcp.srv"
  "srv/SetCurrentTcp.srv"
  "srv/GetCurrentTcp.srv"

  "srv/SetToolDigitalOutput.srv"
  "srv/GetToolDigitalOutput.srv"
  "srv/GetToolDigitalInput.srv"
  "srv/SetCtrlBoxDigitalOutput.srv"
  "srv/GetCtrlBoxDigitalOutput.srv" 
  "srv/GetCtrlBoxDigitalInput.srv"
  "srv/SetCtrlBoxAnalogInputType.srv"
  "srv/SetCtrlBoxAnalogOutputType.srv"
  "srv/SetCtrlBoxAnalogOutput.srv"
  "srv/GetCtrlBoxAnalogInput.srv"
    
  "srv/ConfigCreateModbus.srv"
  "srv/ConfigDeleteModbus.srv"
  "srv/SetModbusOutput.srv"
  "srv/GetModbusInput.srv"

  "srv/DrlStart.srv"
  "srv/DrlStop.srv"
  "srv/DrlPause.srv"
  "srv/DrlResume.srv"
  "srv/GetDrlState.srv"

  "srv/Robotiq2FClose.srv"
  "srv/Robotiq2FOpen.srv"
  "srv/Robotiq2FMove.srv"
  "srv/SerialSendData.srv"
  

  "srv/ConnectRtControl.srv"
  "srv/DisconnectRtControl.srv"
  "srv/GetRtControlInputDataList.srv"
  "srv/GetRtControlInputVersionList.srv"
  "srv/GetRtControlOutputDataList.srv"
  "srv/GetRtControlOutputVersionList.srv"
  "srv/ReadDataRt.srv"
  "srv/SetAccjRt.srv"
  "srv/SetAccxRt.srv"
  "srv/SetRtControlInput.srv"
  "srv/SetRtControlOutput.srv"
  "srv/SetVeljRt.srv"
  "srv/SetVelxRt.srv"
  "srv/StartRtControl.srv"
  "srv/StopRtControl.srv"
  "srv/WriteDataRt.srv"

  DEPENDENCIES builtin_interfaces std_msgs  
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
