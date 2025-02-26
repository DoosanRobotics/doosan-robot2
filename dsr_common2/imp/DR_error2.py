#-*- coding: utf-8 -*-

# ##
# @file     DR_error.py
# @brief    Define ROS2 DRL error constants, DR_Error2 class
# @author   kabdol2<kabkyoum.kim@doosan.com>
# @version  0.10
# @Last update date     2018-11-19
# @details
#
# 0.10      : add error type
#             - Python extension error  : PY_EXT_RET_INVALID_MODBUS_NAME
#             - DR_ERROR                : DR_ERROR_INVALID_MODBUS_NAME
#             - exec_script return      : PY_EXEC_RET_ERR_INVALID_MODBUS_NAME
import rclpy
import inspect
import sys
import traceback

# C extension result
PY_EXT_RET_OK = 0
PY_EXT_RET_ERROR = -1
PY_EXT_RET_STOP = -2
PY_EXT_RET_PAUSE = -3
PY_EXT_RET_SKIP = -4    #OPERATION_VIRTUAL 인 경우, 응답을 기다리지 않고 SKIP 함 add by kabdol2 2017/08/17
                        #관련 명령: set_digital_output, set_analog_output, set_tool_digital_output, set_modbus_output

# DR error type
DR_ERROR_TYPE = 1000
DR_ERROR_VALUE = 1001
DR_ERROR_RUNTIME = 1002
DR_ERROR_STOP = 1003
DR_ERROR_INVALID_MODBUS_NAME = 1010

# script syntax checking result and script execution result
CHECK_SCRIPT_OK = 0
EXEC_SCRIPT_OK = 0

EXEC_SCRIPT_RET_ERR_SYNTAX = 100
EXEC_SCRIPT_RET_ERR_RUNTIME = 101
EXEC_SCRIPT_RET_ERR_EXCEPTION = 102

EXEC_SCRIPT_ERR_DR_TYPE = DR_ERROR_TYPE
EXEC_SCRIPT_ERR_DR_VALUE = DR_ERROR_VALUE
EXEC_SCRIPT_ERR_DR_RUNTIME = DR_ERROR_RUNTIME
EXEC_SCRIPT_ERR_DR_STOP = DR_ERROR_STOP
EXEC_SCRIPT_ERR_DR_INVALID_MODBUS_NAME = DR_ERROR_INVALID_MODBUS_NAME

# =============================================================================================
##
# @brief      class for DRL error
# @details    DRL error에 대한 정보를 관리하는 class
#
class DR_Error(Exception):
    ##
    # @brief      생성자
    # @details    DRL error type, msg 등을 입력받아 DRL error 객체를 초기화한다.
    #             DRL error line number, function name은 stack frame 정보에 의해 계산된다.
    # @param      type - DRL error type
    #               - DR_ERROR_TYPE                 1000 : DRL parameter type error
    #               - DR_ERROR_VALUE                1001 : DRL parameter value error
    #               - DR_ERROR_RUNTIME              1002 : DRL runtime error
    #               - DR_ERROR_STOP                 1003 : stopped
    # @param      msg - DRL error message
    # @param      back - DRL error의 발생 위치
    #               - True : stack frame [1]
    #               - False : stack frame [2]
    # @return     없음
    # @exception  없음
    #
    def __init__(self, type, msg="", back=False):
        # (frame, filename, line_number,
        # function_name, lines, index) = inspect.getouterframes(inspect.currentframe())[1]

        self.type = type
        self.msg = msg

        if back == False:
            self.lineno = inspect.getouterframes(inspect.currentframe())[1][2]
            self.funcname = inspect.getouterframes(inspect.currentframe())[1][3]
        else:
            self.lineno = inspect.getouterframes(inspect.currentframe())[2][2]
            self.funcname = inspect.getouterframes(inspect.currentframe())[2][3]

        print(self.funcname, self.lineno)
        err_msg = "[ERROR] <DSR_ROBOT.py> " + "func_name = "+str(self.funcname) +", "+ "line_no = "+str(self.lineno)
        print(err_msg)
        #rospy.signal_shutdown(err_msg)
        rclpy.shutdown()

        # ....
        '''
        exc_type, exc_value, exc_traceback = sys.exc_info()

        traceback_details = {
            'filename': exc_traceback.tb_frame.f_code.co_filename,
            'lineno': exc_traceback.tb_lineno,
            'name': exc_traceback.tb_frame.f_code.co_name,
            'type': exc_type.__name__,
            # 'message': exc_value.message,  # or see traceback._some_str()
            'message': str(exc_value),  # or see traceback._some_str()
        }
        '''
