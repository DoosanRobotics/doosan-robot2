#
#  dsr_common2
#  Author: Minsoo Song (minsoo.song@doosan.com)
#
#  Copyright (c) 2025 Doosan Robotics
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

import rclpy
import inspect
import sys
import traceback

# C extension result
PY_EXT_RET_OK = 0
PY_EXT_RET_ERROR = -1
PY_EXT_RET_STOP = -2
PY_EXT_RET_PAUSE = -3
PY_EXT_RET_SKIP = -4    

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
class DR_Error(Exception):

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
