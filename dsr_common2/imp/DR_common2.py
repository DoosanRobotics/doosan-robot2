# -*- coding: utf-8 -*-

# ##
# @file     DR_common2.py
# @brief    define ROS2 DRL common functions
# @author   kabdol2<kabkyoum.kim@doosan.com>
# @version  0.10
# @Last update date     2021-02-15
# @details
#


import collections
import collections.abc
import numpy            #for ROS2 add type ndarray 
from DR_error2 import *

# point count
POINT_COUNT = 6

# posb seg_type
DR_LINE = 0
DR_CIRCLE = 1

# =============================================================================================
##
# @brief      class for joint space position
# @details    Joint space position의 좌표 정보를 관리한다.
#
class posj(list):
    ##
    # @brief      생성자
    # @details    Joint space position 좌표 정보를 입력받아서, list[6] 데이터로 관리하며 객체를 초기화한다.
    # @param      q1 - 1축 angle 또는 join space position 좌표 정보 (list[6] - float)
    # @param      q2 - 2축 angle
    # @param      q3 - 3축 angle
    # @param      q4 - 4축 angle
    # @param      q5 - 5축 angle
    # @param      q6 - 6축 angle
    # @return     없음
    # @exception  - DR_ERROR_TYPE : argument의 type 비정상
    #
    def __init__(self, q1=0, q2=0, q3=0, q4=0, q5=0, q6=0):
        if (type(q1) == list and len(q1) == POINT_COUNT) or (type(q1) == posj):
            org_list = q1

            q1 = org_list[0]
            q2 = org_list[1]
            q3 = org_list[2]
            q4 = org_list[3]
            q5 = org_list[4]
            q6 = org_list[5]

        pos_list = [q1, q2, q3, q4, q5, q6]

        if is_number(pos_list) != True:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : q1, q2, q3, q4, q5, q6.")

        list.__init__(self, pos_list)
        # print (pos_list)

    ##
    # @brief      str method (문자열 변환)
    # @details    Joint space position 정보를 string type으로 변환하여 리턴한다. (data는 소숫점 3자리까지 표현)
    # @return     없음
    # @exception  없음
    #
    def __str__(self):
        str = "[" + ", ".join("{0:0.3f}".format(item) for item in self) + "]"

        return str

# =============================================================================================
##
# @brief      class for task space position
# @details    Task space position의 좌표 정보를 관리한다.
#
class posx(list):
    ##
    # @brief      생성자
    # @details    Task space position 좌표 정보를 입력받아서, list[6] 데이터로 관리하며 객체를 초기화한다.
    # @param      x - x position 또는 task space position 좌표 정보(list[6] - float)
    # @param      y - y position
    # @param      z - z position
    # @param      w - w orientation
    # @param      p - p orientation
    # @param      r - r orientation
    # @return     없음
    # @exception  - DR_ERROR_TYPE : argument의 type 비정상
    #
    def __init__(self, x=0, y=0, z=0, w=0, p=0, r=0):
        if (type(x) == list and len(x) == POINT_COUNT) or type(x) == posx:
            org_list = x

            x = org_list[0]
            y = org_list[1]
            z = org_list[2]
            w = org_list[3]
            p = org_list[4]
            r = org_list[5]

        pos_list = [x, y, z, w, p, r]

        if is_number(pos_list) != True:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : x, y, z, w, p, r.")

        list.__init__(self, pos_list)
        # print (pos_list)

    ##
    # @brief      str method (문자열 변환)
    # @details    task space position 정보를 string type으로 변환하여 리턴한다. (data는 소숫점 3자리까지 표현)
    # @return     없음
    # @exception  없음
    #
    def __str__(self):
        str = "[" + ", ".join("{0:0.3f}".format(item) for item in self) + "]"

        return str

# =============================================================================================
##
# @brief      class for segment (Segment는 moveb 수행시 입력 값으로 전달된다.)
# @details    Segment 객체의 정보를 관리한다.
#
class posb():
    ##
    # @brief      생성자
    # @details    Segment 객체의 정보를 입력받아서 관리하며 객체를 초기화한다.
    # @param      seg_type - segment type
    #               DR_LINE     0 : line
    #               DR_CIRCLE   1 : circle
    # @param      posx1 - position 1 for DR_LINE, DR_CIRCLE
    # @param      posx2 - position 2 for DR_CIRCLE
    # @param      radius - blending radius
    # @return     없음
    # @exception  - DR_ERROR_TYPE : argument의 type 비정상
    #             - DR_ERROR_VALUE : argument의 value 비정상
    #
    def __init__(self, seg_type, posx1, posx2=None, radius=0):
        # seg_type
        if type(seg_type) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : seg_type")

        if seg_type != DR_LINE and seg_type != DR_CIRCLE:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : seg_type")

        self._seg_type = seg_type

        # posx1
        self._posx1 = get_posx(posx1)

        # posx2
        self._posx2 = None
        if seg_type == DR_LINE:
            if posx2 != None:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : pos2")
        elif seg_type == DR_CIRCLE:
            if posx2 == None:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : pos2")
            else:
                self._posx2 = get_posx(posx2)

        # radius
        if type(radius) != int and type(radius) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : radius")

        if radius < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : radius")

        self._radius = radius

    ##
    # @brief      str method (문자열 변환)
    # @details    Segment 정보를 string type으로 변환하여 리턴한다. (data는 소숫점 3자리까지 표현)
    # @return     없음
    # @exception  없음
    #
    def __str__(self):
        val_list = []

        val_list.append(self._seg_type)
        val_list.append(self._posx1)
        val_list.append(self._posx2)
        val_list.append(self._radius)

        # print(val_list)
        return dr_form(val_list)

    ##
    # @brief      Segment 정보를 list 형태로 변환하여 리턴한다.
    # @return     없음
    # @exception  없음
    #
    def to_list(self):
        val_list = []

        val_list.append(self._seg_type)
        val_list.append(self._posx1)
        val_list.append(self._posx2)
        val_list.append(self._radius)

        return val_list

# =============================================================================================
##
# @brief      pos를 posj instance로 반환한다.
#             pos가 posj로 변환할 수 없을 경우에는 exception을 발생시킨다.
# @details    처리 방식 : python 내부 함수
# @param      pos : posj type으로 변환할 data
# @return     변환된 posj instance
#             pos가 posj인 경우에는 변환하지 않고 pos를 리턴한다.
# @exception  - DR_ERROR_TYPE : argument의 ype 비정상
#             - DR_ERROR_VALUE : argument의 value 비정상
#
def get_posj(pos):
    if type(pos) == posj:
        if is_number(pos) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : pos", True)
        return pos
    elif type(pos) == list and len(pos) == POINT_COUNT:
        if is_number(pos) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : pos", True)
        return posj(pos)
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos", True)

# =============================================================================================
##
# @brief      pos를 posx instance로 반환한다.
#             pos가 posx로 변환할 수 없을 경우에는 exception을 발생시킨다.
# @details    처리 방식 : python 내부 함수
# @param      pos : posx type으로 변환할 data
# @return     변환된 posx instance
#             pos가 posx인 경우에는 변환하지 않고 pos를 리턴한다.
# @exception  - DR_ERROR_TYPE : argument의의 type 비정상
#             - DR_ERROR_VALUE : argument의의 value 비정상
#
def get_posx(pos):
    if type(pos) == posx:
        if is_number(pos) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : pos", True)
        return pos
    elif type(pos) == list and len(pos) == POINT_COUNT:
        if is_number(pos) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : pos", True)
        return posx(pos)
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos", True)

# =============================================================================================
##
# @brief      pos를 posj 또는 posx로 instance로 반환한다.
#             pos가 posj 또는 posx로 변환할 수 없을 경우에는 exception을 발생시킨다.
# @details    처리 방식 : python 내부 함수
# @param      pos : posj 또는 posx type으로 변환할 data
# @param      def_type : pos가 posj 또는 posx가 아닌 경우, def_type에 의해 지정된 type으로 변환한다.
# @return     변환된 posj 또는 posx instance
#             - posj instance : pos가 posj type이거나, def_type = posj인 경우
#             - posx instance : pos가 posx type이거나, def_type = posx인 경우
# @exception  - DR_ERROR_TYPE : argument의 type 비정상
#             - DR_ERROR_VALUE : argument의 value 비정상
#
def get_normal_pos(pos, def_type=None):
    if type(pos) == posx or type(pos) == posj or type(pos) == numpy.ndarray: #for ROS2 add type ndarray  
        if is_number(pos) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : pos", True)
        return pos
    elif type(pos) == list and len(pos) == POINT_COUNT:
        if is_number(pos) != True:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : pos", True)

        if (def_type == posj):
            return posj(pos)
        elif (def_type == posx):
            return posx(pos)
        else:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos", True)
    else:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : pos", True)

# =============================================================================================
##
# @brief      val을 string type으로 변환하여 리턴한다.
# @details    python 내부 함수
# @param      val : 변환할 데이터
# @return     val이 변환된 string
#             - 숫자 type(정수 또는 부동수소점) : 소수점 3자리까지 변환
#             - None : "None"
#             - 그 외 : str(val)
# @exception  없음
#
def dr_form(val):
    str_val = ""

    # if type(val) == list or type(val) == posj or type(val) == posx:
    if isinstance(val, list) == True:
        str_val = "["

        for item in val:
            if str_val != "[":
                str_val += ", "

            if isinstance(item, collections.abc.Iterable):
                str_val += dr_form(item)
            elif type(item) == int or type(item) == float:
                str_val += "{0:0.3f}".format(item)
            elif item == None:
                str_val += "None"
            else:
                str_val += str(val)

        str_val += "]"
    else:
        if type(val) == int or type(val) == float:
            str_val = "{0:0.3f}".format(val)
        elif val == None:
            str_val = "None"
        else:
            str_val = str(val)

    return str_val

# =============================================================================================
##
# @brief      x, y 값 중에서 None이 아닌 값을 리턴한다.
# @details    python 내부 함수
# @param      x
# @param      y
# @return     - x     : x가 None이 아닐 경우
#             - y     : y가 None이 아닐 경우
#             - None  : x, y 모두 None일 경우
# @exception  없음
#
get_param = lambda x, y: x if x != None else y if y != None else None

# =============================================================================================
##
# @brief      key=value 인수 목록에서 key가 name인 argument의 value를 리턴한다.
# @details    처리 방식 : python 내부 함수
# @param      kargs : key-value argument list
# @param      name : argument key name
# @return     - key list에 name이 있는 경우 : kargs[name]
#             - key list에 name이 없는 경우 : None
# @exception  없음
#
def get_kargs(kargs, name):
    if name in kargs.keys():
        return kargs[name]
    else:
        return None

# =============================================================================================
##
# @brief      data의 값이 숫자 type(정수, 부동소수점)로 구성되어 있는 지 여부를 리턴한다.
#             data가 iterable type인 경우에는 내부 item까지 검사한다.
# @details    처리 방식 : python 내부 함수
# @param      data : 숫자 type을 검사할 data
# @return     - True : 숫자 type
#             - False : 숫자 type이 아님
# @exception  없음
#
def is_number(data):
    try:
        if isinstance(data, collections.abc.Iterable):
            for item in data:
                if type(item) == str:
                    return False
                else:
                    return is_number(item)
        else:
            if type(data) == str:
                return False
            else:
                float(data)
        return True
    except ValueError:
        return False


# =============================================================================================

if __name__ == "__main__":
    print(dr_form(1))
    print(dr_form(1.01234))
    print(dr_form("aaaa"))

    l = [1, 2.3, 4, 5.00001]
    print(dr_form(l))

    print(is_number(l))