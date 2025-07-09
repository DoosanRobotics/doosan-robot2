#-*- coding: utf-8 -*-

# ##
# @file     DL_tcp_client.py
# @brief    Implementation for DRL TCP communication
# @author   htjeong
# @version  0.01
# @Last update date     2017-03-19
# @details
#
# history
#
# 0.01      : 2017-06-08, htjeong
#           : first release
# <functions>
#  client_socket_open(ip, port)
#  client_socket_close(sock)
#  client_socket_state(sock)
#  client_socket_end_data(sock, end_data)
#  client_socket_write(sock, tx_data)
#  client_socket_read(sock, length=-1, timeout=-1)
#  client_socket_flush(sock)
#

import socket
import select
import time
#import ipaddress # 3.2 not supported

from DR_error2 import *

# =============================================================================================
# define

DR_TCP_CLIENT_CONNET_TIMEOUT = 1    #3
DR_TCP_CLIENT_COMM_TIMEOUT   = 0.01    #3
DR_TCP_CLIENT_DEF_TIMEOUT    = 0.01   #3
DR_TCP_CLIENT_BUFF_SIZE      = 4096   #1024

DR_TCP_CLIENT_CONN_LIST = dict()            #opened socket 저장 리스트
DR_TCP_CLIENT_CONN_STATE_LIST = dict()      #opened socket의 연결 상태 저장 리스트
DR_TCP_CLIENT_END_DATA = dict()

# =============================================================================================
##
# @brief      socket을 생성하고, server에 연결을 시도한다.
# @param      ip - ip(ip4) string
# @param      port - port number
# @return     socket : socket instance
# @exception  - DR_ERROR_TYPE : argument의 type 비정상
#             - DR_ERROR_VALUE : argument의 value 비정상
#             - DR_ERROR_RUNTIME : socket.error Exception 발생
#
def client_socket_open(ip, port):
    # ip
    if type(ip) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : ip")

    # 3.2 not supported
    # try:
    #    ipaddress.ip_address(ip)
    #except Exception as e:
    #    raise DR_Error(DR_ERROR_VALUE, "Invalid value : ip / ", e.args)

    # port
    if type(port) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : port")

    if port < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : port")

    while True:
        try:
            # create socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(DR_TCP_CLIENT_CONNET_TIMEOUT)   #1초

            # connect
            server_address = (ip, port)
            #------------------------------------------------------
            sock.connect(server_address)         #여기서 접속대기하다가 Time-out 발생하면, except socket.error as msg 로 점프됨
            #------------------------------------------------------

            # connected
            DR_TCP_CLIENT_CONN_LIST[id(sock)] = sock
            DR_TCP_CLIENT_CONN_STATE_LIST[id(sock)] = 1    #연결 상태 ON으로 설정 
            print("_____OPEN CLIENT SOCKET_____ : ", sock)
            sock.settimeout(DR_TCP_CLIENT_COMM_TIMEOUT)     #0.01초
            return sock

        except socket.error as msg:
            print("client_socket_open() Socket Error: "+str(msg))
            print(sock)
            #sock.shutdown(socket.SHUT_RDWR)
            sock.close()
            time.sleep(0.5)
            print("retry connecting...")
            continue
    
    return sock

# =============================================================================================
##
# @brief      server와의 통신을 종료한다.
# @param      sock - socket instance
# @return     0 : 성공
# @exception  - DR_ERROR_TYPE : argument의 type 비정상
#             - DR_ERROR_RUNTIME : socket.error Exception 발생
#
def client_socket_close(sock):
    try:
        # sock
        if type(sock) != socket.socket:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'sock' is not socket object.")

        #sock.shutdown(socket.SHUT_RDWR)
        print("_____CLOSE CLIENT SOCKET_____ : ", sock)
        sock.close()
        # del sock

        # disconnected
        del DR_TCP_CLIENT_CONN_STATE_LIST[id(sock)]
        del DR_TCP_CLIENT_CONN_LIST[id(sock)]

    except socket.error as msg:
        raise DR_Error(DR_ERROR_RUNTIME, "client_socket_close() Socket Error: ", msg)

    return 0

# =============================================================================================
##
# @brief      Open된 socket을 모두 종료한다.
# @return     None
#
def clean_client_socket():
    print("         clean_client_socket() call")

    for sock_id in list(DR_TCP_CLIENT_CONN_LIST.keys()):
        sock = DR_TCP_CLIENT_CONN_LIST[sock_id]

        # test
        client_socket_close(sock)

    DR_TCP_CLIENT_END_DATA.clear()

    return None

# =============================================================================================
##
# @brief      socket 상태를 리턴한다.
# @param      sock - socket instance
# @return     1 : connected
#             0 : disconnected
# @exception  - DR_ERROR_TYPE : argument의 type 비정상
#
def client_socket_state(sock):
    # sock
    if type(sock) != socket.socket:
        #raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'sock' is not socket object.")
        return 0

    # check opened
    if id(sock) in DR_TCP_CLIENT_CONN_LIST.keys():
        #return 1
        return DR_TCP_CLIENT_CONN_STATE_LIST[id(sock)]
    else:
        return 0

# =============================================================================================
##
# @brief      통신 데이터의 끝을 지정한다.
# @param      sock - socket instance
# @param      end_data - end data
# @return     0 : 성공
# @exception  - DR_ERROR_TYPE : argument의 type 비정상
#
def client_socket_end_data(sock, end_data):
    # sock
    if type(sock) != socket.socket:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'sock' is not socket object.")

    # end_data
    if type(end_data) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : end_data")

    # set
    DR_TCP_CLIENT_END_DATA[id(sock)] = end_data

    return 0

# =============================================================================================
##
# @brief      server에 data를 전송한다.
# @param      sock - socket instance
# @param      tx_data - 통신 데이터 (bytes 유형)
# @return     0 : 성공
#             -1 : connection is not alive
#             -2 : socket.error Exception 발생
# @exception  - DR_ERROR_TYPE : argument의 type 비정상
#
def client_socket_write(sock, tx_data):
    try:
        # sock
        if type(sock) != socket.socket:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'sock' is not socket object.")

        # data
        if type(tx_data) != bytes:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : tx_data")

        # check opened
        if id(sock) not in DR_TCP_CLIENT_CONN_LIST.keys():
            print("Connection is not alive!!")
            return -1

        #com_end_data = DR_TCP_CLIENT_END_DATA.get(id(sock), "")
        #sock.sendall(tx_data + bytes(com_end_data, 'ascii'))
        sock.sendall(tx_data)

    except socket.error as msg:
        print("client_socket_write() Socket Error: ", msg)
        return -2

    return 0

# =============================================================================================
##
# @brief      server로부터의 데이터를 수신한다.
# @param      sock - socket instance
# @param      length - 읽을 byte의 수
# @param      timeout - read timeout
#               . -1 : 무한 대기
#               . 0 : 버퍼의 데이터 읽기, 즉시 리턴
#               . > 0 : timeout
# @return     res, rx_data
#             - res
#               . > 0 : the count of byte read
#               . -1 : connection is not alive
#               . -2 : socket.error Exception 발생
#             - rx_data : (bytes) received data
#
# @exception  - DR_ERROR_TYPE : argument의 type 비정상
#             - DR_ERROR_VALUE : argument의 value 비정상
#
def client_socket_read(sock, length=-1, timeout=-1):
    rx_data = None
    time_cnt = 0
    rxd_size = 0

    # sock
    if type(sock) != socket.socket:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'sock' is not socket object.")

    # length
    if type(length) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : length")

    if length != -1 and length < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : max_length")

    # timeout
    #if type(timeout) != int:
    if type(timeout) != int and type(timeout) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : timeout")

    if timeout != -1 and timeout < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : timeout")

    # check opened
    if id(sock) not in DR_TCP_CLIENT_CONN_LIST.keys():
        print("Connection is not alive!!")
        return -1, None

    # check length
    if length == -1:
        rxd_size = DR_TCP_CLIENT_BUFF_SIZE
    else:
        rxd_size = length

    #---- READ DATA ----------------------------------------------------------
    while True:
        try:
            rxd = sock.recv(rxd_size)
        except socket.timeout as e:
            err = e.args[0]
            # this next if/else is a bit redundant, but illustrates how the
            # timeout exception is setup
            if err == "timed out": #타임아웃 
                if timeout == -1:   #무한대기 
                    #print("client_socket_read(): recv timed out, retry later")
                    continue               #무한 재시도
                else: 
                    time_cnt = time_cnt +1 
                    if (DR_TCP_CLIENT_DEF_TIMEOUT*time_cnt) >= timeout:
                        print("client_socket_read() time-out")
                        return -3, None    #타임아웃 발생 
                    else:
                        continue           #타임아웃 내 재시도
            else:
                print(e)
                print("client_socket_read(): except socket.error <1>")
                return -2, None
        except socket.error as e:  #소켓에러가 발생한 경우  
            # Something else happened, handle error, exit, etc.
            print(e)
            print("client_socket_read(): except socket.error <2>")
            return -2, None
        else:
            if len(rxd) == 0:      #서버가 끊긴 경우 
                print("client_socket_read(): server is disconnected")
                DR_TCP_CLIENT_CONN_STATE_LIST[id(sock)] = 0    # 연결 상태 OFF 로 설정
                return -1, None
            else:                  #정상적으로 데이터 수신 
                #print("OKOKOKOK!!!!!!!!!!!!!!!!!!")
                #print(rxd)
                rx_data = bytes(rxd)
                break

    return len(rx_data), rx_data

# =============================================================================================
##
# @brief      input/ output buffer를 flush한다.
# @param      sock - socket instance
# @return     0 : 성공
#             -1 : connection is not alive
#             -2 : socket.error Exception 발생
#             -3 : inpout/ output buffer 검사 중 오류 발생
#
# @exception  - DR_ERROR_TYPE : argument의 type 비정상
#
def client_socket_flush(sock):
    try:
        # sock
        if type(sock) != socket.socket:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'sock' is not socket object.")

        # check opened
        if id(sock) not in DR_TCP_CLIENT_CONN_LIST.keys():
            print("Connection is not alive!!")
            return -1

        # check ready
        while True:
            r_ready, w_ready, error = select.select([sock], [sock], [sock], 0.1)

            # check error
            if error:
                print("Failed to read!! (socket)")
                return -3

            # if read data...
            if r_ready:
                dummy = sock.recv(DR_TCP_CLIENT_BUFF_SIZE)
                continue

            # if write ready...
            if w_ready:
                break

            time.sleep(0.01)

    except socket.error as msg:
        print("client_socket_flush() Socket Error: ", msg)
        return -2

    return 0
