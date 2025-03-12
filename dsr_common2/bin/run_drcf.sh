#!/bin/bash

echo "Run Emulator of the Doosan Robot Controller"

#echo "Total Param = $#, PROG: $0, param1 =$1, param1 =$2"
#$1 = server port : 12345 
#$2 = Robot model      : m0609, m0617, m1013, m1509   
emulator_version="3.0.1"
emulator_image="doosanrobot/dsr_emulator:$emulator_version"

server_port=$1

echo "dirname:" "$0" 
echo "server_port:" "$server_port" 
echo "robot model:" "${2^^} " 
echo "ns:" "$3"


cd "$(dirname "$0")"
# Doosan emulator name
# We need to use 'emulator' as suffix. some parts addresses it to detect or delete containers.
# TODO(leeminju) namespace mangling conversion needed ("/" division error prone at container name.)
container_name=emulator
if [ -n "$3" ];  then
    container_name="$3_""$container_name"
fi
echo "run : '$container_name' ..."



# check 
if [ "$(docker ps -q -f name=$container_name)" ]; then
    echo "The emulator '$container_name' is already running... kill it"
    docker ps -a --filter name=emulator -q | xargs -r docker rm -f
fi
# run
docker run -dit --privileged --rm --name $container_name --env ROBOT_MODEL=${2^^} -p $server_port:12345 $emulator_image


if [ `getconf LONG_BIT` = "64" ]
then
    echo "ARCH: 64-bit"
    # ./DRCF64 $1 $2 
else
    echo "ARCH: 32-bit"
    # ./DRCF32 $1 $2
fi
