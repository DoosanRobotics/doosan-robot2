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


cd "$(dirname "$0")" || exit
# Doosan emulator name
# We need to use 'emulator' as suffix. some parts addresses it to detect or delete containers.
# TODO(leeminju) namespace mangling conversion needed ("/" division error prone at container name.)
container_name=emulator
if [ -n "$3" ];  then
    container_name="$3_""$container_name"
fi
echo "run : '$container_name' ..."



# check container
if [ "$(docker ps -q -f name=$container_name)" ]; then
    echo "The emulator '$container_name' is already running... kill it"
    docker ps -a --filter name=$container_name -q | xargs -r docker rm -f
fi

# function to get a disjoint cpu set
find_available_cpuset() {
    local cores_per_container=${1:-4}  # Default to 4 cores
    local container_pattern="_emulator"
    
    # Get currently used CPU sets by running containers
    local used_cpusets
    used_cpusets=$(docker ps --format "table {{.Names}}\t{{.ID}}" | grep "$container_pattern" | \
        awk '{print $2}' | xargs -I {} docker inspect {} --format '{{.HostConfig.CpusetCpus}}' | \
        grep -v '^$' | sort -u)
    
    # Convert used CPU sets to individual CPU numbers
    local used_cpus=()
    while IFS= read -r cpuset; do
        if [[ -n "$cpuset" ]]; then
            # Handle range format ("0-3")
            if [[ "$cpuset" =~ ^([0-9]+)-([0-9]+)$ ]]; then
                local start=${BASH_REMATCH[1]}
                local end=${BASH_REMATCH[2]}
                for ((i=start; i<=end; i++)); do
                    used_cpus+=($i)
                done
            # Handle comma-separated format ("0,1,2,3")
            elif [[ "$cpuset" =~ ^[0-9,]+$ ]]; then
                IFS=',' read -ra cpu_array <<< "$cpuset"
                used_cpus+=("${cpu_array[@]}")
            fi
        fi
    done <<< "$used_cpusets"
    
    # Get total number of CPUs
    local total_cpus
    total_cpus=$(nproc)
    
    # Find first available contiguous block
    local start_cpu=0
    while [ $start_cpu -lt $total_cpus ]; do
        local end_cpu=$((start_cpu + cores_per_container - 1))
        
        # Check if this range would exceed available CPUs
        if [ $end_cpu -ge $total_cpus ]; then
            echo "Error: Not enough CPUs available. Need $cores_per_container, but only $((total_cpus - start_cpu)) available starting from CPU $start_cpu" >&2
            return 1
        fi
        
        # Check if any CPU in this range is already used
        local range_available=true
        for ((cpu=start_cpu; cpu<=end_cpu; cpu++)); do
            local cpu_in_use=false
            for used_cpu in "${used_cpus[@]}"; do
                if [[ "$used_cpu" == "$cpu" ]]; then
                    cpu_in_use=true
                    break
                fi
            done
            if [ "$cpu_in_use" = true ]; then
                range_available=false
                break
            fi
        done
        
        if [ "$range_available" = true ]; then
            echo "${start_cpu}-${end_cpu}"
            return 0
        fi
        
        start_cpu=$((start_cpu + cores_per_container))
    done
    
    echo "Error: No available CPU set found for $cores_per_container cores" >&2
    return 1
}

# run
DOCKER_CPUS=$(find_available_cpuset 4)
docker run -dit --rm --name $container_name --env ROBOT_MODEL=${2^^} --cpuset-cpus "$DOCKER_CPUS" -p $server_port:12345 $emulator_image

#echo command
echo "docker run -dit --rm --name $container_name --env ROBOT_MODEL=${2^^} --cpuset-cpus $DOCKER_CPUS -p $server_port:12345 $emulator_image"


if [ `getconf LONG_BIT` = "64" ]
then
    echo "ARCH: 64-bit"
    # ./DRCF64 $1 $2 
else
    echo "ARCH: 32-bit"
    # ./DRCF32 $1 $2
fi
