#!/bin/bash
emulator_version="3.0.1"
emulator_image="doosanrobot/dsr_emulator:$emulator_version"

# Function to stop and remove container
stop_and_remove_container() {
    printf " Starting to stop and remove dsr_emulator"
    # We assumed each emulator container has 'emulator' in container name. 
    docker ps -a --filter name=emulator -q | xargs -r docker stop > /dev/null 2>&1
    docker ps -a --filter name=emulator -q | xargs -r docker rm > /dev/null 2>&1
    printf "Stop and remove Done !"
    # docker stop "$1" > /dev/null 2>&1
    # docker rm "$1" > /dev/null 2>&1
}

# Function to remove image
remove_image() {
    docker rmi "$1" > /dev/null 2>&1
    echo "\n Completed $1."
}

# Stop and remove container
stop_and_remove_container 

# Remove image
remove_image "$emulator_image"