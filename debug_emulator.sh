#!/bin/bash

echo "=== Emulator Debug Script ==="

# Check Docker version
echo "Docker version:"
docker --version

# Check if emulator image exists
echo -e "\nEmulator image status:"
docker images | grep dsr_emulator

# Check running containers
echo -e "\nRunning containers:"
docker ps

# Check all containers with emulator in name
echo -e "\nAll emulator containers:"
docker ps -a --filter name=emulator

# Check port usage
echo -e "\nPort 12345 usage:"
netstat -tulpn | grep :12345 || echo "Port 12345 is free"

echo -e "\nPort 12346 usage:"
netstat -tulpn | grep :12346 || echo "Port 12346 is free"

# Test Docker connectivity
echo -e "\nTesting Docker connectivity:"
docker run --rm hello-world

echo -e "\n=== Debug Complete ==="