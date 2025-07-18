#! /bin/bash

container_name="roboracer_sandbox"

cd $ROBORACER_SANDBOX_ROOT

# stop and remove docker container
docker stop $container_name
echo "Docker container '$container_name' stopped."
docker rm $container_name
echo "Docker container '$container_name' removed."

# remove volumes to clean ws 
sudo rm -rf ../cache/roboracer_sandbox/build/*
echo "Volume '../cache/roboracer_sandbox/build' removed."

sudo rm -rf ../cache/roboracer_sandbox/install/*
echo "Volume '../cache/roboracer_sandbox/install' removed."

sudo rm -rf ../cache/roboracer_sandbox/log*
echo "Volume '../cache/roboracer_sandbox/log' removed."

echo "Please restart the docker container!"