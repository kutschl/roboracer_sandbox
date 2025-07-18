#!/bin/bash

# Function to display help
function show_help() {
    echo "Usage: select_car [car name]"
    echo "Available cars (with ROS domains):"
    echo "  alonso      (14)"
    echo "  leclerc     (16)"
    echo "  rosberg      (6)"
    echo "  vettel       (5)"
    echo "  localhost   (42)"
    exit 1
}

# Function to select car and set ROS_DOMAIN_ID
function select_car() {
    local carName=$1
    local rosDomainId
    #local carIp

    case "$carName" in
        alonso) 
            rosDomainId=14
            #carIp=192.168.1.11
            ;;
        leclerc) 
            rosDomainId=16
            #carIp=192.168.1.16
            ;;
        rosberg)
            rosDomainId=6
            #carIp=192.168.1.6
            ;;
        vettel)
            rosDomainId=5
            #carIp=192.168.1.5
            ;;
        localhost) 
            rosDomainId=42
            ;;
        *) 
            echo "Unknown car name: $carName. Use 'select_car help' for valid options."
            exit 1
            ;;
    esac

    # Update ROS_DOMAIN_ID in ~/.bashrc
    if grep -q "export ROS_DOMAIN_ID=" ~/.bashrc; then
        sed -i "s/export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID=$rosDomainId/" ~/.bashrc 
    else
        echo "export ROS_DOMAIN_ID=$rosDomainId" >> ~/.bashrc 
    fi

        # Update ROS_DOMAIN_ID in ~/.bashrc
    #if grep -q "export ROS_DISCOVERY_SERVER=" ~/.bashrc; then
    #    sed -i "s/export ROS_DISCOVERY_SERVER=.*/export ROS_DISCOVERY_SERVER=$carIp:11811/" ~/.bashrc
    #else
    #    echo "export ROS_DISCOVERY_SERVER=$carIp:11811" >> ~/.bashrc 
    #fi

    # Update LARACE_CAR_VERSION in ~/.bashrc
    if grep -q "export LARACE_CAR_VERSION=" ~/.bashrc; then
        sed -i "s/export LARACE_CAR_VERSION=.*/export LARACE_CAR_VERSION=$carName/" ~/.bashrc 
    else
        echo "export LARACE_CAR_VERSION=$carName" >> ~/.bashrc 
    fi

    # Source the updated ~/.bashrc
    source ~/.bashrc > /dev/null 2>&1
}

# Check if no arguments are provided or if help is requested
if [ $# -eq 0 ] || [ "$1" == "help" ] || [ "$1" == "--help" ] || [ "$1" == "-h" ] || [ "$1" == "list" ] || [ "$1" == "-l" ]; then
    show_help
fi

# Check if the current user is allowed to change ROS_DOMAIN_ID
if [ "$USER" == "alonso" ] || [ "$USER" == "leclerc" ] || [ "$USER" == "vettel" ] || [ "$USER" == "rosberg" ]; then
    echo "Permission denied: The car '$USER' is not allowed to change its static car settings."
    exit 1
fi

# Call select_car function with the provided argument
select_car "$1"


