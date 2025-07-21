#!/bin/bash

export LARACE_ROOT=$(pwd)

echo "LARACE_ROOT: ${LARACE_ROOT}"
echo "LARACE_CAR_VERSION: ${LARACE_CAR_VERSION}"
echo "LARACE_CURRENT_MAP: ${LARACE_CURRENT_MAP}"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"


function _larace_get_yaml() {
    local CONFIG_FILE="$1"
    local KEY_PATH="$2"

    python3 - <<EOF
import yaml, sys

with open("$CONFIG_FILE") as f:
    data = yaml.safe_load(f) or {}

value = data
for key in "$KEY_PATH".split('.'):
    if isinstance(value, dict) and key in value:
        value = value[key]
    else:
        value = ""
        break

print(value, end="")
EOF
}


function is_reachable() {
    local host=$1
    local count=1
    local timeout=3
    
    if ping -c $count -W $timeout $host &> /dev/null; then
        return 0
    else
        return 1
    fi
}


function select_map() {
    N_ARGS=$#

    if [ $N_ARGS -eq 0 ]; then
        echo "Usage: larace select_map | map [map_name]"
    else 
        local MAP_NAME="$1"
        local BASHRC_FILE="$HOME/.bashrc"
        local MAPS_DIR="$LARACE_ROOT/src/core/maps"

        # Check if the map exists
        if [ ! -d "$MAPS_DIR/$MAP_NAME" ]; then
            echo "Map '$MAP_NAME' not found. Available maps:"
            
            for dir in $MAPS_DIR/*/; do
                if [ -d "$dir" ]; then
                    echo "  $(basename "$dir")"
                fi
            done
            
        else
            if grep -q "export LARACE_CURRENT_MAP=" ~/.bashrc; then
                sed -i.bak "s|^export LARACE_CURRENT_MAP=.*|export LARACE_CURRENT_MAP=${MAP_NAME}|" "$BASHRC_FILE"
            else
                echo "export LARACE_CURRENT_MAP=$MAP_NAME" >> ~/.bashrc
            fi

            source ~/.bashrc > /dev/null 2>&1

            echo "LARACE_CURRENT_MAP: ${LARACE_CURRENT_MAP}"
        fi
    fi
}


function select_car() {
    N_ARGS=$#

    if [ $N_ARGS -eq 0 ]; then
        echo "Usage: larace select_car | car [car_name]"
        return 1
    else 
        local BASHRC_FILE="$HOME/.bashrc"
        local CAR_VERSION=$1
        local ROS_DOMAIN_ID
        local CONFIG_FILE 

        case $CAR_VERSION in
            vettel|rosberg|leclerc|alonso)
                CONFIG_FILE="$LARACE_ROOT/src/core/config/$CAR_VERSION/racecar_config.yaml"
                ROS_DOMAIN_ID=$(_larace_get_yaml $CONFIG_FILE general.ros_domain_id)

                if grep -q "export LARACE_CAR_VERSION=" ~/.bashrc; then
                    sed -i.bak "s|^export LARACE_CAR_VERSION=.*|export LARACE_CAR_VERSION=${CAR_VERSION}|" "$BASHRC_FILE"
                else
                    echo "export LARACE_CAR_VERSION=$CAR_VERSION" >> ~/.bashrc
                fi

                if grep -q "export ROS_DOMAIN_ID=" ~/.bashrc; then
                    sed -i.bak "s|^export ROS_DOMAIN_ID=.*|export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}|" "$BASHRC_FILE"
                else
                    echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> ~/.bashrc
                fi

                source ~/.bashrc > /dev/null 2>&1

                echo "LARACE_CAR_VERSION: ${CAR_VERSION}"
                echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
                ;;
            help | *)
                echo "Car name not recognized. Available car names:"
                echo "  alonso"
                echo "  leclerc"
                echo "  rosberg"
                echo "  vettel"
                ;;
            esac

        
    fi
}


function _exec() {
    echo -e $LIGHT_CYAN" -> $@"$NO_COLOUR | sed 's/  */ /g'
    eval "$@"
}


function _larace_ssh() {
    if [ $N_ARGS -eq 0  ]; then
        echo "Usage: larace ssh [car ip/car name]"
    fi

    local CAR_IP=$1

    if ! is_reachable $CAR_IP; then
        echo "$LIGHT_RED Error: Car $CAR_IP not reachable."
        return 1
    fi
    _exec "ssh larace@$CAR_IP"   
}


function _larace_sync_config() {
    local SOURCE_PATH="$LARACE_ROOT/src/core/config"
    local INSTALL_PATH="$LARACE_ROOT/install/core/share/core/config"
    if [ ! -d "$INSTALL_PATH" ]; then
        echo "Installation path not found. Please install project with 'colcon build' first"
    else 
        echo "Synchronizing config: $SOURCE_PATH -> $INSTALL_PATH"
        rsync -avz $SOURCE_PATH/ $INSTALL_PATH/ 
    fi
}


function _larace_launch_local() {
    # Usage: larace launch [launch type]
    N_ARGS=$#

    if [ $N_ARGS -eq 0 ]; then
        COMMAND="help"
    else
        COMMAND=$1
        N_ARGS=$(expr $N_ARGS - 1) 
        shift
    fi

    case "$COMMAND" in
        sim)
            ros2 launch f1tenth_gym_ros gym_bridge_launch.py
            ;;
        foxglove)
            ros2 launch foxglove_bridge foxglove_bridge_launch.xml
            ;; 
        help)
            cat <<- 'EOF'
Usage: larace launch [launch type]

Launch types: 
    sim         launch F1TENTH Gym
    foxglove    launch Foxglove 
EOF
            ;;
        *)
            echo "Launch type $COMMAND not recognized."
            larace launch help
            ;;
        esac
}


function _larace_rviz() {
    # Usage: larace rviz [launch type]
    if [ $N_ARGS -le 0  ]; then
        echo "Usage: larace rviz [launch type]"
        return -1
    fi
    local RVIZ_TYPE=$1

    case "$RVIZ_TYPE" in
        base)
            _exec "rviz2 -d  $LARACE_ROOT/src/core/rviz/base.rviz"
            ;;
        time_trial|tt|time_trials)
            _exec "rviz2 -d  $LARACE_ROOT/src/core/rviz/time_trial.rviz"
            ;;
        mapping)
            _exec "rviz2 -d  $LARACE_ROOT/src/core/rviz/mapping.rviz"
            ;;
        head_to_head|hh|h2h|head2head)
            _exec "rviz2 -d  $LARACE_ROOT/src/core/rviz/head_to_head.rviz"
            ;;
        help)
            cat <<- 'EOF'
Usage: larace rviz [launch type]

Launch types: 
    base
    time_trial
    head_to_head
    mapping
EOF
            ;;
        *)
            echo "Launch type $RVIZ_TYPE not recognized."
            larace rviz help
            ;;
    esac
}


function larace(){
    local LIGHT_RED="$(echo -e "\e[1;31m")"
	local LIGHT_CYAN="$(echo -e "\e[1;36m")"
    local NO_COLOUR="$(echo -e "\e[0m")"

    N_ARGS=$#

    if [ $N_ARGS -eq 0 ]; then
        COMMAND="help"
    else
        COMMAND=$1
        N_ARGS=$(expr $N_ARGS - 1) 
        shift
    fi

    case "$COMMAND" in
        launch)
            _larace_launch_local "$@"
            ;;
        ssh)
            _larace_ssh "$@"
            ;;
        rviz)
            _larace_rviz "$@"
            ;;
        select_map | map)
            select_map "$@"
            ;;
        select_car | car)
            select_car "$@"
            ;;
        sync_config )
            _larace_sync_config "$@"
            ;;
        help)
            cat <<'EOF'
Usage: larace [command]

Commands: 
    launch              [launch type]   
    rviz                [launch type]   
    ssh                 [car name]      
    select_car | car    [car name]      
    select_map | map    [map name]
    sync_config       
    help        

select_map [map_name]
select_car [car_name]
EOF
            ;;
        *)
            echo "Command $COMMAND not recognized."
            larace help
            ;;
        esac 
}
