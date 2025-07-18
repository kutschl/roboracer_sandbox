#!/bin/bash

export LARACE_ROOT=$(pwd)
echo "LaRace root dir (LARACE_ROOT): ${LARACE_ROOT}"
echo "Car version (LARACE_CAR_VERSION): ${LARACE_CAR_VERSION}"
echo "Current map (LARACE_CURRENT_MAP): ${LARACE_CURRENT_MAP}"
echo "Current ros domain id (ROS_DOMAIN_ID): $ROS_DOMAIN_ID"
echo "Discovery server (ROS_DISCOVERY_SERVER): $ROS_DISCOVERY_SERVER"
echo "ROS super client (ROS_SUPER_CLIENT): $ROS_SUPER_CLIENT"

_update_map_name() {
    #
    # Changes the environment variable LARACE_CURRENT_MAP in the .bashrc
    #
    local new_map_name="$1"
    local bashrc_file="$HOME/.bashrc"

    # Check if the provided map name is not empty
    if [ -z "$new_map_name" ]; then
        echo "Usage: update_larace_map <new_map_name>"
        return 1
    fi

    # Use sed to search for the line and update it
    sed -i.bak "s|^export LARACE_CURRENT_MAP=.*|export LARACE_CURRENT_MAP=${new_map_name}|" "$bashrc_file"

    # Notify user of the change
    if [ $? -eq 0 ]; then
        echo "Updated LARACE_CURRENT_MAP to '${new_map_name}' in .bashrc."
    else
        echo "Failed to update LARACE_CURRENT_MAP."
    fi
}

function update_config() {
    # Function copies the configuration files from the source directory
    # into the installation and build directory without installation.
    local sourcePath="/home/luis/ws/src/race_stack/src/core/config"
    local installPath="/home/luis/ws/install/core/share/core/config"
    if [ ! -d "$installPath" ]; then
        echo "Installation path not found. Please install project with 'colcon build' first"
    fi
    echo "Synchronizing config: $sourcePath -> $installPath"
    rsync -avz $sourcePath/ $installPath/ 
}


function _is_localhost() {
    #
    # Function checking whether the given address is the local host
    #
    local addr="$1"

    # Check if the address is 127.0.0.1 or localhost
    if [[ "$addr" == "127.0.0.1" || "$addr" == "localhost" ]]; then
        return 0  # True: it is localhost
    fi

    # Resolve the address to check if it resolves to localhost
    if [[ "$(getent hosts "$addr" | awk '{ print $1 }')" == "127.0.0.1" ]]; then
        return 0  # True: it resolves to localhost
    fi

    return 1  # False: it is not localhost
}

function _exec() {
    echo -e $LIGHT_CYAN" -> $@"$NO_COLOUR | sed 's/  */ /g'
    eval "$@"
}

function _run_in_docker() {
    echo "Running in docker: $@"
    docker exec --tty \
        --interactive \
        racestack_ros2_main \
        /bin/bash -c "$@"
}

# function _set() {
#     if [ $N_ARGS -eq 2  ]; then
#         echo "Please provide car and variable key and value! Usage: larace set [ip/car name] [key] [value]"
#     fi
#     local carAddr=$1
#     local key=$2
#     local value=$3
#     local isLocalhost=1;
#     if ! _is_localhost "$carAddr"; then
#         if ! is_reachable $carAddr; then
#             echo $LIGHT_RED"Error: Car $carAddr not reachable.$NO_COLOUR"
#             return 1
#         fi
#         local isLocalhost=0
#         else
#             echo "Setting variables locally"
#             _set_local "$@"
#     fi

#     case "$key" in
#         map)
#             _exec "ssh -t larace@$carAddr '_run_in_docker _update_map_name $value; exit'"
#         ;;
#         *)
#             echo "Variable not recognized: ${key}"
#             echo "Choose from: map"
#         ;;

#     esac
# }

# function _set_local() {
#     if [ $N_ARGS -eq 2  ]; then
#         echo "Please provide car and variable key and value! Usage: larace set [ip/car name] [key] [value]"
#     fi
#     local carAddr=$1
#     local key=$2
#     local value=$3
#     local isLocalhost=1;

#     case "$key" in
#         map)
#             _run_in_docker "_update_map_name $value"
#         ;;
#         *)
#             echo "Variable not recognized: ${key}"
#             echo "Choose from: map"
#         ;;

#     esac
# }

function _launch() {
    if [ $N_ARGS -eq 1  ]; then
        echo "Please provide car and launch type! Usage: larace launch [ip/car name] [launch type]"
    fi
    local carAddr=$1
    local launchType=$2
    if ! _is_localhost "$carAddr"; then
        if ! is_reachable $carAddr; then
            echo $LIGHT_RED"Error: Car $carAddr not reachable.$NO_COLOR"
            return 1
        fi
    else
        _launch_local "$@"
        return 0
    fi

    case "$launchType" in
        base)
            ssh -t larace@$carAddr "source ~/.bashrc; _run_in_docker 'source_ws; ros2 launch core base_system_launch.xml map_name:=$LARACE_CURRENT_MAP sim:=false racecar_version:=$LARACE_CAR_VERSION'"
        ;;
        behavior)
            ssh -t larace@$carAddr "source ~/.bashrc; _run_in_docker 'source_ws; ros2 launch core behavior_launch.xml map_name:=$LARACE_CURRENT_MAP racecar_version:=$LARACE_CAR_VERSION'"
        ;;
        state_estimation)
            ssh -t larace@$carAddr "source ~/.bashrc; _run_in_docker 'source_ws; ros2 launch state_estimation state_estimation_launch.xml map_name:=$LARACE_CURRENT_MAP racecar_version:=$LARACE_CAR_VERSION'"
        ;;
        perception)
            ssh -t larace@$carAddr "source ~/.bashrc; _run_in_docker 'source_ws; ros2 launch perception perception_launch.xml racecar_version:=$LARACE_CAR_VERSION'"
        ;;

        *)
            echo "Launch type not recognized: ${launchType}"
        ;;

    esac

}

function _launch_local() {
    if [ $N_ARGS -eq 1  ]; then
        echo "Please provide car and launch type! Usage: larace launch [ip/car name] [launch type]"
    fi
    local carAddr=$1
    local launchType=$2

    case "$launchType" in
        base)
            ros2 launch core base_system_launch.xml map_name:=$LARACE_CURRENT_MAP sim:=false racecar_version:=$LARACE_CAR_VERSION "${@:3}"
        ;;
        base_sim)
            ros2 launch core base_system_launch.xml map_name:=$LARACE_CURRENT_MAP sim:=true racecar_version:=$LARACE_CAR_VERSION "${@:3}"
        ;;
        behavior)
            ros2 launch core behavior_launch.xml map_name:=$LARACE_CURRENT_MAP racecar_version:=$LARACE_CAR_VERSION "${@:3}"
        ;;
        state_estimation)
            ros2 launch state_estimation state_estimation_launch.xml map_name:=$LARACE_CURRENT_MAP racecar_version:=$LARACE_CAR_VERSION "${@:3}"
        ;;
        perception)
            ros2 launch perception perception_launch.xml racecar_version:=$LARACE_CAR_VERSION "${@:3}"
        ;;
        planner)
            cd $LARACE_ROOT/src/planner/global_planner
            pip3 install -e .
            cd ../planner_gui
            python3 planner_gui/app.py
        ;;
        mapping)
            ros2 launch core mapping_launch.xml racecar_version:=$LARACE_CAR_VERSION "${@:3}"
        ;;
        time_trial)
            ros2 launch core time_trial_launch.xml map_name:=$LARACE_CURRENT_MAP racecar_version:=$LARACE_CAR_VERSION "${@:3}"
        ;;
        tt)
            ros2 launch core time_trial_launch.xml map_name:=$LARACE_CURRENT_MAP racecar_version:=$LARACE_CAR_VERSION "${@:3}"
        ;;
        head_to_head)
            ros2 launch core head_to_head_launch.xml map_name:=$LARACE_CURRENT_MAP racecar_version:=$LARACE_CAR_VERSION "${@:3}"
        ;;
        h2h)
            ros2 launch core head_to_head_launch.xml map_name:=$LARACE_CURRENT_MAP racecar_version:=$LARACE_CAR_VERSION "${@:3}"
        ;;
        param)
            ros2 run parameter_tuner parameter_tuner
        ;;
        *)
            echo "Launch type not recognized: ${launchType}"
        ;;

    esac

}

function _ssh() {
    if [ $N_ARGS -eq 0  ]; then
        echo "Please provide car connect to! Usage: larace deploy [ip/car name]"
    fi
    local carAddr=$1
    if ! is_reachable $carAddr; then
        echo $LIGHT_RED"Error: Car $carAddr not reachable."
        return 1
    fi
    _exec "ssh larace@$carAddr"   
}


function _deploy() {
    if [ $N_ARGS -eq 0  ]; then
        echo "Please provide car to deploy to! Usage: larace deploy [ip/car name]"
    fi
    local carAddr=$1
    if ! is_reachable $carAddr; then
        echo $LIGHT_RED"Error: Car $carAddr not reachable."
        return 1
    fi   

    _exec "rsync -avuz --delete --exclude ".git"  ${LARACE_ROOT} larace@${carAddr}:~/"

    if [[ $? -eq 0 ]]; then
        echo $LIGHT_CYAN"############################################################"$NO_COLOUR
        echo $LIGHT_CYAN"Source code was successfully synchronized"'!'$NO_COLOUR
        echo $LIGHT_CYAN"############################################################"$NO_COLOUR
    else
        echo $LIGHT_CYAN"############################################################"$NO_COLOUR
        echo $LIGHT_CYAN"Careful: The rsync failed so the code has NOT been deployed"'!'$NO_COLOUR
        echo $LIGHT_CYAN"############################################################"$NO_COLOUR
    fi
}

function _docker() {
    if [ $N_ARGS -le 1  ]; then
        echo "Please provide car to run Docker commands on! Usage: larace docker [ip/car name] [command]"
        return -1
    fi
    local carAddr=$1
    local command=$2
    if _is_localhost "$carAddr"; then
        echo "Running docker locally!"
        _docker_local "$@"
        return
    fi
    if ! is_reachable $carAddr; then
        echo $LIGHT_RED"Error: Car $carAddr not reachable.$NO_COLOUR"
        return -1
    fi
    case "$command" in
        compose)
            _exec "ssh -t larace@${carAddr} 'export UID=$(id -u) && export GID=$(id -g) && cd race_stack && docker compose build larace_x86 && mkdir -p ../cache/humble/build ../cache/humble/install ../cache/humble/log ../cache/humble/data'"
            ;;
        run)
            _exec "ssh -t larace@${carAddr} 'cd race_stack; source .devcontainer/xauth_setup.sh; ./src/core/docker/main_dock.sh'"
            ;;
        main)
            _exec "ssh -t larace@${carAddr} 'cd race_stack; ./src/core/docker/attach_main.sh'"
            ;;
        new)
            _exec "ssh -t larace@${carAddr} 'cd race_stack; ./src/core/docker/add_dock.sh'"
            ;;
        purge)
            _exec "ssh -t larace@${carAddr} 'cd race_stack; docker system prune -a'"
            ;;
        remove_container)
            _exec "ssh -t larace@${carAddr} 'cd race_stack; ./src/core/docker/remove_container.sh'"
            ;;
        *)
            echo "Command not recognized: ${command}"
            ;;
    esac
}

function _docker_local() {
    if [ $N_ARGS -le 1  ]; then
        echo "Please provide car to run Docker commands on! Usage: larace docker [ip/car name] [command]"
    fi
    local carAddr=$1
    local command=$2
    case "$command" in
        compose)
            cd $LARACE_ROOT
            export UID=$(id -u)
            export GID=$(id -g)
            mkdir -p ../cache/humble/build ../cache/humble/install ../cache/humble/log ../cache/humble/data
            if [ "$(uname -m)" = "aarch64" ]; then
                docker compose build larace_arm
            else
                docker compose build larace_x86
            fi
            ;;
        run)
            cd $LARACE_ROOT
            source .devcontainer/xauth_setup.sh
            ./src/core/docker/main_dock.sh
            ;;
        main)
            cd $LARACE_ROOT
            ./src/core/docker/attach_main.sh
            ;;
        new)
            cd $LARACE_ROOT
            ./src/core/docker/add_dock.sh
            ;;
        purge)
            docker system prune -a
            ;;
        remove_container)
            cd $LARACE_ROOT
            sudo sh ./src/core/docker/remove_container.sh
            ;;
        *)
            echo "Command not recognized: ${command}"
            ;;
    esac
}

function _config() {
    if [ $N_ARGS -le 1  ]; then
        echo "Please provide car to run Docker commands on! Usage: larace docker [ip/car name] [command]"
    fi
    local carAddr=$1
    local command=$2
    case "$command" in
        retrieve)
            _exec "rsync -avz larace@$carAddr:~/race_stack/src/core/config $LARACE_ROOT/src/core/"
            ;;
        *)
            echo "Command not recognized: ${command}"
            ;;
    esac
}

function _rviz() {
    if [ $N_ARGS -le 0  ]; then
        echo "Please provide launch type! Usage: larace rviz [launch type]"
        return -1
    fi
    local rvizType=$1

    case "$rvizType" in
        base)
            _exec "rviz2 -d  $LARACE_ROOT/src/core/rviz/base.rviz"
            ;;
        time_trial)
            _exec "rviz2 -d  $LARACE_ROOT/src/core/rviz/time_trial.rviz"
            ;;
        tt)
            _exec "rviz2 -d  $LARACE_ROOT/src/core/rviz/time_trial.rviz"
            ;;
        mapping)
            _exec "rviz2 -d  $LARACE_ROOT/src/core/rviz/mapping.rviz"
            ;;
        head_to_head)
            _exec "rviz2 -d  $LARACE_ROOT/src/core/rviz/head_to_head.rviz"
            ;;
        h2h)
            _exec "rviz2 -d  $LARACE_ROOT/src/core/rviz/head_to_head.rviz"
            ;;
        *)
            echo "Rviz launch type not recognized: ${command}"
            ;;
    esac
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
        deploy)
            _deploy "$@"
            ;;
        docker)
            _docker "$@"
            ;;
        launch)
            _launch "$@"
            ;;
        # set)
        #     _set "$@"
        #     ;;
        ssh)
            _ssh "$@"
            ;;
        make)
            if [ $N_ARGS -eq 0  ]; then
                echo "Please provide car to deploy to! Usage: larace deploy [ip/car name]"
            fi
            local carAddr=$1
            if ! is_reachable $carAddr; then
                echo $LIGHT_RED"Error: Car $carAddr not reachable."
                return 1
            fi

            _exec "rsync -avuz   ${LARACE_ROOT} larace@${carAddr}:~/"

            if [[ $? -eq 0 ]]; then
                echo $LIGHT_CYAN"############################################################"$NO_COLOUR
                echo $LIGHT_CYAN"Source code was successfully synchronized"'!'$NO_COLOUR
                echo $LIGHT_CYAN"############################################################"$NO_COLOUR
            else
                echo $LIGHT_CYAN"############################################################"$NO_COLOUR
                echo $LIGHT_CYAN"Careful: The rsync failed so the code has NOT been deployed"'!'$NO_COLOUR
                echo $LIGHT_CYAN"############################################################"$NO_COLOUR
            fi
            # Add build command over ssh

            ;;
        config)
            _config "$@"
            ;;
        rviz)
            _rviz "$@"
            ;;
        help)
            echo "--------------------------"
            echo "LAMARRacing Roboracer CLI"
            echo "--------------------------"
            echo ""
            echo "Usage: larace <command> [arguments]"
            echo
            echo "Commands:"
            echo "  deploy   <ip|car>                   Deploy the LAMARRacing source code to a specified car."
            echo ""
            echo "  docker   <ip|car> <cmd>             Manage Docker on local or remote car."
            echo "                                        Commands:"
            echo "                                          compose          -> Build docker image"
            echo "                                          run              -> Run main Docker container"
            echo "                                          main             -> Attach to main Docker container"
            echo "                                          new              -> Add new Docker container"
            echo "                                          purge            -> Prune docker image"
            echo "                                          remove_container -> Remove docker container"
            echo ""
            echo "  launch   <ip|car> <type> [extras]   Launch ROS2 components on a car or locally."
            echo "                                        Module-wise:"
            echo "                                          base             -> Launch base system"
            echo "                                          base_sim         -> Launch simulation"
            echo "                                          behavior         -> Launch state machine, planner + controller"
            echo "                                          state_estimation -> Launch state estimation"
            echo "                                          perception       -> Launch perception"
            echo "                                        GUI:"
            echo "                                          planner          -> Start planner GUI"
            echo "                                          param            -> Start parameter tuner GUI"
            echo "                                        Full-Stack:"
            echo "                                          mapping          -> Launch mapping"
            echo "                                          time_trial       -> Launch time trial racing mode"
            echo "                                          tt               -> Launch time trial racing mode"
            echo "                                          head_to_head     -> Launch head-to-head racing mode."
            echo "                                          h2h              -> Launch head-to-head racing mode."
            echo ""
            echo "  ssh      <ip|car>                   Open an SSH session to the specified car."
            echo ""
            echo "  make     <ip|car>                   Sync the source code to the car (alias for deploy)."
            echo ""
            echo "  config   <ip|car> retrieve          Retrieve the config directory from the car."
            echo ""
            echo "  rviz     <type>                     Launch RViz with predefined config file."
            echo "                                        Types:"
            echo "                                          base             -> Base system"
            echo "                                          mapping          -> Mapping"
            echo "                                          time_trial       -> Time trial racing mode"
            echo "                                          tt               -> Time trial racing mode"
            echo "                                          head_to_head     -> RViz for head-to-head races."
            echo "                                          h2h              -> Alias for head-to-head."
            echo ""
            echo "  help                                Show this help message."
            ;;
        *)
            echo "Unknown command: $COMMAND"
            echo "Use 'larace help' for usage information."
            ;;
        esac
}