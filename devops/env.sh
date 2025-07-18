#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBORACER_SANDBOX_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

function _rrs_docker() {
    local command=$1
    case "$command" in
        compose)
            cd $ROBORACER_SANDBOX_ROOT
            export UID=$(id -u)
            export GID=$(id -g)
            sudo mkdir -p ../cache/roboracer_sandbox/build ../cache/roboracer_sandbox/install ../cache/roboracer_sandbox/log
            docker compose build roboracer_sandbox
            ;;
        run)
            cd $ROBORACER_SANDBOX_ROOT
            source devops/docker/xauth_setup.sh
            bash devops/docker/scripts/main_dock.sh
            ;;
        main)
            cd $ROBORACER_SANDBOX_ROOT
            bash devops/docker/scripts/attach_main.sh
            ;;
        new)
            cd $ROBORACER_SANDBOX_ROOT
            bash devops/docker/scripts/add_dock.sh
            ;;
        purge)
            docker system prune -a
            ;;
        remove_container)
            cd $ROBORACER_SANDBOX_ROOT
            sudo bash devops/docker/scripts/remove_container.sh
            ;;
        *)
            echo "Command not recognized: ${command}"
            ;;
    esac
}

function rrs(){
    N_ARGS=$#

    if [ $N_ARGS -eq 0 ]; then
        COMMAND="help"
    else
        COMMAND=$1
        N_ARGS=$(expr $N_ARGS - 1) 
        shift
    fi

    case "$COMMAND" in
        docker)
            _rrs_docker "$@"
            ;;
        help)
            echo "Roboracer Sandbox" 
            ;;
        *)
            echo "Unknown command: $COMMAND"
            ;;
        esac

}