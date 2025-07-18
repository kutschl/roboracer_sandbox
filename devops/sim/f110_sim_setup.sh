#! /bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
WS_DIR=$(readlink -f "$SCRIPT_DIR/../../..")   # should end up in the workspace dir

# ubuntu packages dependencies
# xargs sudo apt-get install -y < "$SCRIPT_DIR/linux_req_sim.txt"

# python dependencies
#echo $SCRIPT_DIR
#pip3 install -r "$LARACE_ROOT/src/core/install/requirements/python_req.txt"
pip3 install -e "$LARACE_ROOT/src/base_system/f1tenth_gym"

