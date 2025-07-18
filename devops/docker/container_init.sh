#!/bin/bash

# Setup permissions
sudo chown -R $USERNAME /home/$USERNAME/ws/

# Setup sim
bash ~/roboracer_sandbox/src/devops/sim/f110_sim_setup.sh || echo "Failed to setup f110_sim"



