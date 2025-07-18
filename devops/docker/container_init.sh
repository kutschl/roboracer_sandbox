#!/bin/bash

# Setup permissions
sudo chown -R $USERNAME /home/$USERNAME/ws/

# Setup sim
source /usr/share/virtualenvwrapper/virtualenvwrapper.sh && \
mkvirtualenv gym_env && \
workon gym_env && \
cd $LARACE_ROOT/src/f1tenth_gym && \
pip install -e . && \
pip install transforms3d


# # Setup sim
# cd $LARACE_ROOT && bash src/devops/sim/f110_sim_setup.sh || echo "Failed to setup f110_sim"

