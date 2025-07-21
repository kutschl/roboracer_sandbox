# Roboracer sandbox extension
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/$USERNAME/ws/install/setup.bash 
alias source_ws='source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/${USERNAME}/ws/install/setup.bash'
alias cont_init='bash ~/ws/src/devops/docker/container_init.sh'

export LARACE_ROOT=~/ws
export LARACE_CAR_VERSION="localhost"
export LARACE_CURRENT_MAP="levine"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

cd $LARACE_ROOT && source src/devops/docker/env/env.sh