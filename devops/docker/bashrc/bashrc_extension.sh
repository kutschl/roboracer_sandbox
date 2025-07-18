# Roboracer sandbox extension
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/$USERNAME/ws/install/setup.bash 
alias source_ws='source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/${USERNAME}/ws/install/setup.bash'
alias cont_init='bash ~/ws/src/devops/docker/container_init.sh'

export LARACE_ROOT=~/ws
export LARACE_CAR_VERSION="test_driver"
export LARACE_MAP="levine"

cd $LARACE_ROOT && source src/devops/docker/env/env.sh