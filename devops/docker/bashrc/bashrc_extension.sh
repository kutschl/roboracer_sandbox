# Roboracer sandbox extension
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/$USERNAME/roboracer_sandbox/install/setup.bash 
alias source_ws='source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/${USERNAME}/roboracer_sandbox/install/setup.bash'
alias cont_init='bash ~/roboracer_sandbox/src/devops/docker/container_init.sh'
