docker exec --tty \
    --interactive \
    roboracer_sandbox \
    /bin/bash -c "cd ~/roboracer_sandbox && colcon build && source install/setup.bash"