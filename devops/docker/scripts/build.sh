docker exec --tty \
    --interactive \
    roboracer_sandbox \
    /bin/bash -c "cd ~/ws && colcon build && source install/setup.bash"