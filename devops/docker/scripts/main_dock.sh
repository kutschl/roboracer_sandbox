#! /bin/bash

# Script to launch the main docker instance
IMAGE=roboracer_sandbox


docker run --tty -it \
    --interactive \
    --network=host \
    --env DISPLAY=$DISPLAY \
    --env USER=$USER \
    --env HOSTNAME=$HOSTNAME \
    --env XAUTHORITY=/home/$USER/.Xauthority \
    -e "TERM=xterm-256color" \
    --volume $XAUTH_LOC:/home/$USER/.Xauthority \
    --volume /dev:/dev \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --restart unless-stopped \
    --volume $ROBORACER_SANDBOX_ROOT/../cache/roboracer_sandbox/build:/home/$USER/roboracer_sandbox/build \
    --volume $ROBORACER_SANDBOX_ROOT/../cache/roboracer_sandbox/install:/home/$USER/roboracer_sandbox/install \
    --volume $ROBORACER_SANDBOX_ROOT/../cache/roboracer_sandbox/log:/home/$USER/roboracer_sandbox/log \
    --volume $ROBORACER_SANDBOX_ROOT:/home/$USER/roboracer_sandbox/src/ \
    --privileged \
    --name roboracer_sandbox \
    --entrypoint /bin/bash \
    ${IMAGE}:humble
