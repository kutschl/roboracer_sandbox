FROM osrf/ros:humble-desktop
ARG USERNAME
ARG UID
ARG GID

ENV SHELL /bin/bash

# Create the user
RUN groupadd --gid $GID $USERNAME \
    && useradd --uid $UID --gid $GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME


# Install packages
# We have to run the below line twice to make it take an effect... idk why
RUN apt-get update --fix-missing && apt-get upgrade -y
RUN apt-get update && apt-get upgrade -y

# install folders are added granularly to avoid unnecessary rebuilds when developing the Dockerfile
ADD devops/docker/install_req/ /tmp/.install_utils/ 
RUN xargs apt-get install -y < /tmp/.install_utils/linux_req.txt

ADD devops/docker/install_req/python_req.txt /tmp/.install_utils/python_req.txt
RUN pip3 install -r /tmp/.install_utils/python_req.txt &&\
    pip3 install quadprog~=0.1.11 virtualenv

RUN rm -rf /var/lib/apt/lists/*

# Foxglove websocket port
EXPOSE 8765


# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************
# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME

# Bashrc extension
ADD devops/docker/bashrc/ /tmp/.install_utils/
RUN echo "export USERNAME=$USERNAME" >> /home/$USERNAME/.bashrc &&\
    cat /tmp/.install_utils/bashrc_extension.sh >> /home/$USERNAME/.bashrc

USER root
# Clean up
RUN rm -rf /tmp/.install_utils
USER $USERNAME

CMD ["/bin/bash"]
