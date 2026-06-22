FROM ros:humble

RUN apt-get update \
    && apt-get install -y \
    # ros-humble-desktop \
    nano \
    tmux \
    tmuxinator \
    jstest-gtk \
    python3-serial \
    ros-humble-ur-client-library \
    ros-humble-ur \
    ros-humble-moveit \ 
    ros-humble-moveit-visual-tools \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

#example copy line
#COPY example_dir/ /site_config/

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf var/lib/apt/lists/*

RUN usermod -aG dialout ${USERNAME}

#may be usefull for autocomplete but idk how to set it up exactly
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc
COPY zenpool.yml /home/${USERNAME}/.config/tmuxinator/zenpool.yml
COPY zenpoolsim.yml /home/${USERNAME}/.config/tmuxinator/zenpoolsim.yml
COPY tmux.conf /home/${USERNAME}/.tmux.conf
COPY zenpool.config.rviz /home/${USERNAME}/.zenpool.config.rviz
COPY bash_aliases /home/${USERNAME}/.bash_aliases


ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]



#swap to ros in dockerfile    
#USER ros
#swap back to root
#USER root