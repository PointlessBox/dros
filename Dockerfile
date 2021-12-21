
#ARG UBUNTU_VERSION
#ARG ROS_INSTALLATION
ARG ROS_VERSION

FROM ros:${ROS_VERSION:-melodic}

ENV ROS_VERSION=${ROS_VERSION:-melodic}

RUN apt update

# Installing xauth to add keys for use with x-server (Needed to start GUI-Applications inside docker container)
RUN apt install -y xauth && \
    touch /root/.Xauthority

# Updating and upgrading system
RUN apt update && apt upgrade -y

# Adding sources to ~/.bashrc
RUN ["/bin/bash", "-c", "echo 'source /ros_entrypoint.sh' >> $HOME/.bashrc"]
RUN /bin/bash -c 'echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bashrc'

# Setting up catkin workspace
#RUN ["/bin/bash", "-c", "mkdir -p ~/catkin_ws/src && cd ~/catkin_ws && source /ros_entrypoint.sh && source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"]

# Installing ROS
#RUN apt update
#RUN ["/bin/bash", "-c", "DEBIAN_FRONTEND=noninteractive apt install -yq ros-$ROS_DISTRO-desktop"] 

# 'sleep infinity' will keep the docker container running, which allows us to connect to it via 'docker exec -it <container-name> /bin/bash'
ENTRYPOINT ["sleep", "infinity"]