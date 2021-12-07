import docker
import subprocess
from consts import ROS_IMAGE_REPOSITORY, ROS_IMAGE_DEFAULT_TAG, DROS_BASE_IMAGE_NAME

def image_exists(image_name: str) -> bool:
    """Checks if the given image is already downloaded and available locally"""

    exists = False
    try:
        CLIENT.images.get(image_name)
        exists = True
    except:
        pass
    return exists


def pull_image(image_name: str = 'ros:latest'):
    """
    Pulls a ros-image from the docker-registry. Defaults to ros:latest

    Paramters
    ---------
    image_name : str | None
        The ros-image e.g. ros:latest or ros:melodic.
    
    Return
    ------
    output : str | generator
    """

    tag = image_name.split(':')[1]

    client = docker.from_env()

    client.api.pull(repository=ROS_IMAGE_REPOSITORY, tag=tag, stream=True, decode=True)


def build_dros_image(ros_version = 'melodic'):
    try:
        create_dockerfile()
    except:
        pass
    subprocess.run(["docker", "build", "-t", DROS_BASE_IMAGE_NAME, "."])


def create_dockerfile():
    dockerfile_content = """
#ARG UBUNTU_VERSION
#ARG ROS_INSTALLATION
ARG ROS_VERSION

FROM ros:${ROS_VERSION:-melodic}

ENV ROS_VERSION=${ROS_VERSION:-melodic}

RUN apt update

# lsb-release, curl and gnupg2 are needed for ROS installation
RUN apt install -y lsb-release && \\
    apt install -y curl && \\
    apt install -y gnupg2

# Installing xauth to add keys for use with x-server (Needed to start GUI-Applications inside docker container)
RUN apt install -y xauth && \\
    touch /root/.Xauthority

# Updating and upgrading system
RUN apt update && apt upgrade -y

# Adding sources to ~/.bashrc
RUN ["/bin/bash", "-c", "echo 'source /ros_entrypoint.sh' >> $HOME/.bashrc"]
RUN ["/bin/bash", "-c", "echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> $HOME/.bashrc"]

# Setting up catkin workspace
RUN ["/bin/bash", "-c", "source /ros_entrypoint.sh && source /opt/ros/$ROS_DISTRO/setup.bash && cd && mkdir -p catkin_ws/src && cd catkin_ws && catkin_make && source devel/setup.bash"]

# Installing ROS
#RUN apt update
#RUN ["/bin/bash", "-c", "DEBIAN_FRONTEND=noninteractive apt install -yq ros-$ROS_DISTRO-desktop"] 

# 'sleep infinity' will keep the docker container running, which allows us to connect to it via 'docker exec -it <container-name> /bin/bash'
CMD ["sleep", "infinity"]
"""

    with open('Dockerfile', 'xt') as dockerfile:
        dockerfile.write(dockerfile_content)