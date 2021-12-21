import click
import docker
import subprocess
import docker_commands
from typing import Optional, List
import consts
import os


def new(workspace: str, ros_version: str, path: Optional[str]) -> None:
    image_name = f'{ROS_IMAGE_REPOSITORY}:{ros_version}'
    if 'ros:' in ros_version:
        image_name = ros_version  # If user gives for example 'ros:latest' as --ros-version then replace the composed image_name

    # Pulls the base ros base image 
    if not docker_commands.image_exists(image_name):
        docker_commands.pull_image(image_name)

    # Build dros base image
    if not docker_commands.image_exists(DROS_BASE_IMAGE_NAME):
        docker_commands.build_dros_image()
        
    catkin_ws = dros_utils.catkin_ws_path_from(path)
    try:
        os.mkdir(catkin_ws)
    except:
        pass

    subprocess.run([
        "docker", "create",
        "--name", workspace,
        "--mount", "type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix",
        "--mount", f"type=bind,source={catkin_ws}/,target=/root/catkin_ws",
        DROS_BASE_IMAGE_NAME
    ])

    dros_utils.init_workspace(workspace)


def start(workspace: str) -> None:
    docker_commands.start_container(workspace)


def connect(workspace: str) -> None:
    docker_commands.start_container(workspace)
    subprocess.run([
        "docker", "exec",
        "-w", "/root",  # Equivalent to home directory
        "-it", workspace, "/bin/bash"
    ])


def clear_workspace(workspace: str) -> None:
    docker_commands.start_container(workspace)

    cmd = "/bin/bash -c 'rm -r ~/catkin_ws/*'"
    docker_commands.exec_run_in(workspace, cmd)


def init_workspace(workspace: str) -> None:
    docker_commands.start_container(workspace)

    cmd = """/bin/bash -c 'mkdir -p ~/catkin_ws/src && \\
            cd ~/catkin_ws && \\
            source /ros_entrypoint.sh && \\
            source /opt/ros/$ROS_DISTRO/setup.bash && \\
            catkin_make'"""
    docker_commands.exec_run_in(workspace, cmd)


def catkin_ws_path_from(path: Optional[str]) -> str:
    catkin_ws_folder = "catkin_ws"

    if path != None and os.path.isdir(path):

        if not os.path.isabs(path):
            path = os.path.abspath(os.path.join(os.getcwd(), path))

        if os.path.exists(path):
                if path.endswith(catkin_ws_folder):
                    return path
                else:
                    return os.path.join(path, catkin_ws_folder)

    return os.path.join(os.getcwd(), catkin_ws_folder)


def get_workspaces() -> List[str]:
    return list(map(
        lambda container : container.name,
        docker_commands.get_containers_with_base_image(consts.DROS_BASE_IMAGE_NAME)
    ))


def workspace_exists(workspace: str) -> bool:
    return docker_commands.container_exists(workspace)


def rename_workspace(workspace, new_name) -> None:
    docker_commands.rename_container(workspace, new_name)