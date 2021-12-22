import click
import docker
import subprocess
import docker_commands
from typing import Optional, List
import consts
import os
import pwd


def new(workspace: str, ros_version="melodic", path: Optional[str]=None) -> None:
    """
    Creates an initializes the given workspace with the given ros_version, and creates a catkin_ws folder at the given path.
    
    Parameters
    ----------
    workspace : str
        Workspace to create.
    ros_version : str
        The ROS-Version to build from (e.g. noetic).
        Defaults to melodic.
        You can find all images here: https://registry.hub.docker.com/_/ros/
    path : str
        The path where the catkin_ws folder should be placed on the host environment.
        e.g. ~/ros-workspaces/ws-1          -> /home/ros-workspaces/ws-1/catkin_ws
             /home/path/to/workspaces/ws-x  -> /home/path/to/workspaces/ws-x/catkin_ws
             /home/ws-x/catkin_ws           -> /home/ws-x/catkin_ws
        Defaults to current working directory (./catkin_ws)
    """
    image_name = f'{consts.ROS_IMAGE_REPOSITORY}:{ros_version}'
    if 'ros:' in ros_version:
        image_name = ros_version  # If user gives for example 'ros:latest' as --ros-version then replace the composed image_name

    # Pulls the base ros base image 
    if not docker_commands.image_exists(image_name):
        docker_commands.pull_image(image_name)

    # Build dros base image
    if not docker_commands.image_exists(consts.DROS_BASE_IMAGE_NAME):
        docker_commands.build_dros_image()
        
    catkin_ws = catkin_ws_path_from(path)

    workspace_config = {
        catkin_ws: {
            "bind": "/root/catkin_ws",  # binds the 'catkin_ws' path on host to '/root/catkin_ws' in container
            "mode": "rw"  # read-write
        }
    }

    docker_commands.run_container(workspace, workspace_config)

    # subprocess.run([
    #     "docker", "create",
    #     "--name", workspace,
    #     "--mount", "type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix",
    #     "--mount", f"type=bind,source={catkin_ws}/,target=/root/catkin_ws",
    #     consts.DROS_BASE_IMAGE_NAME
    # ])

    init_workspace(workspace, catkin_ws)


def start(workspace: str) -> None:
    """
    Starts the given workspace.
    
    Parameters
    ----------
    workspace : str
        Workspace to start.
    """

    docker_commands.start_container(workspace)


def connect(workspace: str) -> None:
    """
    Opens a shell to the given workspace.
    
    Parameters
    ----------
    workspace : str
        Workspace to connect to.
    """
    docker_commands.start_container(workspace)
    cmd = "/bin/bash"
    workdir = "/root"
    #docker_commands.exec_run_in(workspace, cmd, workdir=workdir, stdin=True, tty=True)
    subprocess.run([
        "docker", "exec",
        "-w", "/root",  # Equivalent to home directory
        "-it", workspace, "/bin/bash", "-c", 'while :; do (bash); if [ $? -eq "0" ]; then break; fi; done' 
    ])


def clear_workspace(workspace: str) -> None:
    """
    Removes the content of the catkin_ws folder corresponding to the given workspace.
    
    Parameters
    ----------
    workspace : str
        Workspace to clear catkin_ws from.
    """
    docker_commands.start_container(workspace)

    cmd = "/bin/bash -c 'rm -r ~/catkin_ws/*'"
    docker_commands.exec_run_in(workspace, cmd)


def init_workspace(workspace: str, path: str) -> None:
    """
    Initializes the given workspace with 'catkin_make'.
    
    Parameters
    ----------
    workspace : str
        Workspace to initialize.
    """
    docker_commands.start_container(workspace)

    cmd = """/bin/bash -c 'mkdir -p ~/catkin_ws/src && \\
            cd ~/catkin_ws && \\
            source /ros_entrypoint.sh && \\
            source /opt/ros/$ROS_DISTRO/setup.bash && \\
            catkin_make'"""
    docker_commands.exec_run_in(workspace, cmd)

    current_user = pwd.getpwuid( os.getuid() ).pw_name
    subprocess.run(["sudo", "chown", f"{current_user}:{current_user}", "-R", path])


def catkin_ws_path_from(path: Optional[str]) -> str:
    """
    Build and return the path to a new catkin_ws from the absolute or relative path: 'path'
    
    Parameters
    ----------
    path : str
        Absolute or relative path to create the catkin_ws folder in.
    """
    catkin_ws_folder = "catkin_ws"

    if path != None:
        if not os.path.isabs(path):
            path = os.path.abspath(os.path.join(os.getcwd(), path))

        if not path.endswith(catkin_ws_folder):
            path = os.path.join(path, catkin_ws_folder)
    else:
        path = os.path.join(os.getcwd(), catkin_ws_folder)
    
    if not os.path.exists(path):
        os.makedirs(path)

    return path


def get_workspaces() -> List[str]:
    """
    Gets all existing workspaces.
    """
    return list(map(
        lambda container : container.name,
        docker_commands.get_containers_with_base_image(consts.DROS_BASE_IMAGE_NAME)
    ))


def workspace_exists(workspace: str) -> bool:
    """
    Checks if the given workspace already exists.

    Parameters
    ----------
    workspace : str
        Workspace rename.
    """
    return docker_commands.container_exists(workspace)


def rename_workspace(workspace: str, new_name: str) -> None:
    """
    Renames to given workspace with the new name.

    Parameters
    ----------
    workspace : str
        Workspace to rename.
    new_name : str
        New name of workspace.
    """
    docker_commands.rename_container(workspace, new_name)


def remove(workspace: str, persist=True) -> None:
    """
    Removes the workspace.

    Parameters
    ----------
    workspace : str
        Workspace to remove.
    persist : bool
        Keeps folder content of the 'catkin_ws' folder, corresponding to 'workspace', on host environment.
        Defaults to True.
    """
    if not persist:
        clear_workspace(workspace)
    docker_commands.remove(workspace)


def shout_if_workspace_exists(workspace: str) -> bool:
    """
    Returns true if workspace exists.
    Uses click.echo to tell the user if it does not exist.

    Parameters
    ----------
    workspace : str
        Workspace to check.
    """
    exists = workspace_exists(workspace) 
    if not exists:
        click.echo(f"'{workspace}' does not exist")
    return exists


def get_workspace_from_user_input(workspace: str) -> str:
    result = ""
    try:
        result = list(filter(lambda ws : ws == workspace, get_workspaces()))[0]
    except:
        pass
    return result