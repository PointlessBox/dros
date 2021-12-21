import click
import docker
from typing import Optional
#import json
import subprocess
import docker_commands
import dros_utils
from consts import ROS_IMAGE_REPOSITORY, ROS_IMAGE_DEFAULT_TAG, DROS_BASE_IMAGE_NAME
import os

@click.group()
def cli() -> None:
    pass


@click.command()
@click.option(
    '-r',
    '--ros-version',
    default='latest',
    help='name of the created workspace'
)
@click.option(
    '-p',
    '--path',
    default=None,
    help='path of the catkin_ws on host environment'
)
@click.argument('workspace')
def new(workspace: str, ros_version: str, path: Optional[str]) -> None:
    """
    Creates a new workspace from the given name and ros-version.

    Parameters
    ----------
    name : str
        The name of the created workspace. Defaults to a random name.
    ros-version : str
        Represents the tag of the ros-image e.g. 'ros:melodic'.
        You can find all images here: https://registry.hub.docker.com/_/ros/
    path : str
        The path where the catkin_ws directory will be created.
        Defaults to the current working directory.
    """

    # TODO: Refactor to use dros_utils
    if not docker_commands.container_exists(workspace):
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
        
        click.echo(f"'{workspace}' created")
    else:
        click.echo(f"'{workspace}' already exists")


@click.command()
@click.argument('workspace')
def reinit(workspace: str) -> None:
    """
    Clears the catkin_ws folder of the given workspace and initializes it again.

    Parameters
    ----------
    workspace : str
        Workspace reinitialize.
    """

    click.echo(f"Clearing workspace '{workspace}' ...")
    dros_utils.clear_workspace(workspace)
    click.echo(f"Initializing workspace '{workspace}' ...")
    dros_utils.init_workspace(workspace)


@click.command()
@click.argument('workspace')
def connect(workspace: str) -> None:
    """
    Connects you to the given workspace by opening a shell.
    
    Parameters
    ----------
    workspace : str
        Workspace to connect to.
    """
    dros_utils.connect(workspace)


@click.command()
def select() -> None:
    """
    Gives you a list with all workspaces to connect to, and asks which one to connect you to.
    """
    workspaces = dros_utils.get_workspaces()
    indices = range(len(workspaces))
    for (index, workspace) in zip(indices, workspaces):
        click.echo(f"[{index}] {workspace}")

    click.echo()
    selector = input("Select workspace (index or name): ")

    for (index, workspace) in zip(indices, workspaces):
        try:
            indx = int(selector)
            if indx == index:
                dros_utils.connect(workspace)
                break
        except:
            if selector == workspace:
                dros_utils.connect(workspace)
                break


@click.command()
@click.argument('workspace')
def rename(workspace: str) -> None:
    """
    Renames the given workspace

    Parameters
    ----------
    workspace : str
        Workspace rename.
    """
    
    new_name = input("New name: ")

    dros_utils.rename_workspace(workspace, new_name)


@click.command()
def list() -> None:
    """
    List all DROS workspaces.
    """

    workspaces = dros_utils.get_workspaces()
    indices = range(len(workspaces))
    for (index, workspace) in zip(indices, workspaces):
        click.echo(f"[{index}] {workspace}")


@click.command()
@click.argument('workspace')
def start(workspace: str) -> None:
    """
    Starts the given workspace.
    """

    dros_utils.start(workspace)


cli.add_command(new)
cli.add_command(start)
cli.add_command(connect)
cli.add_command(select)
cli.add_command(reinit)
cli.add_command(rename)
cli.add_command(list)


if __name__ == '__main__':
    cli()