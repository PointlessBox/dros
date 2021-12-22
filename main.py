import click
import docker
from typing import Optional
import subprocess
import docker_commands
from dros import dros_utils
from consts import ROS_IMAGE_REPOSITORY, ROS_IMAGE_DEFAULT_TAG, DROS_BASE_IMAGE_NAME
import os

@click.group()
def cli() -> None:
    """
    A tool to create ROS-Workspaces as docker-containers and to interact with them.
    Only compatible with docker-images of ROS-Versions pre ROS2.
    """
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
    Creates an initializes the given workspace with the given ros_version, and creates a catkin_ws folder at the given path.
    
    \b
    Parameters
    ----------
    workspace : str
        Workspace to create.
    ros_version : str
        The ROS-Version to build from (e.g. noetic).
        Defaults to melodic.
    path : str
        The path where the catkin_ws folder should be placed on the host environment.
        e.g. ~/ros-workspaces/ws-1          -> /home/ros-workspaces/ws-1/catkin_ws
             /home/path/to/workspaces/ws-x  -> /home/path/to/workspaces/ws-x/catkin_ws
             /home/ws-x/catkin_ws           -> /home/ws-x/catkin_ws
        Defaults to current working directory (./catkin_ws)
    """

    if not dros_utils.workspace_exists(workspace):
        click.echo(f"Creating '{workspace}'. This might take a while")

        dros_utils.new(workspace, ros_version, path)
        
        click.echo(f"'{workspace}' created")
    else:
        click.echo(f"'{workspace}' already exists")


@click.command()
@click.argument('workspace')
def reinit(workspace: str) -> None:
    """
    Clears the catkin_ws folder of the given workspace and initializes it again.

    \b
    Parameters
    ----------
    workspace : str
        Workspace reinitialize.
    """

    confirm_message = "This will clear all files in your persisted workspace. Continue?"

    if dros_utils.shout_if_workspace_exists(workspace) and click.confirm(confirm_message, abort=True):
        click.echo(f"Clearing workspace '{workspace}' ...")
        dros_utils.clear_workspace(workspace)
        click.echo(f"Initializing workspace '{workspace}' ...")
        dros_utils.init_workspace(workspace)


@click.command()
@click.argument('workspace')
def connect(workspace: str) -> None:
    """
    Connects you to the given workspace by opening a shell.
    
    \b
    Parameters
    ----------
    workspace : str
        Workspace to connect to.
    """

    if dros_utils.shout_if_workspace_exists(workspace):
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
    selector = click.prompt("Select workspace (index or name)")

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

    \b
    Parameters
    ----------
    workspace : str
        Workspace to rename.
    """

    if dros_utils.shout_if_workspace_exists(workspace):
        new_name = click.prompt("New name")

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

    if dros_utils.shout_if_workspace_exists(workspace):
        dros_utils.start(workspace)
    

@click.command()
@click.option(
    '--persist/--no-persist',
    default=True,
    help='keeps the content of the catkin_ws folder on host environment.'
)
@click.argument('workspace')
def remove(workspace: str, persist: bool) -> None:
    """
    Removes the given workspace.

    \b
    Parameters
    ----------
    workspace : str
        Workspace to remove.
    persist : bool
        Keeps folder content of the 'catkin_ws' folder, corresponding to 'workspace', on host environment.
        Defaults to True.
    """

    if dros_utils.shout_if_workspace_exists(workspace):
        click.echo(f"Removing '{workspace}'. This might take a while")
        dros_utils.remove(workspace, persist)


cli.add_command(new)
cli.add_command(start)
cli.add_command(connect)
cli.add_command(select)
cli.add_command(reinit)
cli.add_command(rename)
cli.add_command(list)
cli.add_command(remove)


if __name__ == '__main__':
    cli()