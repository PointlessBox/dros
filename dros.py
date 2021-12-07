import click
import docker
#from typing import Optional
#import json
import subprocess
from docker_commands import image_exists, pull_image, build_dros_image
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
@click.argument('workspace')
def new(workspace: str, ros_version: str) -> None:
    """
    Creates a new workspace from the given name and ros-version.

    Parameters
    ----------
    name : str
        The name of the created docker container. Defaults to a random name.
    ros-version : str
        Represents the tag of the ros-image e.g. 'ros:melodic'.
        You can find all images here: https://registry.hub.docker.com/_/ros/
    """

    image_name = f'{ROS_IMAGE_REPOSITORY}:{ros_version}'
    if 'ros:' in ros_version:
        image_name = ros_version  # If user gives for example 'ros:latest' as --ros-version then replace the composed image_name
    
    # Pulls the base ros base image 
    if not image_exists(image_name):
        pull_image(image_name)

    # Build dros base image
    if not image_exists(DROS_BASE_IMAGE_NAME):
        build_dros_image()

    subprocess.run([
        "docker", "create",
        "--name", workspace,
        "--mount", "type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix",
        "--mount", f"type=bind,source={os.getcwd()},target=/root",
        DROS_BASE_IMAGE_NAME
    ])


#docker create \
#--name dros-ws-1 \
#--mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix \
#--mount type=bind,source=$(pwd),target=/root \
#dros-ws

cli.add_command(new)
# cli.add_command(connect)


if __name__ == '__main__':
    cli()