- I used virtualenv to develop the python cli tool
# Table of contents
<!-- TOC -->
- [Enable GUI Applications inside workspace](#enable-gui-applications-inside-workspace)
<!-- /TOC -->


# How to

## Creating a new workspace




# Special Cases


## Enable GUI applications inside workspace

1. Open a terminal on your host environment

2. Type `xauth version` into your terminal to check if it is installed. If it is not installed go ahead and install it.

3. List your auth entries with `xauth list`. You should get an output similar to this:  
`myuser/unix:0  MIT-MAGIC-COOKIE-1  158**************************1a8`

4. Copy the output from your terminal.

5. Connect to your dros workspace by using the 'connect' or 'select' command.

6. Type in the following command in the connected terminal:  
`xauth add <insert copied entry from step 3 and 4>`

7. Now you should be able to start GUI Applications from inside your workspace.

<mark>**You will probably need to redo this process after a system or workspace restart.**</mark>


## Edit your workspace without connecting to workspace

1. Open a terminal in your host environment and navigate to the folder containing your 'catkin_ws' folder.

2. Type in the following command:  
`sudo chown <your username>:<your username> -R catkin_ws`  
This will change the ownership of all folders and files to the given user.

3. Now you are able to modify your workspace from your host environment.

<mark>**After using the `reinit` command or after using `catkin_make` you will have to redo this process, because the ownership will be reset to the root user of the workspace environment.**</mark>


# Tutorials

## Running SICK laserscanner with dros

Before yor start this tutorial, be sure to enable GUI Applications in your workspace by following the associated tutorial.

1. Create a new workspace with name 'sick-ws'

2. Connect to workspace 'sick-ws'

3. Create a 'src' folder in your catkin_ws if it does not exists yet.

4. Clone the following repositories into '~/catkin_ws/src':  
`https://github.com/ros-drivers/sicktoolbox`  
`https://github.com/ros-drivers/sicktoolbox_wrapper`

5. Run the following command:  
`catkin_make install`  
If there is an error about a package named 'diagnostic_update', then run the following command:  
`sudo apt-get install ros-<your ros distro>-diagnostics`  
Now run `catkin_make install` again.

6. Now make sure you got read/write permissions on the associated USB device by running `sudo chmod a+rw /dev/ttyUSB0`

7. Now open two additional terminals and connect them to your dros workspace 'sick-ws'.

8. In the first terminal tun `roscore`  
In the second terminal run  
`rosrun sicktoolbox_wrapper sicklms _port:=/dev/ttyUSB0 _baud:=38400`  
In the third terminal run `rosrun rviz rviz`.  
You should now have a window running the RViz Visualizer.

9. Now configure the visualizer to display the scanner data  
--INSERT Image here--