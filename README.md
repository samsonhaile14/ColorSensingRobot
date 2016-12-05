# ColorSensingRobot
CPE 470 Project  
_  
Instructions for running simulations(tested using kinetic distro):  
_  
The program is designed to be run in a linux environment.
_  
First, ensure that the turtlebot simulator is installed before continuing. 
Instructions for doing so can be found in this video tutorial: 
(URL: https://www.youtube.com/watch?v=9U6GDonGFHw)  
Special Notes:  
The video uses the indigo distro for installation, so use:
```  
apt-cache search turtlebot
```  
to determine the name of the kinetic packages and install those instead if using
ros kinteic. Also, ensure that the package 'ros-kinetic-turtlebot-gazebo' 
is installed in case it is not. After this, source the environment variables with
the command:
```  
source /opt/ros/(distro)/setup.bash
```  
Where (distro) denotes the version of ROS being used. These instructions, along 
with the video should ensure the installation of the turtlebot
simulator.  
_  
In addition, catkin must be installed on the distro in 
order to run related ROS packages. This is done with terminal command:  
```  
sudo apt-get install ros-(distro)-catkin  
```  
where (distro) is the distro of ros being used. This simulation has only been
tested with the kinetic distro and so the instructions in this README will rely
on that fact.  
_  
With consideration of the path to the directory representing the repository, run
the command:  
```  
source (path)/ColorSensingRobot/catkin_ws/devel/setup.bash  
```  
where (path) is the path to the ColorSensingRobot directory. Three terminal 
windows will be required in order to run the program. One to run the simulation,
 another to run the multiplexer controlling the robot's velocity, and another to
run the program. In the terminal window for running the program,in the 
ColorSensingRobot/catkin_ws directory, run:  
```  
catkin_make  
```  
This should make the project.  
_  
In the multiplexer terminal window, run:  
```  
roslaunch yocs_cmd_vel_mux cmd_vel_mux.launch  
```  
This command might not work for distros hydro and earlier. The terminal window
will then start running the multiplexer for controlling velocity.  
_  
In the simulation terminal window, run the command:
```  
roslaunch turtlebot_gazebo turtlebot_world.launch  
```  
Note that sometimes gazebo will not successfully start. This is likely due to
that the kinetic distro is new and suffers from some bugs. In this event, I find
that the turtlebot_gazebo command will have to be run several times in a
trial-and-error fashion until the simulation successfully starts up.  
_  
Prior to running the program, the project's setup.bash must be sourced with the
following command:
```  
source (path to repository directory)/ColorSensingRobot/catkin_ws/devel/setup.bash
```  
Lastly, in the terminal for the program, run the command:
```  
rosrun location_monitor location_monitor_node  
```  
The robot in the gazebo simulation window should start to move according to the
program specified in the source code.

