Authors / Team members: Sagar Ojha, Guest Student
	   		Obaid ur Rahman,  119451762

GitHub Repo link: https://github.com/Sagar-Ojha/ENPM-661/tree/main/Project3/Phase%202

Below are the listed libraries used to run the program:

1) heapq as hq
2) numpy as np
3) time
4) cv2
5) sys

(ROS related)
6) rospy
7) Twist from geometry_msgs.msg

Below are the listed packages used to run the program:

1) gazebo_ros: To get map.world scene in the gazebo simulation and to spawn turtlebot3_burger onto the scene.
2) turtlebot3_description: To get the turtlebot3_burger model urdf/xacro file.
3) xacro: To read the xacro file of the model
4) enpm661_project3: This is the package we created.

========
PART 01:
========
Go to Part01 directory and in the terminal, type: python3 proj3p2_Sagar_Fnu.py.
Give the inputs as shown below when prompted.
Enter the clearance amount (mm): 15
Enter the starting x and y position in cm separated by a space: 20 20
Enter the starting orientation of the robot (in deg): 135
Enter the final x and y position in cm separated by a space: 550 20
Enter the 2 sets of wheel rpms separated by a space: 50 100

========
PART 02:
========
Note: Download the dependencies required for turtlebot3_simulation package.

After copying/cloning the enpme661_proj3 package to the src directory of the workspace,
open up terminal in the root directory of the workspace and type: catkin_make.

Also, set the python script to executable mode by following the steps below.
Go to the src directory inside enpm661_proj3 directory and open up the terminal in that directory.
Type: chmod +x proj3p2_Sagar_Fnu.py

After the package is built, type: roslaunch enpm661_project3 proj3p2.launch
We can pass the initial robot spawn position, orientation and clearance as the arguments to the launch file.

The arguments are:
1) x: This is the initial x coordinate given in meters (default: 0 m)
2) y: This is the initial y coordinate given in meters (default: 0 m)
3) angle: This is the angular position about z-axis given in radians (default: 1.57 rad)
4) clearance: This is the robot clearance value also given in meters (default: 0.02 m)
5) model (not used): We can spawn other urdf/xacro models in map.world

In the terminal: roslaunch enpm661_project3 proj3p2.launch x:=0.4 y:=-0.25 angle:=0 clearance:=0.03
When prompted, give the inputs as such:
Enter the final x and y position in m separated by a space: 3.5 0.75
Enter the 2 sets of wheel rpms separated by a space: 50 100

============
Animation @: https://drive.google.com/drive/folders/1w5AOGcSoTfaYkt-EasbNlsIwgUcbFC-L?usp=share_link
============