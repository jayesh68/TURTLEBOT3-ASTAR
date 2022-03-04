# Path Planning for Turtlebot3 robot using A* 
This project consists of implementing a path planning algorithm using A* on a turtlebot3 robot of a particular height, width and clearance to get to a end coal given different start nodes and wheel RPMs. The simulation has been performed in Gazebo.

# Initital Conditions
The start node and intitial orientation is already entered. The user needs to input the two RPMs and the goal nodes.The start node of the turtlebot is set 
in the launch file in terms of the gazebo coordinates. For example if a start node of -4,-4.5 is given in the launch file. The start node in our program would be 100,50. The threshold distance is 100 cm (1m).
A total of 6 test cases have been tested. For the 10x10 map. The maximum value in gazebo is 5 and the minimum value is -5. 

## Steps to launch the environment and model in Gazebo
1. Clone the repository to your local machine.
2. Copy the ROS package p3_Astar to your catkin workspace.
3. The python script would present in the src folder in the ROS package. The name of the file is p3_astar_ros1.py.
4. The file can be launched using the command roslaunch p3_Astar astar.launch
5. The video simulations of the 6 test cases have been attached in the simulation videos folder

## Test Cases
The first four test cases are for a start node of 100,50 (gazebo:-4,-4.5) and initial orientation of 30 degrees whereas the remaining are for different conditions. 
1. For the first test case the goal coordinates to be reached are 500,400 (gazebo: 0,-1). The RPMs used were 30,35. 
2. For the second case the goal coordinates are 800,100. The RPMs used were (800,100). 
3. For the third test case the goal coordinates were 200,350 (gazebo: -3,-3.5). The RPMs used were 20,30. The magnitude of the velocity had to be scaled by 1.5 in this case to reach the goal node. 
4. For the fourth case the goal coordinates were 300,700 (gazebo: -2,2). The RPMs used were 20,30. The magnitude of the velocity had to be scaled by 1.5 in this case to reach the goal node.
5. For a start node of 800,500 (gazebo: 3,0) and the goal node to be 700,700 (gazebo 2,2). The RPMs which worked were 5,10. 
6. For a start node of 600,800 (gazebo: 1,3) and the goal node to be 900,900 (gazebo: 4,4). The RPMs which worked were 15,20.

## Gazebo environment
<img src="https://github.com/jayesh68/TURTLEBOT3-ASTAR/blob/main/PathImages/Screenshot%20from%202022-03-04%2011-34-05.png"/>

## Robot position in 2d visualization and Gazebo
<img src="https://github.com/jayesh68/TURTLEBOT3-ASTAR/blob/main/PathImages/Screenshot%20from%202022-03-04%2011-26-58.png" width="350" height="350"/>
<img src="https://github.com/jayesh68/TURTLEBOT3-ASTAR/blob/main/PathImages/Screenshot%20from%202022-03-04%2011-27-25.png" width="350" height="350"/>

<img src="https://github.com/jayesh68/TURTLEBOT3-ASTAR/blob/main/PathImages/Screenshot%20from%202022-03-04%2011-28-11.png" width="350" height="350"/>
<img src="https://github.com/jayesh68/TURTLEBOT3-ASTAR/blob/main/PathImages/Screenshot%20from%202022-03-04%2011-31-49.png" width="350" height="350"/>

## Simulation Videos
* https://youtu.be/89I4cntxWxM
* https://youtu.be/89I4cntxWxM
