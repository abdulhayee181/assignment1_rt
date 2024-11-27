# ROS Assignment1_RT

This ROS package contains two nodes that control and monitor turtles in the `turtlesim` environment.


## What I Have Done in This Project

This project involved developing two main nodes that interact with the `turtlesim` environment:

1. **Node 1 - UI Node**:
   - **Objective**: Allow the user to spawn a second turtle (`turtle2`) and control either `turtle1` or `turtle2` by setting their velocities.
   - I created a Python script (`ui_node.py`) that:
     - Spawns a new turtle (`turtle2`) at a specified location.
     - Accepts user input to select either `turtle1` or `turtle2`.
     - Accepts user input to set the linear and angular velocities for the selected turtle.
     - The turtle moves with the specified velocities for 1 second, then stops, and the user can issue new commands.

2. **Node 2 - Distance Node**:
   - **Objective**: Monitor the distance between the two turtles and stop them if they come too close to each other or to the boundaries of the `turtlesim` world.
   - I created a Python script (`distance_node.py`) that:
     - Subscribes to the `/turtle1/pose` and `/turtle2/pose` topics to get the positions of both turtles.
     - Calculates the Euclidean distance between the two turtles.
     - Publishes the distance on the `/turtle_distance` topic.
     - If the distance between the turtles is less than a threshold, the turtles are stopped.
     - If either turtle moves too close to the boundary (x or y outside [2.0, 8.0]), it is also stopped.

3. **Package Organization**:
   - I created a new ROS package `assignment1_rt` that contains both nodes.
   - Both nodes are implemented in Python and make use of the `turtlesim` package to control the turtles and monitor their positions.

4. **Functionality Implementation**:
   - **UI Node**:
     - Spawned a turtle at a specified position and orientation.
     - Took user input for controlling the velocity of the selected turtle.
     - Moved the selected turtle for 1 second and then stopped it.
   - **Distance Node**:
     - Subscribed to the pose information of both turtles.
     - Calculated and monitored the distance between them.
     - Stopped the turtles if they became too close to each other or the boundaries.

5. **Testing**:
   - I tested the nodes by running them in parallel, controlling the turtles via the UI Node and monitoring the distance between them using the Distance Node.
   - Ensured the system responds correctly when turtles come too close to each other or move out of bounds.


## Package Overview

- **Node 1 (UI Node)**: 
  - **Spawns a new turtle** (`turtle2`).
  - **Provides a simple interface** for the user to select and control either `turtle1` or `turtle2`, setting their velocities.

- **Node 2 (Distance Node)**: 
  - **Monitors the distance** between `turtle1` and `turtle2`.
  - Stops the turtles if they come **too close** to each other (distance threshold) or to the **boundaries** of the simulation environment.


## Prerequisites

Ensure that the following software is installed on your machine:

1. **ROS (Robot Operating System)**: This package is designed for **ROS Noetic**. You can install ROS Noetic by following the official ROS installation guide: [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation).
2. **turtlesim package**: This package is required to visualize and control the turtles in the simulation environment. It's usually installed by default in ROS Noetic.
3. **catkin workspace**: You need to have a ROS workspace to build and run the nodes. If you don't have a workspace, you can create one using the following commands:
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## Installation and Setup

### Step 1: Clone the repository

Clone the repository containing this package into your `catkin_ws/src` directory:

    ```
	cd ~/catkin_ws/src
	git clone <https://github.com/abdulhayee181/assignment1_rt.git>
	```
###  Step 2: Build the package

Once you've cloned the repository, navigate to your catkin_ws directory and build your workspace:

	```
	cd ~/catkin_ws
	catkin_make
	source devel/setup.bash
	```
### Step 3: Launch turtlesim and Run the Nodes

Before running the nodes, you'll need to launch the turtlesim simulation environment.

**Start** turtlesim:
Open a new terminal and run the following command to start the turtlesim simulator:

	```
	roscore
	```
Then, in a new terminal, run:

	```
	rosrun turtlesim turtlesim_node
	```
	
This opens a new window showing the turtles in the simulation environment.

### Step 4: Run the First Node (UI Node)

In another terminal, run the UI Node (this will allow you to control the turtles):

	```
	rosrun assignment1_rt ui_node_1.py

	```
This will start the first node, which allows you to input velocity commands for turtle1 or turtle2. You'll need to provide the turtle name (either turtle1 or turtle2) and its linear and angular velocities.


### Step 5: Run the Second Node (Distance Node)

Now, run the Distance Node in another terminal to monitor the distance between the turtles and ensure they stop if they get too close to each other or to the boundaries:

	```
	rosrun assignment1_rt distance_node_2.py
	```
This node will automatically monitor the distance between turtle1 and turtle2 and stop the turtles if necessary (when they approach the threshold distance or get too close to the boundary).

### Step 6: Interact with the System
	- **Control Turtles** : In the terminal where the UI Node is running, you can select the turtle and input the velocities. For example:
	
	select the turtle : turtle1 or turtle2 : 
	turtle1
	Enter the linear velocity for turtle1: 
	2.0
	enter angular velocity for turtle1: 
	0.5

The turtle will move with the specified velocities for 1 second before stopping. You can repeat the process to change velocities or select a different turtle.

	- **Monitor Distance** : The Distance Node will automatically monitor the distance between 		turtle1 and turtle2 and print messages in the terminal if they are too close. It will also 		stop the turtles if necessary.
	
### Step 7: (Optional) Visualize the Distance

You can also visualize the distance between turtle1 and turtle2 using rqt_plot or by echoing the /turtle_distance topic:

	```
	rostopic echo /turtle_distance
	```
	
This will print the distance between the turtles, and you can see when it goes below the threshold and triggers stopping


### Troubleshooting 
- **Turtles don't stop** :
	- Ensure that both turtles are spawned before running the distance monitoring node.
	- Verify that the turtles' positions are being published by checking the /turtle1/pose and 		  /turtle2/pose topics:
	
	```rostopic echo /turtle1/pose
	   rostopic echo /turtle2/pose
	```
- **Node fails to start** :

	- Make sure you have sourced your ROS setup file:
	```
	source ~/catkin_ws/devel/setup.bash
	```
- **UI Node doesn't accept input** :

	- If the terminal isn't accepting input, try running the script in the foreground to ensure 	  it receives user input:

	```
	rosrun assignment1_rt ui_node_1.py
	```
	
- **Turtles not moving** :

	- Check if the velocities are set correctly in the UI Node and that there are no issues in 		  the cmd_vel topic.
	
	
**Conclusion**
This package provides a simple UI for controlling turtles in the turtlesim environment and a monitoring system that stops the turtles if they come too close to each other or the boundaries. The two nodes work together to provide a basic interactive control interface and distance-based safety measures for the turtles.
