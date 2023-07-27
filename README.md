# pyolp_robotics
A 3D graphic tool to sketch serial robots offline programming

Full python framework, simple and easy to implement 
Robotic fk & ik implemented into a Pythonocc CAD kernel, enabling collision detection, 3d operations and trajectory planning in the same framework 

This tool is mainly a pedagogic tool to understand the basic concepts of robotic kinematics in a friendly programming environment.

## Use
Find robot's pose with a given configuration/3D frame to reach/collision environment/etc.\
The tool use IKPY to solve the inverse kinematics of the robot model. The model is specified with its *Denavit Hartenberg* parameters.
Thanks to the Pythonocc library we can unleash the 3d operations possibilities, as it has the potential to treat nurbs objects.\
It also enables a realistic visualisation of the poses and to define collisions environnement.
The FCL python wrapper is used with Pythonocc to deal with collision, this is not optimized though.

## Examples / tests
A simple forward kinematic example with sliders

An example of poses obtained from two frames, and intermediates poses calculated in a linear articular space

Another example using the extraction of an edge from a shape to generate poses, similar than a linear one in the cartesian space

A pose computed taking into account the collision environment

The capability to detect self-collision events

and finally an example of application, a simple slicer method and poses generation (taken from pythonocc-demos)

## Installation
Successfully runned on windows 10 and Ubuntu 22
Install *pyolp_robotis* as a package inside a conda environment :
```
conda env create -f environment.yml
conda activate pyolp
pip install .
```
run a test
```
cd tests
python ./ik_articular_poses.py
```

you're good!

## How to define your robot
Create a new class such as `ur.py` that inherits from `Robot`.
All you need is to :
- specify the *Denavit-Hartenberg* parameters of your robot
- set its joint limits
- create a directory `your_robot_axes_stp` with the 3D representation of your robot, with all axes saved in its initial position in `.stp` file format separately and named by order
- define colors, material and other attributes to your class

You can also define new tool, in the `tool` directory. As the robot's axes all you need to do is to save the 3D stp file in its initial position, name it `your_tool`, add its name to the available tool of the robot. 
You will need to create a `your_tool.txt` file containing the coordinates of its `TCP` : "(x, y, z), (DirZx, DirZy, DirZz), (DirXx, DirXy, DirXz)".
Softwares like *Rhinoceros3d* or *Blender* could be useful for this.

## Connect it to robot
Tried successfully using these interfaces :
ur interface
python matlab server

## Warning
This tool is mainly a pedagogic tool to understand the basic concepts of robotic kinematics
- No garanty of good behavior, it works for me in the real world but use with cautious.
- Trajectory planning and logic may be different from the one implemented in the robot's controller, use with cautious at low velocity for first tries
For a more robust solution you can check ROS Industrial, which offers great tools and has a large community of developpers.

## Further work to be done
Implement jacobian method to deal with trajectory speed and so on
add other robots
...
