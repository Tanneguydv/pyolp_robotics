![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/5838dab0-7230-46e3-baa3-6a66069d5c1d)

# pyolp_robotics
A 3D graphic tool to sketch serial robots offline programming

Full python framework, simple and easy to implement 
Robotic fk & ik implemented into a Pythonocc CAD kernel, enabling collision detection, 3d operations and trajectory planning in the same framework.\
For the moment two robots are implemented : an UR10 from Universal Robot (6 axes) and an Iiwa from Kuka (7axes).

This tool is mainly a pedagogic tool to understand the basic concepts of robotic kinematics in a friendly programming environment.

## Use
Find robot's pose with a given configuration/3D frame to reach/collision environment/etc.\
The tool use [IKPY](https://github.com/Phylliade/ikpy) to solve the inverse kinematics of the robot model (but used my fork for compatibility with this repo) . The model is specified with its *Denavit Hartenberg* parameters.\
Thanks to the [Pythonocc library](https://github.com/tpaviot/pythonocc-core) we can unleash the 3d operations possibilities, as it has the potential to treat nurbs objects.\
It also enables a realistic visualisation of the poses and to define collisions environnement.\
The [FCL python wrapper](https://pypi.org/project/python-fcl/) is used with Pythonocc to deal with collision, this is not optimized though.

## Examples / tests
A simple forward kinematic example with sliders
![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/2507f370-df5f-445f-9de4-24d09a0c2b69)

An example of poses obtained from two frames, and intermediates poses calculated in a linear articular space
![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/91d434b8-7834-4a36-a02f-f3f09d1b18e6)

Another example using the extraction of an edge from a shape to generate poses, similar than a linear one in the cartesian space\
![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/be9dc955-b40f-4248-b4b7-fa78f7a5acc1)

A pose computed taking into account the collision environment
![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/163cadd9-cd78-401e-a7f8-4c78e822c9f1)


The capability to detect self-collision events\
![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/6e69d469-0c26-4794-805b-ac1e4d6ce2cb)


and finally an example of application, a simple slicer method and poses generation (taken from [pythonocc-demos](https://github.com/tpaviot/pythonocc-demos))
![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/4611cf42-6a5c-4c87-a645-38d56c867373)


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
Create a new class such as `ur.py` that inherits from `Robot`.\
All you need is to :
- specify the *Denavit-Hartenberg* parameters of your robot
- set its joint limits
- create a directory `your_robot_axes_stp` with the 3D representation of your robot, with all axes saved in its initial position in `.stp` file format separately and named by order
- define colors, material and other attributes to your class

You can also define new tool, in the `tool` directory. As the robot's axes all you need to do is to save the 3D stp file in its initial position, name it `your_tool`, add its name to the available tool of the robot.\
You will need to create a `your_tool.txt` file containing the coordinates of its `TCP` : "(x, y, z), (DirZx, DirZy, DirZz), (DirXx, DirXy, DirXz)".
Softwares like *Rhinoceros3d* or *Blender* could be useful for this.

## Connect it to robot
Tried successfully using these interfaces with few minor ergonomic changes :\
[ur interface](https://github.com/ErwinLutke/UR-Interface)\
[IiwaPy3](https://github.com/Modi1987/iiwaPy3)

## Warning
This tool is mainly a pedagogic tool to understand the basic concepts of robotic kinematics
- No garanty of good behavior, it works for me in the real world but use with cautious.
- The inverse kinematics solver can only combine one orientation (Z in the planner) with the position (see https://github.com/Phylliade/ikpy/issues/138). With a tool aligned with the flange, it can be resolved by calculted the angle between the X direction of frame to reach and the TCP's one, turn accordingly the flange, test if pose is suitable for the robot and forward.
- Trajectory planning and logic may be different from the one implemented in the robot's controller, use with cautious at low velocity for first tries\
For a more robust solution you can check [ROS Industrial](https://rosindustrial.org/), which offers great tools and has a large community of developpers.

## Further work to be done
Implement jacobian method to deal with trajectory speed and so on\
add other robots\
...\
feel free to contribute :)
