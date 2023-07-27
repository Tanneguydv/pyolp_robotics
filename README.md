![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/e60bea8b-bc73-4eab-9e3c-d317f81a622c)
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
![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/24fd89eb-fa97-4e0c-90e2-a2948099ad70)

An example of poses obtained from two frames, and intermediates poses calculated in a linear articular space
![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/e76a3d35-26cc-4512-b659-8577768184f6)

Another example using the extraction of an edge from a shape to generate poses, similar than a linear one in the cartesian space\
![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/9832c7f4-81f7-4ced-b7f8-75fbf2fe2cf1)

A pose computed taking into account the collision environment
![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/fd4a36e0-1023-4107-bd08-ec19cfce3dbb)

The capability to detect self-collision events\
![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/fa4a757d-8af6-4144-8741-97c23d94eef8)

and finally an example of application, a simple slicer method and poses generation (taken from [pythonocc-demos](https://github.com/tpaviot/pythonocc-demos))
![image](https://github.com/Tanneguydv/pyolp_robotics/assets/81742654/0c689996-914d-4040-9850-a198f79ec093)


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
