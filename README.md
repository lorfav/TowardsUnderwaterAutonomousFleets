# Decentralized Model Predictive Control for Increased Autonomy in Fleets of ROVs

## Description

Helo!

This is the repository connected to a Bachelors thesis from NTNU in 2023. It contains the code for implementation of a decentralized model predictive control architecture to be implemented in a set of [BlueROV2 Heavys](https://bluerobotics.com/store/rov/bluerov2-upgrade-kits/brov2-heavy-retrofit/).

The Python toolbox [do-mpc](https://www.do-mpc.com/en/latest/) was used for the MPC controller.

### REQUIREMENTS

To be able to run all code in this repository these programs are needed:

- [Ubuntu 22.04.2 LTS (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/): OS for running ROS 2 and Gazebo Garden

- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html): Software for system arcetecture and comunication

- [Gazebo Garden](https://gazebosim.org/docs/garden/install_ubuntu): Simulation software

- [do-mpc](https://www.do-mpc.com/en/latest/): Python toolbox used for the MPS controller (for running the Python simulator, only do-mpc is required)

-  **bluerov2_garden** Gazebo workspace (not publicly available)

### CONTENT

#### Gazebo simulator

Contains the packages made by the group connected to the Gazebo simulator.

To run this simulator all [REQUIREMENTS](https://github.com/lrfosso/TowardsUnderwaterAutonomousFleets#requirements) must be installed.

Steps to run this simulator from terminal:

1. Navigate to the **/bluerov2_garden** workspace were the **/src** folder is located

2. Source ROS 2 install: **source /opt/ros/humble/setup.bash**

3. Navigate to **/bluerov2_garden/src/mpc_controller/params**  file and set **n_multi_agent** parameter to the amount of agent you wish to spawn into the simulator.

4. Navigate to the **/bluerov2_garden** 

5. Run the command: **colcon build** to build the workspace

6. Source install in two terminals: **source install/setup.bash** 

7.  Run command in one terminal: **ros2 launch bluerov_launch bluerov_launch.py**

8. Run command in the another terminal: **ros2 launch mpc_controller multi_mpc_launch.py**

Gazebo Garden and a GUI should have launched at this point. Start the simulation in Gazebo Garden with the play button in the lower left corner.

#### Python simulator(Do-mpc simulator)


Contains the code made by the group connected to the do-mpc simulator.
To run this simulator only [do-mpc](https://www.do-mpc.com/en/latest/) is required:
1. Navigate to the folder [PythonSimulators](https://github.com/lrfosso/TowardsUnderwaterAutonomousFleets/tree/main/PythonSimulators/)

2. Chose the simulator that you want to run, and navigate to it's directory
	- [PythonSimulator (Used in results)](https://github.com/lrfosso/TowardsUnderwaterAutonomousFleets/tree/main/PythonSimulators/PythonSimulator)
	- [CentralizedSimulator](https://github.com/lrfosso/TowardsUnderwaterAutonomousFleets/tree/main/PythonSimulators/CentralizedSimulator)
	- [QuaternionsSimulator](https://github.com/lrfosso/TowardsUnderwaterAutonomousFleets/tree/main/PythonSimulators/QuaternionsSimulator)
3. Run command in terminal: **python3 rovSimulator.py** 
4. To visualize the data run: **python3 3D_Simulator_ROV.py**

#### Data

Contains the data from the testing in the thesis. 
