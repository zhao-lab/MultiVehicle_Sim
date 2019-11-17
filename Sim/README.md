Carla Simulation of Multi-Vehicle Interaction Scenario and Vehicle Controllers
This repo provides the python implementation of vehicle controller and velocity tracking algorithms to geenrate multi-vehicle interaction scenarios. The interation patterns are generated through the DPGP algorithm using Gaussian Process to represent multi-vehicle driving scenarios with Dirichlet Process adapting cluster numbers.

This repository contains both online and offline simulation architectures. The online simulation architecture simualtes multi-vehicle scenario by communication with the DPGP algorithm at each simulation epoch. The DPGP algorihtm generates a finite horizon trajectory for all the vehicles based on their current state and the matching traffic pattern.
The offline architecture takes the finite horizon trajectories of all the vehicles in the scenario as input and tracks the trajectory using the vehicle driving algorithm.<br>
The driving algorihtm uses the vehical controller (LQR lateral and PID longitudinal), along-with a collision detection and traffic light state detection pipeline to track the finite horizon reference trajectories and velocity profiles.
This repository also provides a data collection script to generate datasets from the Carla Simulator. The recorded dataset contains states (position, velcoity, acceleration) of all the agents (vehicles and pedestrians), traffic light locations and states, as well as location of speed limit signs. 

#### Implement
Control vehicle using Logitech G920 steering controller: **manual_control_steeringwheel.py** <br>
Online simualtion of traffic patterns: **online_controller.py** <br>
Offline simulation of multiple traffic patterns: **controller5.py** <br>
Offline simulation of single traffic pattern: **controller4.py** <br>
Dataset Collection: **data_query.py**, example implementation in **manual_control.py** <br>

The python version code is implemented by Ashish Roongta @SafeAI lab in CMU. <br>
The scripts are compatible with Carla 0.9.6.

<p align="center">
  <img width="460" height="300" src=images/sim_setup.jpg>
</p>
