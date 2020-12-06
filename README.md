# My master's thesis code work

**Title:** *Sliding mode control of a quadrotor with a suspended load for trajectories based on the differential flatness property of the system and input shaping*

**Abstract:**	In the context of high demand for autonomous aircrafts in cargo transport applications, this work presents a motion control and trajectory generation solution for a system composed of a quadrotor with a cable-suspended load that aims to control the aircraft position and reduce the load swing. First, the dynamic model of the system is derived using the Newton-Euler and Euler-Lagrange methods and divided into two parts: a fully actuated subsystem associated with the robot altitude and yaw angle, and an underactuated subsystem associated with the other state variables of the quadrotor. Each subsystem is controlled by a sliding mode controller which is proved to be stable in Lyapunov's sense for the task of driving the system to the sliding surfaces and staying on them. It is demonstrated by the Routh-Hurwitz stability criterion that the sliding surfaces associated with the underactuated subsystem are locally stable given some constraint rules obtained for the control parameters that make the tuning process easier. Finally, a new trajectory generation structure is proposed to suppress the load balance which consists on build a polynomial trajectory for the load, apply input shaping on it and compute the desired state of the aircraft by making use of the differential flatness property of the system. The controller is tested together with the designed trajectory generator in simulation for a point-to-point trajectory and for different durations. The controller is effective in controlling the aircraft state even when no filter is applied to the input reference and the trajectory generator greatly reduces the load swing. However, the aircraft trajectory obtained by the differentially flat model becomes prohibitive for the controller when the reference speed and acceleration reach certain limits for which the alternative solution of only applying input shaping to the aircraft trajectory presents moderate performance.

DOI: http://doi.org/10.14393/ufu.di.2019.2548 [Portuguese]

## Code Structure

### Notes
Jupyter notebooks containing all the mathematical deductions of the kinematics, dynamics, differential flatness property and the controller stability.

### Simulation
 MATLAB code to simulate the controlled system


### **Feel free to [contact me](https://github.com/mateus-amarante) for any further explanation or help!**