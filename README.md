# Simulation of multirotors carrying suspended loads using MATLAB

This folder contains a generic framework to test controllers for drones carrying suspended loads. Actually, it can be extended for any system.

## Framework Description

To simulate a controlled system, you need to define:

1) Physical Parameters (struct)

2) Dynamic Model (function)

3) Controller (function)

4) Trajectory (function)

5) Simulation Parameters(structure)

5) Plot Functions (function)

The system must be written in the form:

```math
\begin{cases}
\dot{x}_ 1= x_{n_{/2}+1} \\
...\\
\dot{x}_i= x_{n_{/2}+i}\\
...\\
\dot{x}_{n_{/2}}= x_{n}\\
\dot{x}_{n_{/2}+1}=f_1(x,u,t) \\
... \\
\dot{x}_{n_{/2}+i}=f_i(x,u,t) \\
... \\
\dot{x}_{n}=f_n(x,u,t) 
\end{cases}
```

For a 2 DOF system:

```math
\begin{cases}
\dot{x}_ 1 = x_3 \\
\dot{x}_ 2 = x_4 \\
\dot{x}_ 3 = f_1(x,u,t) \\
\dot{x}_ 4 = f_2(x,u,t)
\end{cases}
```

## Folder Description

* **/config** : each config file must define a set of physical parameters, a trajectory (function), a dynamic model (function), simulation parameters and functions;

* **/plot** : define plot functions, especially plot\_state and plot\_animation;

* **/trajectory_planning** : trajectory planning helper functions;

* **/trajectories** : common trajectory (functions) used in config files;

* **/physical_params** : define dimensions and physical properties of the system;

* **/control** : general control functions used by controllers;
 
* **/controllers** : controllers used in "ode\_fun" and selected in config files;

* **/dynamics** : contain functions that compute the dynamic model of the system, returning the derivatives of the provided state variables;

* **/auxiliary** : helper functions not related to other folders.

* **/analysis** : scripts used to write the master thesis

* **/optimization_functions** : used to optimize control parameters (not used anymore)


Pick one configuration from "/config" folder as input for *main.m* script. 
