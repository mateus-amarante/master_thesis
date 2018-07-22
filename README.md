# Simulation of multirotors carrying suspended loads using MATLAB

This folder contains a generic framework to test controllers for drones carrying suspended loads. Actually, it may be used for any system.

## Framework Description

To simulate a controlled system, you need to define:

1) Physical Parameters (struct)
2) Dynamic Model (function)
3) Controller (function)
4) Trajectory (function)
5) Plot Functions (optional)

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

* **/config** : each config file must define a set of physical parameters, a trajectory (function), a dynamic model (function) and a set of plot functions;

* **/plot** : define plot functions and common plot sets;

* **/trajectory_planning** : trajectory planning helper functions;

* **/trajectories** : common trajectory (functions) used in config files;

* **/physical_params** : define dimensions and physical properties of the system;

* **/control** : general control functions used by controllers;

* **/dynamics** : contain functions that compute the dynamic model of the system, returning the derivatives of the provided state variables;

* **/auxiliary** : helper functions not related to other folders.


Pick one configuration from "/config" folder as input for *main.m* script. 


## How to use

* **main.m** : run the simulation based on the chosen configuration selected from "config" folder;
* **ode_fun.m** :  default ode function passed to ode45 in main.m;