<img src="media/image1.jpeg" style="width:2.58194in;height:1.53611in" />

ACKNOWLEDGEMENTS

To my advisor, **Roberto Finzi**, for the trust in my work, for the commitment to remote communication, and, most importantly, for the understanding and flexibility in allowing me to work in another state while pursuing my master's degree.

To Professor **Leonardo Sanches**, who was my co-advisor and professor in the first stage of the course, with whom I learned many concepts used in this work.

To the **Graduate Program of the Mechanical Engineering Faculty (FEMEC)** of the **Federal University of Uberlândia (UFU)**, which offered a high-quality study and research environment, making me feel connected to the scientific world.

To the colleagues of the **Autonomous Aircraft Laboratory (LAA)**, with a special mention to **Ivan Tarifa**, **Felipe Machini** and **Douglas Costa**, with whom I shared relevant practical experiences in the areas of aerial robotics and entrepreneurship.

To **CAPES**, for the scholarship offered during the first months of the program, which allowed me to focus on academic studies.

To the psychologist **Vanda**, who provided psychological support when I was anxious during the transition to graduate studies and the decision to work and pursue a master's degree simultaneously;

At the **SENAI CIMATEC**, I was given the opportunity to work on research and development projects in robotics, both during my master's program and as an incentive to pursue further education, receiving compensation for my work. This allowed me to interact with competent professionals and cutting-edge technology, which contributed to my professional and academic growth.

To my friend **Murilo Mendonça**, who diligently reviewed my work in the final stages.

To my parents **Antônio** and **Cleide**, who have always supported me unconditionally in all the challenging moments of my life, which, incidentally, have been many in the last two years.

To my wife **Camilla**, with whom I have been living together for almost a year, and whose healthy companionship has motivated me to complete the work within the deadline.

A **God**, who, sometimes through winding paths, but always correctly, brings about circumstances of much learning, achievement, and joy in my life.

In memory of my brother **Filipe**, to whom the memory of an energetic and determined person refers, characteristics that I was called upon to embody in order to complete this work.

Araújo, M. A. CONTROL BY SLIDING MODES OF A QUADCOPTER WITH A CARGO HANGED BY CABLE FOR TRAJECTORIES BASED ON THE SYSTEM'S DIFFERENTIAL STABILITY PROPERTY AND INPUT SHAPING. 100 p. Master's Dissertation, Federal University of Uberlândia, Uberlândia.

**Summary**

In the context of high demand for autonomous aircraft in cargo transport, this paper presents a control and trajectory generation solution for the quadcopter system with a suspended cargo, aimed at controlling the aircraft's position while also reducing the cargo's swing. First, a dynamic model of the system is developed using the Newton-Euler and Euler-Lagrange methods, and it is separated into two parts: a fully actuated subsystem associated with altitude and yaw angle, and a partially actuated subsystem with the other variables of the system. Each subsystem is controlled by a sliding mode controller, which is demonstrated to be stable in the Lyapunov sense for the task of driving the system to the sliding surfaces and maintaining it in that condition. The Routh-Hurwitz stability criterion also demonstrates that the sliding surfaces associated with the partially actuated subsystem are locally stable, resulting in rules for defining the control parameters that facilitate the tuning process. Finally, a new trajectory generation configuration is proposed to contain the cargo's swing. The strategy consists of constructing a point-to-point polynomial trajectory for the cargo, to which *input shaping* is applied, and then calculating the desired state of the aircraft using the system's differential flatness property. Through simulation, it is verified that the controller is effective in controlling the aircraft, and that the trajectory generator effectively reduces the cargo's oscillation, especially for low and medium aggressiveness maneuvers. Alternatively, it was verified that applying *input shaping* directly on trajectories defined for the aircraft also reduces the cargo's swing, even for aggressive maneuvers, in which the new proposed solution presented unsatisfactory results.

Araujo, M. A. **Sliding Mode Control of a Quadrotor with a Suspended Load for Trajectories based on the Differential Flatness Property of the System and Input Shaping.** 100 p. M. Sc. Dissertation, Federal University of Uberlandia, Uberlandia.

**Abstract**

Given the high demand for autonomous aircrafts in cargo transport applications, this work presents a motion control and trajectory generation solution for the problem of controlling the state of a quadrotor carrying a cabled-suspended payload while keeping the load swing stable. First, it develops the dynamic model of the system using the Newton-Euler and Euler-Lagrange methods and divides it into two parts: a fully actuated subsystem associated with the robot altitude and yaw angle, and an underactuated subsystem associated with the other state variables of the quadrotor. Each subsystem is controlled by a sliding mode controller which is proved to be stable in Lyapunov's sense for the task of driving the system to the sliding surfaces and staying on them. It is demonstrated by the Routh-Hurwitz stability criterion that the sliding surfaces associated with the underactuated subsystem are locally stable, finding constraint rules for the control parameters that helps the tuning process. Finally, a new trajectory generation structure is proposed to suppress the load balance. The strategy consists on build a point-to-point piecewise polynomial trajectory for the load, apply input shaping on it and compute the desired state of the aircraft by making use of the differential flatness property of the system. It is verified by simulation that the proposed solution effectively controls the aircraft position and greatly reduces the load swing, especially for low and mid-aggressive maneuvers. Alternatively, this work also tests the application of input shaping directly on the quadrotor trajectories, verifying it attenuates the load balance even for aggressive maneuvers that the proposed solution presented unsatisfactory results.

LIST OF SYMBOLS

| $$\alpha_{RMS}$$                                                                                                                   | Effective value of the cable angle relative to the vertical during the accommodation time for sample from a simulation.                                                   |
|------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| $$\beta_{RMS}$$                                                                                                                    | Effective value of the angle between the inertial (${\overrightarrow{e}}_{z}$) and the non-inertial (${\overrightarrow{e}}_{z}^{b}$) axes for sample from a simulation. |
| $$\mathbf{B}\left( \overrightarrow{q} \right)$$                                                                                    | Input matrix of the translational dynamic model of the system.                                                                                                       |
| $$c_{x},\ c_{y},c_{z}$$                                                                                                            | Translational linear drag coefficient of the aircraft along the x, y and z axes of the body reference frame, respectively.                                                  |
| $$c_{d}$$                                                                                                                          | Translational linear drag coefficient of the load.                                                                                                                |
| $$\mathbf{C}\left( \overrightarrow{q},\dot{\overrightarrow{q}} \right)$$                                                           | Centrifugal and *Coriolis* forces matrix of the translational dynamic model of the system.                                                                             |
| $$d$$                                                                                                                              | Length of each arm of the quadcopter.                                                                                                                                  |
| $$d_{x},d_{y},d_{x}$$                                                                                                              | Disturbance on the translational accelerations of the quadcopter.                                                                                                         |
| $$d_{\phi},d_{\theta},d_{\psi}$$                                                                                                   | Disturbance on the accelerations of the Euler angles that describe the aircraft orientation.                                                                          |
| $$d_{\phi_{L}},\ d_{\theta_{L}}$$                                                                                                  | Disturbance on the accelerations of the angles $\phi_{L}$ and $\theta_{L}$, which describe the orientation of the cable                                                             |
| $${\overrightarrow{D}}_{F} = \left\lbrack D_{F}^{x},D_{F}^{y},D_{F}^{z} \right\rbrack^{T}$$                                        | Force disturbance on the system in the inertial reference frame.                                                                                                          |
| $${\overrightarrow{D}}_{\tau} = \left\lbrack D_{\tau}^{x},D_{\tau}^{y},D_{\tau}^{z} \right\rbrack^{T}$$                            | Moment disturbance on the aircraft in the non-inertial reference frame.                                                                                                   |
| $$\Sigma^{i} = \left\lbrack {\overrightarrow{e}}_{x},{\overrightarrow{e}}_{y},{\overrightarrow{e}}_{z} \right\rbrack$$             | Inertial coordinate system.                                                                                                                                     |
| $$\Sigma^{b} = \left\lbrack {\overrightarrow{e}}_{x}^{b},{\overrightarrow{e}}_{y}^{b},{\overrightarrow{e}}_{z}^{b} \right\rbrack$$ | Non-inertial coordinate system located at the aircraft's center of mass                                                                                        |
| $$\Sigma^{c} = \left\lbrack {\overrightarrow{e}}_{x}^{c},{\overrightarrow{e}}_{y}^{c},{\overrightarrow{e}}_{z}^{c} \right\rbrack$$ | Intermediate coordinate system resulting from the rotation of angle $\psi$ around the z axis of the inertial reference frame.                                              |
| $$\Sigma^{d} = \left\lbrack {\overrightarrow{e}}_{x}^{d},{\overrightarrow{e}}_{y}^{d},{\overrightarrow{e}}_{z}^{d} \right\rbrack$$ | Intermediate coordinate system resulting from the rotation of angle $\theta$ around the y axis of the $\Sigma^{c}$ reference frame.                                        |
| $${\overrightarrow{F}}_{b} = \left\lbrack 0,0,F_{z}^{b} \right\rbrack^{T}$$                                                        | Propulsion force resulting on the aircraft in the body reference frame.                                                                                                   |
| $$F_{z}^{b} = \sum_{i = 1}^{4}F_{i}$$                                                                                              | Propulsion force along ${{\overrightarrow{e}}_{b}}_{z}$, where $F_{i}$ is the propulsion force generated by each rotor.                                           |
| $${\overrightarrow{F}}_{d}$$                                                                                                       | Translational linear drag force on the system.                                                                                                                    |
| $${\overline{f}}_{\omega}$$                                                                                                        | Average frequency of the angular frequency $\overrightarrow{\omega}$ calculated over samples from a simulation.                                                |
| $$g$$                                                                                                                              | Gravitational acceleration.                                                                                                                                            |
| $$\mathbf{G}\left( \overrightarrow{q} \right)$$                                                                                    | Vector associated with the gravitational force in the translational dynamics of the aircraft in the matrix obtained from the Lagrange formulation.                             |
| $${\overrightarrow{h}}_{\omega},\ {\overrightarrow{h}}_{\alpha}$$                                                                  | Result of the vector product of angular and acceleration velocities in the body reference frame with ${\overrightarrow{e}}_{z}^{b}$, respectively.                      |
| $$\eta_{z},\eta_{\psi},\eta_{1},\eta_{2}$$                                                                                         | Control parameters that ensure robustness against limited disturbances                                                                                              |
| $$\overrightarrow{\eta} = \lbrack\phi,\theta,\psi\rbrack^{T}$$                                                                     | Angles that define the orientation of the drone according to the Euler notation: roll, pitch, and yaw angles, respectively                                       |
| $${\overrightarrow{\eta}}_{L} = \left\lbrack \phi_{L},\theta_{L} \right\rbrack^{T}$$                                               | Angles that define the orientation of the cable obtained by a rotation around the x axis ($\phi_{L}$) followed by another around the y axis                                                    |
| $$\mathbf{I} = diag\ \left( I_{x},\ I_{y},I_{z} \right)$$                                                                          | Moment of inertia of the quadcopter.                                                                                                                                   |
| $$k_{t},k_{m}$$                                                                                                                    | Propulsion and drag constants of the rotors                                                                                                                      |
| $$\kappa_{z},\kappa_{\psi},\kappa_{1},\kappa_{2}$$                                                                                 | Control parameters that multiply the sliding variables linearly                                                                                          |
| $$l$$                                                                                                                              | Length of the cable.                                                                                                                                                 |
| $$L$$                                                                                                                              | Lagrange associated with the translational dynamic model of the system                                                                                                    |
| $$\lambda_{z},\lambda_{\psi},\ \lambda_{i = 1,2,\ldots,8}$$                                               | Control parameters present in the definition of the sliding variables                                                                                              |
| $$M,\ m$$                                                                                                                          | Mass of the aircraft and the load, respectively                                                                                                                         |
| $$\mathbf{M}\left( \overrightarrow{q} \right)$$                                                                                    | Inertia matrix of the translational dynamic model of the system.                                                                                                       |
| $$\overrightarrow{p}$$                                                                  | Unit vector from the center of mass of the drone to the center of the load, describing the orientation of the cable                                                       |
| $$\mathbf{P}\left( \overrightarrow{q} \right)$$                                                                                    | Translational linear drag force matrix in the translational dynamic matrix model of the system obtained from the Euler-Lagrange formulation.                                   |
| $$\overrightarrow{q} = \left\lbrack x,y,z,\phi_{L},\theta_{L} \right\rbrack$$                                                      | Generalized coordinates of the translational dynamic model of the system.                                                                                                |
| $$\overrightarrow{r} = \lbrack x,y,z\rbrack$$                                                                                      | Position of the drone in the inertial reference frame                                                                                                                           |
| $${\overrightarrow{r}}_{L} = \left\lbrack x_{L},y_{L},z_{L} \right\rbrack$$                                                        | Position of the load in the inertial reference frame                                                                                                                             |
| $$s_{1},s_{2},s_{3},s_{4}$$                                                                                                                          | Sliding variables.                                                                                                                                               |
| $$\overrightarrow{T} = T\ \overrightarrow{p}$$                                                                                     | Tension force in the cable with a module $T$ along $\overrightarrow{p}$.                                                                                             |
| $${\overrightarrow{\tau}}_{b} = \left\lbrack \tau_{x_{b}},\ {\tau_{y}}_{b},\ {\tau_{z}}_{b} \right\rbrack^{T}$$                    | Resulting moment around the center of mass of the aircraft produced by the propulsion force of the rotors and drag on the blades                                       |
| $$\overrightarrow{u} = \left\lbrack u_{1},u_{2},u_{3},u_{4} \right\rbrack^{T}$$                                                    | Control signal, corresponding to the input forces of the system $\left\lbrack F_{b},{\tau_{b}}_{x},{\tau_{b}}_{y},{\tau_{b}}_{z} \right\rbrack^{T}$             |
| $$\overrightarrow{v} = \lbrack u,v,w\rbrack$$                                                                                      | Translational velocity of the drone in the non-inertial reference frame                                                                                                      |
| $$V(x)$$                                                                                                                           | Lyapunov function with respect to the variable $x$.                                                                                                                        |
| $${\widetilde{x}}_{b},{\widetilde{y}}_{b}$$                                                                                        | Components $x$ and $y$ of the position error of the aircraft relative to the non-inertial reference frame projected in the plane $xy$                                                     |
| $$\omega_{n}$$                                                                                                                     | Natural frequency.                                                                                                                                                  |
| $$\overrightarrow{\omega} = \lbrack p,q,r\rbrack^{T}$$                                                                             | Angular velocity of the drone in the non-inertial reference frame                                                                                                   |
| $$\overrightarrow{\Omega} = \left\lbrack \dot{\phi},\dot{\theta},\dot{\psi} \right\rbrack^{T}$$                                    | Rate of change of the Euler angles ($\dot{\overrightarrow{\eta}}$)                                                                                                   |
| $$\zeta$$                                                                                                                          | Damping coefficient.                                                                                                                                        |

SUMMARY

1.  [INTRODUCTION [11](#_Toc24304534)](#_Toc24304534)

[1.1 Motivation and Applications [11](#motivation-and-applications)](#motivation-and-applications)

[1.2 Related Works [14](#related-works)](#related-works)

[1.2.1 Open-Loop Control [15](#open-loop-control)](#open-loop-control)

[1.2.2 Closed-Loop Control [16](#closed-loop-control)](#closed-loop-control)

[1.3 Objective and Contributions [16](#objective-and-contributions)](#objective-and-contributions)

[1.4 Document Structure [18](#document-structure)](#document-structure)

2.  [DYNAMIC MODEL [19](#dynamic-model)](#dynamic-model)

[2.1 Drone without Load [19](#drone-sem-carga)](#drone-sem-carga)

[2.2 *Drone* with Cargo Suspended by Cable [23](#drone-com-carga-suspensa-por-cabo)](#drone-com-carga-suspensa-por-cabo)

3. [CONTROLE [30](#controle)](#controle)

[3.1 Introduction [30](#introduction-1)](#introduction-1)

[3.1.1 Characteristics of Operation [30](#characteristics-of-operation)](#characteristics-of-operation)

[3.1.2 Slider Mode Control [32](#slider-mode-control)](#slider-mode-control)

[3.2 Control Strategy [35](#control-strategy)](#control-strategy)

[3.2.1 CMD of the Fully Actuated Subsystem ($z,\psi$) [36](#cmd-do-subsistema-totalmente-atuado-zpsi)](#cmd-do-subsistema-totalmente-atuado-zpsi)

[3.2.2 CMD do Subsistema Sub-atuado [40](#cmd-do-subsistema-sub-atuado)](#cmd-do-subsistema-sub-atuado)

[3.2.3 Summary [49](#resumo)](#resumo)

[3.3 Simulation [50](#simulation)](#simulation)

[3.3.1 System response to single step input [51](#response-to-system-for-single-step-input)](#response-to-system-for-single-step-input)

[3.3.2 Assessing the stability condition on the sliding surface [55](#assessing-the-stability-condition-on-the-sliding-surface)](#assessing-the-stability-condition-on-the-sliding-surface)

4. [TRAJECTORY GENERATION [58](#trajectory-generation)](#trajectory-generation)

[4.1 Generation of Trajectory Based on the Differential Planarity of the System [58](#generation-of-trajectory-based-on-the-differential-planarity-of-the-system)](#generation-of-trajectory-based-on-the-differential-planarity-of-the-system)

[4.1.1 Differential Planing of the System [59](#differential-planing-of-the-system)](#differential-planing-of-the-system)

[4.1.2 Determining System Variables [60](#determining-system-variables)](#determining-system-variables)

[4.1.3 Defining Load Trajectories [64](#defining-load-trajectories)](#defining-load-trajectories)

[4.2 Input Shaping [65](#input-shaping)](#input-shaping)

[4.2.1 Theoretical Framework [65](#fundamentação-teórica)](#fundamentação-teórica)

[4.2.2 Input shaping applied to the problem [67](#input-shaping-aplicado-ao-problema)](#input-shaping-aplicado-ao-problema)

[4.3 Trajectories Based on the Differential Flatness of the System with *Input Shaping* [69](#trajetórias-baseadas-na-planicidade-diferencial-do-sistema-com-input-shaping)](#trajetórias-baseadas-na-planicidade-diferencial-do-sistema-com-input-shaping)

5. [CONTROLLER WITH TRAJECTORY GENERATOR [74](#controller-with-trajectory-generator)](#controller-with-trajectory-generator)

[5.1 Structure of the Analysis [74](#estrutura-da-análise)](#estrutura-da-análise)

[5.2 Analysis of Results [77](#analysis-of-results)](#analysis-of-results)

6.  [CONCLUSIONS [87](#conclusões)](#conclusões)

[BIBLIOGRAPHIC REFERENCES [89](#referências-bibliográficas)](#referências-bibliográficas)[APPENDIX I – CINEMATIC TRANSFORMATIONS [95](#apêndice-i-transformações-cinemáticas)](#apêndice-i-transformações-cinemáticas)[Euler Angles [95](#ângulos-de-euler)](#ângulos-de-euler

[Angular Velocity Transformation [96](#transformação-da-velocidade-angular)](#transformação-da-velocidade-angular)

[APPENDIX II – POLYNOMIAL INTERPOLATION BY PARTS [98](#appendix-ii-polynomial-interpolation-by-parts)](#appendix-ii-polynomial-interpolation-by-parts)

1.  

# INTRODUCTION

The development of unmanned aerial vehicles (UAVs), also known as *drones*, has been a major focus of research in academia and business in recent years. Compared to manned aerial vehicles, UAVs eliminate the risk to the pilot, promote significant reductions in size and cost, and have a wide range of applications. Currently, *drones* are widely used for aerial photography, remote sensing, inspection of power lines, crop spraying, and even aerial delivery.

However, more recent research has been focused on performing increasingly complex tasks. According to Ding et al. (2018), one of the research areas involving *drones* that has grown significantly in recent years is aerial manipulation, in which the aircraft physically interacts with the environment. This area involves various applications, such as cargo transport, construction, contact inspection, and remote operation. In addition to having great potential for applications, this problem attracts the interest of researchers due to the large engineering challenge involved, particularly in the areas of modeling and control.

In this context, in line with state-of-the-art in aerial control and robotics, this work proposes the control of multi-rotor aircraft for the transportation of suspended cargo via cable.

## Motivation and Applications

Currently, helicopters equipped with cargo lifting elements are used in many applications, such as transporting timber in remote, difficult-to-access areas (Figure 1.1-a), collecting and dropping water in firefighting missions (Figure 1.1-b), and manipulating large structures (Figure 1.1-c), such as transmission towers (VARGAS MORENO, 2017; PDG Aviation Services, 2018).

<img src="media/image2.png" style="width:5.90551in;height:2.66885in" />

Figure 1.1 - Applications for transporting cargo via helicopter. (a) transportation of trees (PERKOWSKI, 2015), (b) firefighting (BOB, \[s.d.]), and (c) transportation of transmission towers (SHEPHERD; JARVIS; HUNT, 2014).

However, these operations are high-risk for the pilot and require specialized training, factors that motivate the investigation of UAVs for these applications. They can perform this type of mission with greater agility, precision, and safety, and can operate intelligently without human intervention.

The use of autonomous aircraft also makes it possible to transport smaller goods (Figure 2-a), which can be used for home deliveries and sending supplies to areas that are difficult to access in disaster situations (FAUST et al., 2017), as well as rescuing people and animals in dangerous situations, launching exploration robots in remote areas, for example.

<img src="media/image3.emf" style="width:5.90551in;height:2.71038in" />

Figure 1.2 - Specific applications of *drones* with suspended cargo via cable. (a) transportation of supplies (FAUST et al., 2017), (b) mine detection (BISGAARD, 2008) and (c) water sample collection (ORE et al., 2015).

Besides serving for the general transport of goods, this system can have specific utilities. For example, Bisgaard (2008) develops a complete control solution for a helicopter in handling mine detection equipment (Figure 2-b). The aircraft maintains the apparatus stabilized close to the ground, eliminating the risk of explosion and increasing the speed of the operation. *Drones* with cables are used to collect water samples for analysis in rivers and lakes, as shown in Figure 1-c). Thus, the need for mobilizing teams with boats is eliminated, reducing cost, increasing the agility and safety of the operation (ORE et al., 2015).

In comparison to systems where the load is rigidly connected to the *drone*, the use of cables confers greater agility to the aircraft, as it does not alter its rotational inertia and allows for the carriage of objects of larger dimensions and various shapes. On the other hand, the suspended load adds complexity to the system, presenting movement that is not directly controlled and that is sensitive to external disturbances, causing significant disturbances in the aircraft's dynamics. Therefore, it is necessary to develop specialized controllers that consider this dynamic coupling to produce the desired movements for the drone and load as a whole.

## Related Works

The academic community has shown significant interest in the topic of load lifting in the last decade. Based on the approach made in the studies on crane control presented by Qian and Yi (2015) and Ramli et al. p. 20 (2017), it has been identified that research on controlling *drones* with suspended loads by cable can be divided into two categories: open-loop and closed-loop. Table 1.1 presents a summary of the main techniques used and the main reference works.

Table 1.1 - Techniques for controlling a drone with a suspended load via cable.

<table border="1">
<colgroup>
<col style="width: 12%" />
<col style="width: 45%" />
<col style="width: 41%" />
</colgroup>
<thead border="1">
<tr class="header">
<th><strong>Category</strong></th>
<th><strong>Strategy</strong></th>
<th><strong>References</strong></th>
</tr>
</thead>
<tbody border="1">
<tr class="odd">
<td rowspan="4"><strong>Open-Loop</strong></td>
<td><em Input Shaping</em></td>
<td>(BISGAARD; COUR-HARBO; BENDTSEN, 2008; KLAUSEN; FOSSEN; JOHANSEN, 2015, 2017)</td>
</tr>
<tr class="even">
<td>Trajectory Optimization - Dynamic Programming</td>
<td>(PALUNKO; FIERRO; CRUZ, 2012)</td>
</tr>
<tr class="odd">
<td>Trajectory Generation by Intelligent Agent Obtained with Reinforcement Learning</td>
<td>(FAUST et al., 2013, 2017)</td>
</tr>
<tr class="even">
<td>Trajectory Generation Based on Differentially Flat System Property</td>
<td>(SREENATH; LEE; KUMAR, 2013; SREENATH; MICHAEL; KUMAR, 2013)</td>
</tr>
<tr class="odd">
<td rowspan="7"><p><strong>Closed-Loop</strong></p>
<p><strong>System</strong></p></td>
<td>Backstepping</td>
<td>(KLAUSEN; FOSSEN; JOHANSEN, 2015, 2017)</td>
</tr>
<tr class="even">
<td>Sliding Mode Control (SMC)</td>
<td>(KUI et al., 2017; ZHOU et al., 2016)</td>
</tr>
<tr class="odd">
<td>Passive-Based Control</td>
<td>(GUERRERO et al., 2015a, 2015b; GUERRERO-SÁNCHEZ et al., 2017a)</td>
</tr>
<tr class="even">
<td>Geometric Control</td>
<td>(GOODARZI; LEE; LEE, 2014; KOTARU; WU; SREENATH, 2017; SREENATH; LEE; KUMAR, 2013; SREENATH; MICHAEL; KUMAR, 2013)</td>
</tr>
<tr class="odd">
<td>Optimal Control (iLQG, SQL, and H∞)</td>
<td>(CROUSAZ; FARSHIDIAN; BUCHLI, 2014; CROUSAZ et al., 2015; RAFFO; ALMEIDA, 2016)</td>
</tr>
<tr class="even">
<td>Model Predictive Control (MPC)</td>
<td>(ALEXIS et al., 2016; NOTTER et al., 2016; ZÜRN et al., 2016)</td>
</tr>
<tr class="odd">
<td>Adaptive Control</td>
<td>(BISGAARD; LA COUR-HARBO; DIMON BENDTSEN, 2010; DAI; LEE; BERNSTEIN, 2014; FENG et al., 2015)</td>
</tr>
</tbody
</table>

It should be noted that, despite the separation presented, solutions are often not applied in isolation. Many works present hybrid solutions, combining both types of techniques, as will also be done in this work.

### Open-loop control

The open-loop control techniques operate by modifying the reference signal based on prior information about the system's behavior without using feedback from sensors. The main techniques applied to this system are: *input shaping*, trajectory optimization, reinforcement learning, and the analytical method based on the definition of a differentially flat system.

*Input shaping* is based on the idea of inducing transient oscillations in the system and canceling them immediately afterward through the insertion of an input that would produce an opposite oscillation. This is done through the convolution of the reference signal with appropriately selected impulsive signals based on the natural frequency of the system (QIAN; YI, 2015). Several studies (BISGAARD; COUR-HARBO; BENDTSEN, 2008; KLAUSEN; FOSSEN; JOHANSEN, 2015, 2017) apply this filter to arbitrary reference signals, substantially reducing oscillations compared to a solution using only closed-loop control.

Palunko; Fierro; Cruz (2012) apply an *offline* optimization procedure using dynamic programming to determine reference trajectories that, based on cost functions obtained from a linearized and discrete model of the system, aim to minimize response oscillations. Faust et al. (2013, 2017) develop a trajectory generator without oscillations through an iterative value approximation reinforcement learning algorithm, such that the inferred policy extends to domains beyond the training situation, being robust to noise and uncertainties in the model.

Sreenath; Lee; Kumar (2013) and Sreenath; Michael; Kumar (2013) demonstrate that the system with a suspended load drone is differentially planar. This means that, given a set of output variables defined as differentiable functions up to a certain order, it is possible to determine all other variables in the system and the input forces.

### Closed-Loop Control

Given that the system is non-linear, much of the control solutions are based on tools for analyzing non-linear systems, such as the Lyapunov stability criterion. For example, Klausen; Fossen; Johansen (2015, 2017) develop a backstepping controller that guarantees the tracking of arbitrary trajectories, independent of the pendulum's motion. Kui et al (2017) and Zhou et al. (2016) also seek to control the drone's position by compensating for disturbances caused by the load, as well as external disturbances with known thresholds, by applying sliding mode control. Guerrero et al. (2015a, 2015b, 2017a) apply a controller based on the principle of passivity in order to minimize the oscillation of the load, without directly measuring its position, since the control law does not depend on it.

Many techniques utilize or are based on optimization techniques. For example, Crousaz; Farshidian; Buchli (2014) and Crousaz et al. (2015) apply similar optimal quadratic linear control techniques: iLQG and SQL, which consider the joint optimization of trajectory and control performance in a closed-loop manner. Raffo; Almeida (2016) apply *H∞* to control the position of the load under conditions of parameter uncertainty and external disturbances. Other authors propose various variations of model-based predictive control, which are based on the principle of generating optimal inputs that consider the current state and the predicted behavior of the system over a finite horizon (ALEXIS et al., 2016; NOTTER et al., 2016; ZÜRN et al., 2016).

Many authors also apply geometric control, which avoids singularities and defines coordinates in the system, and is popular in the control of *drones* (unladen) to perform aggressive maneuvers (LEE; LEOK; MCCLAMROCH, 2010). Goodarzi; Lee; Lee (2014) are able to control the *drone* to perform agile maneuvers while simultaneously stabilizing the position of the cable and load, which is modeled as a multi-joint arm in series. Sreenath; Lee; Kumar (2013); Sreenath; Michael; Kumar (2013) already propose the control of the load position with defined trajectories based on the differential flatness property of the system.

## Objective and Contributions

It is observed that the system under analysis meets various demands of society and presents a complexity that challenges researchers in the field of dynamics and control. There is a significant amount of recent work on this topic, however, it has been observed that there is still no dominant solution and that there is much room to explore new techniques. In light of this scenario, this work aims to develop a new solution that brings innovations mainly in the definition of the closed-loop controller and in the trajectory generation strategy.

Basically, this work applies a specific variation of the sliding mode control technique to control the position of the *drone*, compensating for external disturbances and those caused by the load's movement, along with the combination of two open-loop control methods to mitigate the load's oscillation: *input shaping* and trajectory generation based on the system's differential flatness property.

The sliding mode control (SMC) has already been successfully applied significantly in the control of *drones* without payload and of rolling bridges. Two recent review articles on *drone* control (MO; FARID, 2018; ÖZBEK; ÖNKOL;EFE, 2016) select controllers based on sliding mode as the best cost-effectiveness among the solutions analyzed, and various review articles on rolling bridge control point to this technique as one of the main ones for application (QIAN; YI, 2015; RAMLI et al., 2017).

Despite the success in these related applications, it has been observed that this technique has been little explored in the target application of this work. Only two papers use sliding mode control directly (KUI et al., 2017; ZHOU et al., 2016). Both apply the technique with the intention of controlling the position of the *drone* by compensating for external disturbances and those caused by the load oscillation, but they do not aim to stabilize it.

Specifically, the control technique developed in this work is based on (ZHENG; XIONG; LUO, 2014) and (XIONG; ZHENG, 2014), which control a *quadrotor* without payload. In comparison to these reference works, the solution developed innovates by adding the effect of the suspended payload to the model, by proposing an alternative definition for the sliding variables, and by developing its own strategy for determining the control parameters in order for the system to be locally stable based on the Routh-Hurwitz stability criterion.

In relation to the trajectory generation strategy, this paper tests the application of *input shaping* to polynomial trajectories defined for the drone and for the payload. In the case where a trajectory is defined for the payload, the drone's reference is obtained by using the system's differential flatness property. It is demonstrated that this combination is a simple alternative to elaborate optimization strategies for generating trajectories for the drone that can be followed by the controller. No work that combines these two techniques for this application has been identified.

## Document Structure

The present work is structured in six chapters:

- **Chapter I:** This section, in which the motivations behind the research were presented, a brief bibliographic review, and the presentation of the research's contributions were introduced;

- **Chapter II:** deduction of the dynamic model of the system through the methods of *Newton-Euler* and *Euler-Lagrange*.

- **Chapter III:** presents the control solution developed in detail, including the treatment of the system's inherent underperformance and stability analysis, as well as a verification of the controlled system's behavior in simulation;

- **Chapter IV:** discusses trajectory generation techniques based on the system's differential flatness property and *input shaping*, followed by presenting the proposed combined solution;

- **Chapter V:** analysis of the controller's performance in simulation for different trajectory configurations, including the new proposed combination,

- **Chapter VI:** compilation of results and indication of future work.

2.  

# DYNAMIC MODEL

This chapter presents the development of the dynamic model of the system, which, in general, is obtained by applying the *Newton-Euler* and *Lagrange* equations. We start with an understanding of the dynamics of a *drone* without load, and then derive the equations of the complete system so that the effects of adding the load are evident.

## Drone Without Payload

A Figure 2.1 illustrates a *quadcopter*, indicating the coordinate systems and the forces applied to it.

<img src="media/image4.emf" style="width:3.00473in;height:2.63296in" />

Figure 2.1 -Schematic representation of an unloaded drone with indication of coordinate systems, applied forces and moments, as well as the direction of propeller rotation.

As indicated in Figure 2.1, an inertial reference frame $\Sigma_{i} = \left\lbrack {\overrightarrow{e}}_{x},{\overrightarrow{e}}_{y},{\overrightarrow{e}}_{z} \right\rbrack$ is defined at the origin of the system and a non-inertial (or body) frame $\Sigma_{b} = \left\lbrack {\overrightarrow{e}}_{x}^{b},{\overrightarrow{e}}_{y}^{b},{\overrightarrow{e}}_{z}^{b} \right\rbrack$ located at the center of mass of the aircraft. Its position relative to the inertial reference frame is represented by $\overrightarrow{r} = \lbrack x,y,z\rbrack^{T}$, the vector $\overrightarrow{v} = \lbrack u,v,w\rbrack^{T}$ represents the linear velocity in the inertial reference frame, and the vector $\overrightarrow{\omega} = \lbrack p,q,r\rbrack^{T}$ represents its angular velocity in the non-inertial reference frame. The mass of the vehicle is represented by $M$, while $d$ is the distance between a rotor and its opposite, and $g$ is the gravitational acceleration.

The vehicle's orientation is defined by the Euler angles $\overrightarrow{\eta} = \lbrack\phi,\theta,\psi\rbrack^{T}$, also known as roll, pitch, and yaw angles, so that the non-inertial coordinate system is obtained through three consecutive rotations around the axes $z$, $y$, and $x$, respectively. The transformation of vector quantities defined in the body frame to the inertial frame is given by the transformation matrix (APPENDIX I – KINEMATIC TRANSFORMATIONS):

|                                                                                                                         |       |
|-------------------------------------------------------------------------------------------------------------------------|-------|
| $$\mathbf{R} = \begin{bmatrix}                                                                                          
 \cos\theta\cos\psi & \sin\phi\sin\theta\cos\psi - \cos\phi\sin\psi & \sin\phi\sin\theta + \cos\phi\sin\theta\cos\psi \\  
 \cos\theta\sin\psi & \sin\phi\sin\theta\sin\psi + \cos\phi\cos\psi & \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \\    
  - \sin\theta & \sin\phi\cos\theta & \cos\phi\cos\theta                                                                  
 \end{bmatrix}$$                                                                                                          | (2.1) |

Already, the transformation between angular velocities in the non-inertial reference frame and the rate of change of Euler angles is given by the transformation matrix (APPENDIX I – KINEMATIC TRANSFORMATIONS):

|                                                               |       |
|---------------------------------------------------------------|-------|
| $$\left\lbrack \begin{array}{r}                               
 \dot{\phi} \\                                                  
 \dot{\theta} \\                                                
 \dot{\psi}                                                     
 \end{array} \right\rbrack = \begin{bmatrix}                    
 1 & \sin\phi{tg}\theta & \cos\phi{tg}\theta \\                 
 0 & \cos\phi & - \sin\phi \\                                   
 0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta}  
 \end{bmatrix}\left\lbrack \begin{array}{r}                     
 p \\                                                           
 q \\                                                           
 r                                                              
 \end{array} \right\rbrack$$                                    | (2.2) |

A Figure 2.1 também indicates the main efforts existing in the aircraft. The main forces acting in the system are the weight force $- Mg{\overrightarrow{e}}_{z}$ and the thrust forces of the rotors, indicated as $F_{1},F_{2},F_{3}$ and $F_{4}$, applied along ${\overrightarrow{e}}_{z}^{b}$. The resulting thrust force is given by:

|                                                                             |       |
|-----------------------------------------------------------------------------|-------|
| $$F_{z}^{b} = F_{1} + F_{2} + F_{3} + F_{4}$$                               | (2.3) |
| $${\overrightarrow{F}}_{b} = \left\lbrack 0,0,F_{z}^{b} \right\rbrack^{T}$$ | (2.4) |

The aircraft also experiences moments on three axes: the thrust forces create moments around axes ${\overrightarrow{e}}_{x}^{b}$ and ${\overrightarrow{e}}_{y}^{b}$, while the drag on the propellers, which acts against the rotation of the same, creates a moment around axis ${\overrightarrow{e}}_{z}^{b}$. With $d$ being the distance between a rotor and its opposite, and $\tau_{i}$ being the drag moment on each propeller, the resulting moment on the aircraft is given by:

|                                                                                                         |       |
|---------------------------------------------------------------------------------------------------------|-------|
| $$\tau_{x}^{b} = d\left( F_{2} - F_{4} \right)$$                                                        | (2.5) |
| $$\tau_{y}^{b} = d\left( F_{1} - F_{3} \right)$$                                                        | (2.6) |
| $$\tau_{z}^{b} = \tau_{1} - \tau_{2} + \tau_{3} - \tau_{4}$$                                            | (2.7) |
| $${\overrightarrow{\tau}}_{b} = \left\lbrack \tau_{x}^{b},\tau_{y}^{b},\tau_{z}^{b} \right\rbrack^{T}$$ | (2.8) |

The thrust forces and the torque on the propellers due to drag, in turn, are proportional to the square of the rotational speed of each propeller $\Omega_{i}$:

|                                    |        |
|------------------------------------|--------|
| $$F_{i} = k_{t}\Omega_{i}^{2}$$    | (2.9)  |
| $$\tau_{i} = k_{m}\Omega_{i}^{2}$$ | (2.10) |

The constants $k_{t}$ and $k_{m}$ depend on the air density, the radius, shape, number, and geometry of the blades, as well as the drag and lift coefficients associated (PROUTY, 2001).

Thus, the mapping of the rotor speed (command variable to the motor controllers) and the forces applied to the drone, which will be the inputs to the controller to be detailed, is given by:

|                                                 |        |
|-------------------------------------------------|--------|
| $$\overrightarrow{u} = \left\{ \begin{array}{r} 
 u_{1} \\                                         
 u_{2} \\                                         
 u_{3} \\                                         
 u_{4}                                            
 \end{array} \right\} = \left\{ \begin{array}{r}  
 F_{z}^{b} \\                                     
 \tau_{x}^{b} \\                                  
 \tau_{y}^{b} \\                                  
 \tau_{z}^{b}                                     
 \end{array} \right\} = \begin{bmatrix}           
 k_{t} & k_{t} & k_{t} & k_{t} \\                 
 0 & dk_{t} & 0 & - dk_{t} \\                     
 dk_{t} & 0 & - dk_{t} & 0 \\                     
 k_{m} & - k_{m} & k_{m} & - k_{m}                
 \end{bmatrix}\left\{ \begin{array}{r}            
 \Omega_{1}^{2} \\                                
 \Omega_{2}^{2} \\                                
 \Omega_{3}^{2} \\                                
 \Omega_{4}^{2}                                   
 \end{array} \right\}$$                           | (2.11) |

It should be noted that, given this direct mapping between the action of the rotors and the resulting force and moment on the aircraft, the control problem is reduced to defining these stresses.

With this, the dynamic model of the *quadcopter* is obtained by applying the Newton-Euler equations. The translational dynamics are written in the inertial reference frame and obtained by equating the rate of change of linear motion to the sum of the external forces:

|                                                                                                                                                          |        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\frac{d}{dt}\left( M\dot{\overrightarrow{r}} \right) = \sum_{}^{}{\overrightarrow{F}}_{ext}$$                                                          | (2.12) |
| $$M\ \ddot{\overrightarrow{r}} = \mathbf{R}{\overrightarrow{F}}_{b} - Mg{\overrightarrow{e}}_{z} + {\overrightarrow{F}}_{d} + {\overrightarrow{D}}_{F}$$ | (2.13) |

In Eq. (2.13), ${\overrightarrow{F}}_{d}$ refers to the translational drag force on the aircraft, modeled as proportional to the speed of the *drone* (FREDDI; LANZON; LONGHI, 2011):

|                                                                                                              |        |
|--------------------------------------------------------------------------------------------------------------|--------|
| $${\overrightarrow{F}}_{d} = {- \left\lbrack c_{x}\dot{x},{\ c}_{y}\dot{y},c_{z}\dot{z} \right\rbrack}^{T}$$ | (2.14) |

Being $c_{x}$, $c_{y}$, and $c_{z}$ the translational drag coefficients in each direction. ${\overrightarrow{D}}_{F}$ refers to the force disturbances that are not modeled. By isolating the acceleration terms from Eq. (2.13), one obtains:

|                                                                                                                                |        |
|--------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\left\{ \begin{aligned}                                                                                                      
  & \ddot{x} = \frac{1}{M}\left( \cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi \right)u_{1} - \frac{c_{x}}{M}\dot{x} + d_{x} \\  
  & \ddot{y} = \frac{1}{M}\left( \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \right)u_{1} - \frac{c_{y}}{M}\dot{y} + d_{y} \\  
  & \ddot{z} = - g + \frac{1}{M}\left( \cos\phi\cos\theta \right)u_{1} - \frac{c_{z}}{M}\dot{z} + d_{z}                          
 \end{aligned} \right.\ $$                                                                                                       | (2.15) |

In Eq. (2.15), $d_{x}$, $d_{y}$ and $d_{z}$ refers to the effects of the ${\overrightarrow{D}}_{F}$ on each component of the translation acceleration.

Already, the rotating dynamics are taken in the body's reference frame and obtained by equating the angular velocity rate to the sum of the external moments:

|                                                                                                                                                                                           |        |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\frac{d}{dt}\left( \mathbf{I}\overrightarrow{\omega} \right) = \sum_{}^{}{\overrightarrow{\tau}}_{ext}^{b}$$                                                                            | (2.16) |
| $$\mathbf{I}\dot{\overrightarrow{\omega}} + \overrightarrow{\omega} \times \left( \mathbf{I}\overrightarrow{\omega} \right) = {\overrightarrow{\tau}}^{b} + {\overrightarrow{D}}_{\tau}$$ | (2.17) |

In Eq. (2.17), $\mathbf{I}$ is the inertia matrix of the drone, taken as $\text{diag}\text{ }\left( I_{x},I_{y},I_{z} \right)$. The rate of change of angular momentum includes, in addition to the term of angular acceleration ($\dot{\overrightarrow{\omega}}$) and a component related to the variation of the direction of the drone's angular momentum ($\overrightarrow{\omega} \times \mathbf{I}\overrightarrow{\omega}$). The external moments include, in addition to ${\overrightarrow{\tau}}^{b}$ from Eq. (2.8) and unmodeled moment disturbances (${\overrightarrow{D}}_{\tau}$).

Isolating the terms of acceleration Eq. (2.17), we obtain:

|                                                                                                   |        |
|---------------------------------------------------------------------------------------------------|--------|
| $$\left\{ \begin{aligned}                                                                         
 \dot{p} & = \frac{\left( I_{y} - I_{z} \right)}{I_{x}}qr + \frac{1}{I_{x}}u_{2} + D_{\tau}^{x} \\  
 \dot{q} = \frac{\left( I_{z} - I_{x} \right)}{I_{y}}pr + \frac{1}{I_{y}}u_{3} + D_{\tau}^{y} \\    
 \dot{r} & = \frac{\left( I_{x} - I_{y} \right)}{I_{z}}pq + \frac{1}{I_{z}}u_{4} + D_{\tau}^{z}     
 \end{aligned} \right.\ $$                                                                          | (2.18) |

From the control perspective, however, one works with the rate of variation of the Euler angles $\overrightarrow{\Omega}$, which requires the use of the transformation described in Eq. (2.2). This transformation, in turn, generates high complexity in the equations, justifying the adoption of a simplification commonly made in the literature: $\lbrack p,q,r\rbrack \approx \lbrack\dot{\phi},\dot{\theta},\dot{\psi}\rbrack$, which is exact for the equilibrium point where $\phi = 0$ and $\theta = 0$, resulting in:

|                                                                                                                         |        |
|-------------------------------------------------------------------------------------------------------------------------|--------|
| $$\left\{ \begin{aligned}                                                                                               
 \ddot{\phi} & = \frac{\left( I_{y} - I_{z} \right)}{I_{x}}\dot{\theta}\dot{\psi} + \frac{1}{I_{x}}u_{2} + d_{\phi} \\    
 \ddot{\theta} & = \frac{\left( I_{z} - I_{x} \right)}{I_{y}}\dot{\phi}\dot{\psi} + \frac{1}{I_{y}}u_{3} + d_{\theta} \\  
 \ddot{\psi} & = \frac{\left( I_{x} - I_{y} \right)}{I_{z}}\dot{\phi}\dot{\theta} + \frac{1}{I_{z}}u_{4} + d_{\psi}       
 \end{aligned} \right.\ $$                                                                                                | (2.19) |

To simplify the disturbance notation in Eq. (2.19), making it more convenient for use in the controller, the disturbance terms that result in the accelerations are summarized as $d_{\phi}$, $d_{\theta}$ and $d_{\psi}$.

Therefore, the dynamic model of the drone without payload can be summarized by the translation dynamics described in the system of equations (2.15) and the rotational dynamics, described in the system of equations (2.19).

## Drone with Payload Suspended by Cable

The dynamic model developed considers the *drone* as a rigid body with a point mass attached to its center of mass via a negligible-mass cable that is always under tension. In this way, the elastic effects of the cable, the orientation of the mass, and its interference with the *drone*'s rotational dynamics are disregarded. The suspended mass does not generate moment disturbances in the *drone*, only force disturbances. It should be noted that these considerations are in line with a large portion of the related works found in the literature (GUERRERO-SÁNCHEZ et al., 2017b; KLAUSEN; FOSSEN; JOHANSEN, 2017; SREENATH; MICHAEL; KUMAR, 2013).

The following figure illustrates the system, indicating the additional elements to the system consisting only of the *drone*.

<img src="media/image5.emf" style="width:2.72077in;height:2.71021in" />

Figure 2.2 - Schematic representation of a drone with a load suspended by a cable, indicating the references, the position of the load, as well as the tension force of the cable and the weight of the load.

A position of the load is represented by ${\overrightarrow{r}}_{L} = \left\lbrack x_{L},y_{L},z_{L} \right\rbrack^{T}$ and is related to the position of the *drone* as:

|                                                                           |        |
|---------------------------------------------------------------------------|--------|
| $${\overrightarrow{r}}_{L} = \overrightarrow{r} + l\ \overrightarrow{p}$$ | (2.20) |

$l$ corresponds to the cable length and $\overrightarrow{p}$ consists of the unit vector that points from the center of gravity of the quadcopter to the load, which is obtained through two consecutive rotations of the vector $- {\overrightarrow{e}}_{z}$: one rotation of angle $\phi_{L}$ around $x$, followed by another rotation of angle $\theta_{L}$ around the axis $y$:

|                                                                                                           |        |
|-----------------------------------------------------------------------------------------------------------|--------|
| $$\overrightarrow{p} = R_{x}\left( \phi_{L} \right)R_{y}\left( \theta_{L} \right)\left\{ \begin{array}{r} 
 0 \\                                                                                                       
 0 \\                                                                                                       
  - 1                                                                                                       
 \end{array} \right\}$$                                                                                     | (2.21) |
| $$\overrightarrow{p} = \left\{ \begin{array}{r}                                                           
  - \sin\theta_{L} \\                                                                                       
 \sin\left( \phi_{L} \right)\cos\left( \theta_{L} \right) \\                                                
  - \cos\left( \phi_{L} \right)\cos\left( \theta_{L} \right)                                                
 \end{array} \right\}$$                                                                                     | (2.22) |

Throughout this vector, the pulling force of the cable on the drone ($\overrightarrow{T} = T\overrightarrow{p}$) is applied, assuming that it is non-zero at every instant. There is also the application of the weight force on the payload and on the *drone*, as well as the propulsive force on the rotors and the drag force.

So, applying Newton's equations for the drone and the load, we get:

|                                                                                                                                                                             |        |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$M\ddot{\overrightarrow{r}} = \mathbf{R}{\overrightarrow{F}}_{b} - Mg{\overrightarrow{e}}_{z} + \overrightarrow{T} + {\overrightarrow{F}}_{d} + {\overrightarrow{D}}_{F}$$ | (2.23) |
| $$m{\ddot{\overrightarrow{r}}}_{L} = - \overrightarrow{T} - mg{\overrightarrow{e}}_{z} + {\overrightarrow{F}}_{d}^{L} + {\overrightarrow{D}}_{F}^{L}$$                      | (2.24) |

Substituting ${\overrightarrow{r}}_{L}$ and its derivatives from Eq. ((2.20)) in Eq. (2.24) and $\overrightarrow{T}$ of Eq. (2.24) in Eq. (2.23)[^1], one obtains:

|                                                                                                                                                                                                                                                                |        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$(M + m)\ddot{\overrightarrow{r}} + ml\ddot{\overrightarrow{p}} + (M + m)g{\overrightarrow{e}}_{z} = \mathbf{R}{\overrightarrow{F}}_{b} + {\overrightarrow{F}}_{d} + {\overrightarrow{F}}_{d}^{L} + {\overrightarrow{D}}_{F} + {\overrightarrow{D}}_{F}^{L}$$ | (2.25) |

$\left\lbrack {\overrightarrow{F}}_{d},{\overrightarrow{F}}_{d}^{L} \right\rbrack$ and $\left\lbrack {\overrightarrow{D}}_{F},{\overrightarrow{D}}_{F}^{L} \right\rbrack$ refer to the drag force and force disturbances applied to the *drone* and the load, respectively. The drag force on the load is also modeled as proportional to the speed, as was done for the drone (Eq. 2.14), but symmetric in the three directions:

|                                                                          |        |
|--------------------------------------------------------------------------|--------|
| $${\overrightarrow{F}}_{d}^{L} = - c_{L}{\dot{\overrightarrow{r}}}_{L}$$ | (2.26) |

Given the premises considered for the model, adding the load only affects the translation dynamics in the *drone*, Eq. (2.15), in such a way that the rotational dynamics, Eq. (2.19), remain unchanged. Furthermore, since the load is considered a point mass, its rotational dynamics are disregarded.

Developing the Eq. (2.24), we obtain a set of three equations that depend not only on the state variables of the $\left\lbrack \dot{x},\dot{y},\dot{z},\phi,\theta,\psi,\dot{\phi},\dot{\theta},\dot{\psi} \right\rbrack$, but also on the load state, described as a function of $\left\lbrack \phi_{L},\theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L},{\ddot{\phi}}_{L},{\ddot{\theta}}_{L} \right\rbrack$. However, in the Newton-Euler formulation, the behavior of these variables is not apparent. Therefore, the Lagrange formulation is applied to detail the obtained model.

For this, the *Lagrangian* of the system is defined, given by the difference between the kinetic and potential energies of the system:

|                                                                                                                                                                                                     |        |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$L = \frac{M}{2}\left( {\dot{x}}^{2} + {\dot{y}}^{2} + {\dot{z}}^{2} \right) + \frac{m}{2}\left( {\dot{x}}_{L}^{2} + {\dot{y}}_{L}^{2} + {\dot{z}}_{L}^{2} \right) - g\left( Mz + mz_{L} \right)$$ | (2.27) |

Thus, the *Lagrangian* is developed by substituting ${\overrightarrow{r}}_{L}$, Eq. (2.20), and its derivative in Eq. (2.27). With this, the dynamic relationships of the system are obtained by applying the Lagrange equation based on the generalized coordinates $\overrightarrow{q} = \left\lbrack x,y,z,\phi_{L},\theta_{L} \right\rbrack$, as shown in Eq. (2.28), where $Τ_{i}$ are the generalized forces along each coordinate:

|                                                                                                                                    |        |
|------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\frac{\partial}{\partial t}\left( \frac{\partial L}{\partial{\dot{q}}_{i}} \right) - \frac{\partial L}{\partial q_{i}} = Τ_{i}$$ | (2.28) |

Analyzing the efforts present in Equations (2.23) and (2.24), the thrust and drag forces on the *drone* are already described in $x$, $y$, and $z$. However, the drag force on the payload is described as a function of its Cartesian coordinates, and therefore must be converted to generalized coordinates. To do this, the potential function associated with the drag forces is defined as:

|                                                                                                                                                                                              |        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$P = - \frac{1}{2}\left\lbrack c_{x}{\dot{x}}^{2} + c_{y}{\dot{y}}^{2} + c_{z}{\dot{z}}^{2} + c_{L}\left( {\dot{x}}_{L}^{2} + {\dot{y}}_{L}^{2} + {\dot{z}}_{L}^{2} \right) \right\rbrack$$ | (2.29) |

Thus, the generalized effort associated with the translational drag force along each coordinate $i$ is given by:

|                                                          |        |
|----------------------------------------------------------|--------|
| $$Τ_{i}^{P} = \frac{\partial P}{\partial{\dot{q}}_{i}}$$ | (2.30) |

By convenience, unknown terms of disturbance are transferred directly to each coordinate. Thus, the equations of the dynamics of the resulting system are given by:

| $$\left\{ \begin{aligned}                                                                                                                                                                                                               
  & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial\dot{x}} \right) - \frac{\partial L}{\partial x} & = & \left( \cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi \right)u_{1} + \frac{\partial P}{\partial\dot{x}} + D_{x} \\  
  & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial\dot{y}} \right) - \frac{\partial L}{\partial y} & = & \left( \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \right)u_{1} + \frac{\partial P}{\partial\dot{y}} + D_{y} \\  
  & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial\dot{z}} \right) - \frac{\partial L}{\partial z} & = & \left( \cos\phi\cos\theta \right)u_{1} + \frac{\partial P}{\partial\dot{z}} + D_{z} \\                             
  & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial{\dot{\phi}}_{L}} \right) - \frac{\partial L}{\partial\phi_{L}} & = & \frac{\partial P}{\partial{\dot{\phi}}_{L}} + D_{\phi_{L}} \\                                       
  & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial{\dot{\theta}}_{L}} \right) - \frac{\partial L}{\partial\theta_{L}} & = & \frac{\partial P}{\partial{\dot{\theta}}_{L}} + D_{\theta_{L}}                                  
 \end{aligned} \right.\ $$                                                                                                                                                                                                                | (2.31) |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|

Developing the system of equations (2.31), the system can be written in matrix form:

|                                                                                                                                                                                                                                                                                                                                                             |        |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\mathbf{M}\left( \overrightarrow{q} \right)\ddot{\overrightarrow{q}} + \mathbf{C}\left( \overrightarrow{q},\dot{\overrightarrow{q}} \right)\dot{\overrightarrow{q}} + \mathbf{G}\left( \overrightarrow{q} \right) = \mathbf{B}\left( \overrightarrow{q} \right)u_{1} + \mathbf{P}\left( \overrightarrow{q} \right)\dot{\overrightarrow{q}} + \mathbf{D}$$ | (2.32) |

In what:

|                                                                                                                                                                                                                 |        |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\mathbf{M}\left( \overrightarrow{q} \right) = \begin{bmatrix}                                                                                                                                                 
 (M + m) & 0 & 0 & 0 & - ml\ c\theta_{L} \\                                                                                                                                                                       
 0 & (M + m) & 0 & ml\ c\theta_{L}\ c\theta_{L} & - ml\ s\phi_{L}\ c\theta_{L} \\                                                                                                                                 
 0 & 0 & (M + m) & ml\ s\phi_{L}\ c\theta_{L} & ml\ c\phi_{L}\ s\theta_{L} \\                                                                                                                                     
 0 & ml\ c\theta_{L}\ c\theta_{L} & ml\ s\phi_{L}\ c\theta_{L} & ml^{2}\ {c\theta_{L}}^{2} & 0 \\                                                                                                                 
  - ml\ c\theta_{L} & - ml\ s\phi_{L}\ c\theta_{L} & ml\ c\phi_{L}\ s\theta_{L} & 0 & ml^{2}                                                                                                                      
 \end{bmatrix}$$                                                                                                                                                                                                  | (2.33) |
| $$\mathbf{C}\left( \overrightarrow{q},\dot{\overrightarrow{q}} \right) = \left\lbrack \mathbf{Ο}_{\mathbf{5 \times 3}} \middle| \begin{matrix}                                                                  
 0 & ml\ s\theta_{L}\ {\dot{\theta}}_{L} \\                                                                                                                                                                       
  - ml\left( s\phi_{L}c\theta_{L}\ {\dot{\phi}}_{L} + c\phi_{L}s\theta_{L}\ {\dot{\theta}}_{L} \right) & - ml\left( s\phi_{L}c\theta_{L}\ {\dot{\theta}}_{L} + c\phi_{L}s\theta_{L}\ {\dot{\phi}}_{L} \right) \\  
 ml\left( c\phi_{L}c\theta_{L}\ {\dot{\phi}}_{L} - s\phi_{L}s\theta_{L}\ {\dot{\theta}}_{L} \right) & ml\left( c\phi_{L}c\theta_{L}\ {\dot{\theta}}_{L} - s\phi_{L}s\theta_{L}\ {\dot{\phi}}_{L} \right) \\       
  - ml^{2}s\theta_{L}c\theta_{L}{\dot{\theta}}_{L} & - ml^{2}s\theta_{L}c\theta_{L}{\dot{\phi}}_{L} \\                                                                                                            
 ml^{2}s\theta_{L}c\theta_{L}{\dot{\phi}}_{L} & 0                                                                                                                                                                 
 \end{matrix} \right\rbrack$$                                                                                                                                                                                     | (2.34) |
| $$\mathbf{G}\left( \overrightarrow{q} \right) = \left\lbrack \begin{array}{r}                                                                                                                                   
 0 \\                                                                                                                                                                                                             
 0 \\                                                                                                                                                                                                             
 (M + m)g \\                                                                                                                                                                                                      
 mgl\sin\phi_{L}\cos\theta_{L} \\                                                                                                                                                                                 
 mgl\cos\phi_{L}\sin\theta_{L}                                                                                                                                                                                    
 \end{array} \right\rbrack$$                                                                                                                                                                                      | (2.35) |
| $$\mathbf{B}\left( \overrightarrow{q} \right) = \left\lbrack \begin{array}{r}                                                                                                                                   
 u_{x} \\                                                                                                                                                                                                         
 u_{y} \\                                                                                                                                                                                                         
 u_{z} \\                                                                                                                                                                                                         
 0 \\                                                                                                                                                                                                             
 0                                                                                                                                                                                                                
 \end{array} \right\rbrack = \left\lbrack \begin{array}{r}                                                                                                                                                        
 \cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi \\                                                                                                                                                                 
 \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \\                                                                                                                                                                 
 \cos\phi\cos\theta \\                                                                                                                                                                                            
 0 \\                                                                                                                                                                                                             
 0                                                                                                                                                                                                                
 \end{array} \right\rbrack$$                                                                                                                                                                                      | (2.36) |
| $$\mathbf{P}\left( \overrightarrow{q} \right) = \begin{bmatrix}                                                                                                                                                 
  - \left( C_{x} + C_{L} \right) & 0 & 0 & 0 & C_{L}lc\theta_{L} \\                                                                                                                                               
 0 & - \left( C_{y} + C_{L} \right) & 0 & - C_{L}lc\phi_{L}c\theta_{L} & C_{L}ls\phi_{L}s\theta_{L} \\                                                                                                            
 0 & 0 & - \left( C_{x} + C_{L} \right) & - C_{L}ls\phi_{L}c\theta_{L} & - C_{L}lc\phi_{L}s\theta_{L} \\                                                                                                          
 0 & - C_{L}lc\phi_{L}c\theta_{L} & - C_{L}ls\phi_{L}c\theta_{L} & - C_{L}l^{2}c\theta_{L}^{2} & 0 \\                                                                                                             
 C_{L}lc\theta_{L} & C_{L}ls\phi_{L}s\theta_{L} & - C_{L}lc\phi_{L}s\theta_{L} & 0 & - C_{L}l^{2}                                                                                                                 
 \end{bmatrix}$$                                                                                                                                                                                                  | (2.37) |

It can be observed that $\mathbf{M}\left( \overrightarrow{q} \right)$ is a positive definite matrix, i.e., it is symmetric and the terms on the main diagonal are strictly positive except when $\theta_{L} = \frac{\pi}{2}$. With this, it is taken as a constraint that $\theta_{L}$ and $\phi_{L}$ are less than $\frac{\pi}{2}$, so that the cable always moves below the level of the aircraft. Given this scenario, it is possible to isolate the acceleration term $\ddot{\overrightarrow{q}}$ from Eq. (2.32):

|                                                                                                                                                                                                                                                                                                                                                                                            |        |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\ddot{\overrightarrow{q}} = \mathbf{M}^{- 1}\left( \overrightarrow{q} \right)\left\lbrack - \mathbf{C}\left( \overrightarrow{q},\dot{\overrightarrow{q}} \right)\dot{\overrightarrow{q}} - \mathbf{G}\left( \overrightarrow{q} \right) + \mathbf{B}\left( \overrightarrow{q} \right)u + \mathbf{P}\left( \overrightarrow{q} \right)\dot{\overrightarrow{q}} + \mathbf{D} \right\rbrack$$ | (2.38) |

To control purposes, the term "drag" is encompassed as a disturbance, so the resulting equations from the development of Eq. (2.38) with this consideration are given by:

|                                                                                                                                                                                                          |        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\left\{ \begin{aligned}                                                                                                                                                                                
  & \ddot{x} & = & f_{x}\left( \theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L} \right) & + & b_{x}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right)\  & u_{1} & + d_{x} \\                                
  & \ddot{y} & = & f_{y}\left( \phi_{L},\theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L} \right) & + & b_{y}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right) & u_{1} & + d_{y} \\                         
  & \ddot{z} & = & f_{z}\left( \phi_{L},\theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L} \right) & + & b_{z}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right)\  & u_{1} & + d_{z} \\                       
  & {\ddot{\phi}}_{L} & = & f_{\phi_{L}}\left( \theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L} \right) & + & b_{\phi_{L}}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right)\  & u_{1} & + d_{\phi_{L}} \\  
  & {\ddot{\theta}}_{L} & = & f_{\theta_{L}}\left( \theta_{L},{\dot{\phi}}_{L} \right) & + & b_{\theta_{L}}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right)\  & u_{1} & + d_{\theta_{L}}                
 \end{aligned} \right.\ $$                                                                                                                                                                                 | (2.39) |

The last term of each equation in the system (2.39) refers to the effect of disturbances (including drag) on the accelerations.

Finally, it is worth noting that the expansion of the terms in Eq. (2.39) is useful for implementing the controller, as it fully utilizes the model written in this form. Thus, here is the expanded equation for each of the terms in Eq. (2.39):

|                                                                                                                                                                                                                     |        |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$f_{x} = - \frac{ml\sin\theta_{L}}{(M + m)}\left( \cos^{2}\theta_{L}{\dot{\phi}}_{L}^{2} + {\dot{\theta}}_{L}^{2} \right)$$                                                         | (2.40) |
| $$b_{x} = \frac{m}{M(M + m)}\left\lbrack s\theta_{L}c\theta_{L}\left( u_{y}s\phi_{L} - u_{z}c\phi_{L}\  \right) + u_{x}\left( \frac{M}{m} + {c\theta_{L}}^{2} \right) \right\rbrack$$                               | (2.41) |
| $$f_{y} = \frac{ml\sin\phi_{L}\cos\theta_{L}}{(M + m)}\left( \cos^{2}\theta_{L}{\dot{\phi}}_{L}^{2} + {\dot{\theta}}_{L}^{2} \right)$$                                                                              | (2.42) |
| $$b_{y} = \frac{m}{M(M + m)}\left\lbrack s\phi_{L}c\theta_{L}\left( u_{x}s\theta_{L} + u_{z}c\phi_{L}c\theta_{L} \right) + u_{y}\left( \frac{M}{m} + 1 - {s\phi_{L}}^{2}{c\theta_{L}}^{2} \right) \right\rbrack$$   | (2.43) |
| $$f_{z} = - \frac{ml\cos\phi_{L}\cos\theta_{L}}{(M + m)}\left( \cos^{2}\theta_{L}{\dot{\phi}}_{L}^{2} + {\dot{\theta}}_{L}^{2} \right) - g$$                                                                        | (2.44) |
| $$b_{z} = \frac{m}{M(M + m)}\left\lbrack c\phi_{L}c\theta_{L}\left( - u_{x}s\theta_{L} + u_{y}s\phi_{L}c\theta_{L} \right) + u_{z}\left( \frac{M}{m} + 1 - {c\phi_{L}}^{2}{c\theta_{L}}^{2} \right) \right\rbrack$$ | (2.45) |
| $$f_{\phi_{L}} = 2{tg}\theta_{L}{\dot{\phi}}_{L}{\dot{\theta}}_{L}$$                                                                                                                                                | (2.46) |
| $$b_{\phi_{L}} = - \frac{\left( u_{y}\cos\phi_{L} + u_{z}\sin\phi_{L} \right)}{Ml\cos\theta_{L}}$$                                                                                                                  | (2.47) |

3.  

# CONTROL

This chapter presents the control solution developed for the drone system with suspended payload, as described in Chapter 2. As previously introduced, the goal is to control the position of the aircraft while maintaining the stable position of the payload. To achieve this, a sliding mode controller is developed, which controls the position of the drone, taking into account the coupled dynamics of the payload, but does not aim to stabilize it. The task of stabilizing the payload is left to the trajectory generator described in the next chapter.

Firstly, a qualitative analysis of the dynamic model and the control problem is presented, evaluating the system's operating characteristics and referring to other relevant control solutions. Subsequently, the controller is detailed, presenting everything from the formulation and stability analysis to the verification of the system's behavior in simulation.

## Introduction

### Characteristics of Operation

Analyzing the equations of the dynamics of a drone with a payload, it is observed that the system has six degrees of freedom ($\overrightarrow{q} = \left\lbrack x,y,z,\phi,\theta,\psi,\phi_{L},\theta_{L} \right\rbrack^{T}$) for four independent control inputs ($\overrightarrow{u} = \left\lbrack u_{1},u_{2},u_{3},u_{4} \right\rbrack^{T}$). This difference characterizes the system as under-actuated, which means that the control action is not capable of acting on all degrees of freedom independently. Compared to the system consisting only of the drone, the proposed challenge adds two unactuated degrees of freedom to the problem.

It is verified that $u_{2}$, $u_{3}$ and $u_{4}$, the torque efforts on the *drone*, appear explicitly in the equations of the variables that describe its orientation $\phi$, $\theta$ and $\psi$ (Eq. (2.19)). This means that, independently, it is possible to control the three variables through these three inputs. However, $\phi$ and $\theta$ also configure the orientation of the thrust force, which is responsible for causing the displacement of the aircraft. Therefore, the input signals $u_{2}$ and $u_{3}$ indirectly affect the position $\lbrack x,y,z\rbrack$. This influence can also be verified by noting the presence of the angles in the equations of the aircraft's translation dynamics (Eq. (2.15)).

The thrust force $u_{1}$, on the other hand, is explicitly present in the equations of the variables that describe the position of the drone $\lbrack x,y,z\rbrack$ and the payload $\left\lbrack \phi_{L},\theta_{L} \right\rbrack$ (Eq. (2.39)). However, $u_{1}$ points vertically at the equilibrium point, around which the system variables are maintained, exerting influence only on the acceleration $\ddot{z}$ under this condition. In other words, in the most frequent operating conditions, the thrust exerts control primarily along $z$.

These operating characteristics motivated researchers in the field of multi-rotor aircraft control to develop cascaded control solutions, as illustrated in the following figure.

<img src="media/image6.emf" style="width:5.502in;height:1.99852in" />

Figure 3.1 – Waterfall control structure for multicopters. Adapted from MO; FARID (2018)

Basically, the solution presents a cascaded position controller to an attitude controller (or orientation). Given the desired positions, the position controller generates the signal $u_{1}$ and reference values for the roll and pitch angles ($\phi_{d}$ and $\theta_{d}$), which, along with the desired orientation $\psi_{d}$, feed the attitude controller that generates the signals $u_{2}$, $u_{3}$ and $u_{4}$. Typically, $u_{4}$ is determined independently based on the desired yaw angle. The displacement in directions $x$ and $y$ is achieved through the action of $u_{2}$ and $u_{3}$, which direct the thrust force $u_{1}$ towards the direction of error reduction, as illustrated in the Figure 3.2.

<img src="media/image7.emf" style="width:3.64493in;height:2.13275in" />

Figure 3.2 - Illustration of the effect of $u_{2}$ and $u_{3}$ on the horizontal displacement of the drone.

As illustrated in Figure 3.2, the action of $u_{2}$ directs the propulsion in the direction of displacing the aircraft along ${\overrightarrow{e}}_{y}^{b}$, while the action of $u_{3}$ has an indirect influence on the displacement along ${\overrightarrow{e}}_{x}^{b}$.

### Sliding Control (CMD)

#### Basic Concepts

The CMD is a robust, nonlinear control technique, meaning it is insensitive to external disturbances and parameter uncertainties. Its implementation consists of:

1) Define the variable sliding calls, which are carefully designed functions of system variables that, when cancelled, cause the system to exhibit stable behavior;

2) Design the input signals to drive the sliding variables to zero and maintain them in this condition.

For example, given a nonlinear system written in the form:

|                      |        |
|----------------------|--------|
| $$\dot{x} = f(x,u)$$ | (3.48) |

being $x$ a system variable and $u$ the input signal, it is possible to define, for example, a sliding variable as a linear combination of the state variable and its derivative:

|                                             |        |
|---------------------------------------------|--------|
| $$s = \dot{x} + \lambda x,\ \ \lambda > 0$$ | (3.49) |

It can be observed that, when $s = 0$, the following is observed:

|                                              |        |
|----------------------------------------------|--------|
| $${\dot{x} = - \lambda x                     
 }{x = x(0)e^{- \lambda t}                     
 }{\dot{x} = - \lambda x(0)e^{- \lambda t}}$$  | (3.50) |

In this situation, $x$ and $\dot{x}$ converge asymptotically to zero. Therefore, when the control signal is set appropriately, the phase portrait of the system becomes similar to that shown in Figure 3.3 when the sliding variables are defined as in the example of Eq. (3.49).

<img src="media/image8.emf" style="width:3.58953in;height:2.42462in" />

Figure 3.3 - Characteristic phase portrait of a system controlled by a sliding mode controller for a linear sliding variable. Adapted from GHAZALI et al. (2011).

As shown in Figure 3.3, the state corresponding to $s = 0$ corresponds to the straight line indicated in the phase portrait. This state is referred to as the sliding surface or mode. Considering $s \neq 0$ in the initial state, the controlled system is first driven to the sliding surface, executing the "approach phase", and then continues to slide along the surface to the equilibrium point, executing the "sliding phase".

There are various solutions around this concept. Generally, the CMD versions differ in the way they determine the sliding variables and the strategy they use to perform the approximation and sliding phases. To learn more about the technique and its variations, the following sources are recommended (QIAN; YI, 2015; SHTESSEL et al., 2013; UTKIN; GULDNER; SHI, 2009).

#### CMD Applied to Drones with Cargo Suspended by Cable

Two works in the literature apply sliding mode control to this system (KUI et al., 2017; ZHOU et al., 2016). Basically, they use the same control principle commonly used for drones, as shown in Figure 3.1 (Section 3.1.1).

(KUI et al., 2017) assume the existence of a force applied to the aircraft with independent components along each axis of the inertial coordinate system. From a geometric analysis of this vector through the Eq. (2.36), it is possible to determine what the thrust force and the ideal roll and pitch angles should be to produce this input. In this way, the system can be treated as fully actuated, so that each component of the virtual force is designed to control the aircraft's position along each axis through sliding modes. The resulting thrust force from the geometric transformation is passed on, while the calculated roll and pitch angles are passed as a reference to an attitude controller that also applies classical sliding mode control for each axis, resulting in the torques to be sent to the aircraft's actuation system.

It can be observed that, in this approach, the determination of the pitch and roll angles is entirely dependent on the output of the position controller. This structure allows for the use of reference values determined externally, as is done in this work with the trajectory generator.

It is also worth noting that the articles do not clearly state the interdependence between the drone and the load accelerations in the dynamic equation. Considering the dynamic model in terms of generalized coordinates, Eq. (2.39), the "virtual forces" mentioned do not manifest in a way that exactly translates into a desired orientation, as happens in the drone equations without load.

In contrast, the controller developed in this work explicitly takes into account the underperformance characteristic of the system, without using the virtual force feature; it allows for the external determination of references for roll and pitch angles; and it considers the complete dynamic model, considering all mutual interactions between the *drone* and the load.

## Control Strategy

As described in Chapter 2, the system's accelerations are described by the equations (2.39) and (2.19), which represent the translation and rotation dynamics, respectively. In order to perform the control, the system is divided into two, one fully actuated, consisting of the variables $z$ and $\psi$, and another partially actuated, consisting of the variables $x$, $y$, $\phi$ and $\theta$:

|                                                                                   |        |
|-----------------------------------------------------------------------------------|--------|
| $$\left\{ \begin{aligned}                                                         
 \ddot{z} & = f_{x} + b_{x}u_{1} + d_{z} \\                                         
 \ddot{\psi} & = f_{\psi} + b_{\psi}u_{4} + d_{\psi}                                
 \end{aligned} \right.\ \ \ \ \ \ \ \ \ \ \ \text{(Subistema totalmente atuado)}$$  | (3.51) |
| $$\left\{ \begin{aligned}                                                         
 \ddot{x} & = f_{x} + b_{x}u_{1} + d_{x} \\                                         
 \ddot{y} & = f_{y} + b_{y}u_{1} + d_{y} \\                                         
  \\                                                                                
 \ddot{\phi} & = f_{\phi} + b_{\phi}u_{2} + d_{\psi} \\                             
 \ddot{\theta} & = f_{\theta} + b_{\theta}u_{3} + d_{\theta}                        
 \end{aligned} \right.\ \ \ \ \ \ \ \ \ \ \ (\text{Subsistema sub-atuado)}$$        | (3.52) |

The state of the load, which is directly associated with the variables $\phi_{L}$ and $\theta_{L}$, is not directly controlled. Its stabilization is achieved through the generation of appropriate trajectories, as will be described in Section 4.

In this context, the solution developed consists of two groups of controllers operating in a cascaded sliding mode, as illustrated in Figure 3.4.

<img src="media/image9.emf" style="width:4.46481in;height:2.66038in" />

Figure 3.4 - Control structure of the drone system with suspended load via cable.

As illustrated in Figure 3.4, the first CMD is responsible for controlling the fully actuated subsystem based on the desired values for altitude and heading angle up to its second derivatives, determining $u_{1}$ and $u_{4}$. The second controller commands the sub-actuated subsystem based on the horizontal position references and roll and pitch angles up to the second derivative, as well as $u_{1}$ obtained previously, generating the remaining control signals: $u_{2}$ and $u_{3}$.

### Command for the Fully Operational Subsystem ($z,\psi$)

To control the aircraft's altitude and yaw, the classic variation of the sliding mode controller is applied to nonlinear systems, replicating what has already been done by other authors (XIONG; ZHENG, 2014; ZHENG; XIONG; LUO, 2014).

#### Controller Deduction

First, the following variables are defined:

|                                                                                                         |        |
|---------------------------------------------------------------------------------------------------------|--------|
| $$s_{1} = \left( {\dot{z}}_{d} - \dot{z} \right) + \lambda_{z}\left( z_{d} - z \right)$$                | (3.53) |
| $$s_{2} = \left( {\dot{\psi}}_{d} - \dot{\psi} \right) + \lambda_{\psi}\left( \psi_{d} - \psi \right)$$ | (3.54) |

The goal of the SMC is to drive these variables to zero so that, once in this condition, the subsystem variables stabilize such that $z \rightarrow z_{d}$ and $\psi \rightarrow \psi_{d}$. In fact, when $s_{1} = 0$, one has:

|                                                                                                          |        |
|----------------------------------------------------------------------------------------------------------|--------|
| $${\left( {\dot{z}}_{d} - \dot{z} \right) = - \lambda_{z}\left( z_{d} - z \right)                        
 }{\left( z_{d} - z \right) = \left( z_{d} - z \right)(0)e^{- \lambda_{z}t}                                
 }{\left( {\dot{z}}_{d} - \dot{z} \right) = - \lambda_{z}\left( z_{d} - z \right)(0)e^{- \lambda_{z}t}}$$  | (3.55) |

Thus, $\left( z_{d} - z \right) \rightarrow 0$ and $\left( {\dot{z}}_{d} - \dot{z} \right) \rightarrow 0$ asymptotically. The same applies to $s_{2} = 0$, where $\left( \psi_{d} - \psi \right) \rightarrow 0$ and $\left( {\dot{\psi}}_{d} - \dot{\psi} \right) \rightarrow 0$ asymptotically.

The next step consists of defining the system inputs that perform the regularization of the variables. To do this, first, the derivatives of $s_{1}$ and $s_{2}$ are extracted:

|                                                                                                                                 |        |
|---------------------------------------------------------------------------------------------------------------------------------|--------|
| $${\dot{s}}_{1} = \left( {\ddot{z}}_{d} - \ddot{z} \right) + \lambda_{z}\left( z_{d} - z \right)$$                              | (3.56) |
| $${\dot{s}}_{2} = \left( {\ddot{\psi}}_{d} - \ddot{\psi} \right) + \lambda_{\psi}\left( {\dot{\psi}}_{d} - \dot{\psi} \right)$$ | (3.57) |

It can be observed that ${\dot{s}}_{1}$ and ${\dot{s}}_{2}$ contain the accelerations $\ddot{z}$ and $\ddot{\psi}$, these defined as a function of the inputs $u_{1}$ and $u_{4}$ according to the system of equations (3.51). Thus, it is possible to control ${\dot{s}}_{1}$ and ${\dot{s}}_{2}$ in such a way as to stabilize $s_{1}$ and $s_{2}$ as desired. With this, it is intended that:

|                                                                                                            |        |
|------------------------------------------------------------------------------------------------------------|--------|
| $${\dot{s}}_{1} = - \kappa_{1}s_{1} - \eta_{1}{sign}{\left( s_{1} \right),\ \ \kappa_{1},\eta_{1} > 0\ }$$ | (3.58) |
| $${\dot{s}}_{2} = - \kappa_{2}s_{2} - \eta_{2}{sign}\left( s_{2} \right),\ \ \kappa_{2},\eta_{2} > 0$$     | (3.59) |

Where:

|                                       |        |
|---------------------------------------|--------|
| $${sign}(x) = \left\{ \begin{aligned} 
 1,\ \ \  & se\ x \geq 0 \\             
  - 1,\ \ \  & se\ x < 0                
 \end{aligned} \right.\ $$              | (3.60) |

Substituting (3.56) and (3.57) in (3.58) and (3.59) given the accelerations written as a function of the inputs as described in Eq. (3.51) without disturbances, we find:

|                                                                                                                    |        |
|--------------------------------------------------------------------------------------------------------------------|--------|
| $$u_{1} = \frac{{\ddot{z}}_{d} - f_{z} + \kappa_{1}s_{1} + \eta_{1}{sign}\left( s_{1} \right)}{b_{z}}$$            | (3.61) |
| $$u_{4} = \frac{{\ddot{\psi}}_{d} - f_{\psi} + \kappa_{2}s_{2} + \eta_{2}{sign}\left( s_{2} \right)\ }{b_{\psi}}$$ | (3.62) |

#### Stability Analysis

The motivation behind the definition of ${\dot{s}}_{1}$ and ${\dot{s}}_{2}$ is based on the Lyapunov stability theory, which establishes the criterion that, given a system written in the form $\dot{x} = f(x)$ with $x = 0$ as a point of equilibrium, a function $V(x):\mathbb{R}^{n}\mathbb{\rightarrow R}$ is called a Lyapunov candidate function and the system is stable in the Lyapunov sense if:

1) $V(x) = 0$ if and only if $x = 0$;

2)  $V(x) > 0$ for all $x \neq 0$;

3) $\dot{V}(x) \leq 0$ $\rightarrow$ The system is stable locally;

4) $\dot{V}(x) < 0$ for all $x \neq 0 \rightarrow$ the system is asymptotically stable;

A candidate *Lyapunov* function can be interpreted as an energy function of the system that is always dissipated over time until zero, when the system reaches the equilibrium point (QIAN; YI, 2015). With this, the following candidate *Lyapunov* functions are defined:

|                                                      |        |
|------------------------------------------------------|--------|
| $$V_{1}\left( s_{1} \right) = \frac{1}{2}s_{1}^{2}$$ | (3.63) |
| $$V_{2}\left( s_{2} \right) = \frac{1}{2}s_{2}^{2}$$ | (3.64) |

It can be observed that the conditions (a) $V(0) = 0$ and (b) $V(x) > 0\ (x \neq 0$ are satisfied, since $V_{1}$ and $V_{2}$ are quadratic functions. Differentiating $V_{1}$, one obtains:

| $${{\dot{V}}_{1} = s_{1}{\dot{\mathbf{s}}}_{\mathbf{1}}                                                                                                                                                                                                                             
 }{{\dot{V}}_{1} = s_{1}\left\lbrack \left( {\ddot{z}}_{d} - \ddot{\mathbf{z}} \right) + \lambda_{z}\left( {\dot{z}}_{d} - \dot{z} \right) \right\rbrack                                                                                                                              
 }{{\dot{V}}_{1} = s_{1}\left\lbrack \left( {\ddot{z}}_{d} - f_{z} - b_{z}\mathbf{u}_{\mathbf{1}} - d_{z} \right) + \lambda_{z}\left( {\dot{z}}_{d} - \dot{z} \right) \right\rbrack = s_{1}\left\lbrack - \kappa_{1}s_{1} - \eta_{1}{sign}\left( s_{1} \right) - d_{z} \right\rbrack  
 }{{\dot{V}}_{1} = - \kappa_{1}s_{1}^{2} - \eta_{1}\left| s_{1} \right| - \mathbf{d}_{\mathbf{z}}\mathbf{s}_{\mathbf{1}} \leq - \kappa_{1}s_{1}^{2} - \eta_{1}\left| s_{1} \right| + \left| d_{z} \right|\left| s_{1} \right|}$$                                                      | (3.65) |

Taking $D_{z} = \max\left( \left| d_{z} \right| \right)$:

| $${{\dot{V}}_{1} \leq - \kappa_{1}s_{1}^{2} - \eta_{1}\left| s_{1} \right| + \left| \mathbf{d}_{\mathbf{z}} \right|\left| \mathbf{s}_{\mathbf{1}} \right| \leq - \kappa_{1}s_{1}^{2} - \eta_{1}\left| s_{1} \right| + D_{z}\left| s_{1} \right| 
 }{{\dot{V}}_{1} \leq - \kappa_{1}s_{1}^{2} + \left( D_{z} - \eta_{1} \right)\left| s_{1} \right|}$$                                                                                                                                              | (3.66) |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|

It can be observed that ${\dot{V}}_{1} < 0$ for $\kappa_{1} \geq 0$ and $\eta_{1} > D_{z}$. Similarly, ${\dot{V}}_{2} < 0$ for $\kappa_{2} \geq 0$ and $\eta_{2} \geq D_{\psi}$, with $D_{\psi} = \max\left( \left| d_{\psi} \right| \right)$. Therefore, under these conditions, the subsystem is asymptotically stable in the sense of *Lyapunov* and robust against disturbances that do not exceed the defined limits.

#### Relevant Observations

A key point to highlight is that the control signal is discontinuous, due to the presence of the function ${sign}(s)$. This characteristic causes a noisy input, which leads to the phenomenon called "chattering". The Figure 3.5 presents the characteristic phase portrait of a sliding mode controlled system, exhibiting this effect.

‘<img src="media/image10.emf" style="width:2.95283in;height:2.09375in" />

Figure 3.5 -Illustration of the effect of *chattering* during the sliding phase. Adapted from HOSSAIN et al. (2017).

As shown in Figure 3.5, the discontinuous term in the input causes the system to make small jumps around the sliding surface, characterizing the phenomenon of *chattering*. This occurs during the sliding phase, as it is when the sliding variable oscillates around zero, causing the control signal to switch between $+ \eta$ and $- \eta$ due to the term ${sign}(s)$. With this, the system can respond with unwanted oscillations, potentially leading to instability.

This feature is one of the most disadvantageous aspects of the sliding mode controller, but there are ways to mitigate it. The simplest way is to approximate the function ${sign}(s)$ with a continuous function, such as the sigmoid and hyperbolic tangent. However, doing so makes the controller no longer ideal, although it can still produce good results in practice (SHTESSEL et al., 2013).

Another relevant observation is that the terms associated with the constants $\kappa_{1}$ and $\kappa_{2}$ are not strictly necessary to guarantee the system's convergence. The Figure 3.6 presents an example of the behavior of a sliding variable, illustrating the contribution of each term.

<img src="media/image11.emf" style="width:3.80181in;height:2.85075in" />

Figure 3.6 - Example of convergence of a sliding variable with exponential, constant, and combined decay. $s$

A Figure 3.6 shows that $s$ exhibits linear decay for $\dot{s} = - \eta|s|$ and exponential decay for $\dot{s} = - \kappa s$. The first component is responsible for ensuring convergence within a finite time and overcoming external disturbances (as demonstrated previously), while the main role of the second is to accelerate convergence when far from zero.

### CMD of the Subsystem

There are several variations of sliding mode control for controlling underactuated systems in the literature (ASHRAFIUON; ERWIN, 2004, 2008; SANKARANARAYANAN; MAHINDRAKAR, 2009; WANG et al., 2004; WANG; LIU; YI, 2007; XU; ÖZGÜNER, 2008). Basically, the strategy for extending the use of this technique to this type of system consists of defining sliding variables that combine components under the direct influence of control signals with variables without direct actuation, and then determining the input to ensure the convergence of these surfaces and the system variables during the sliding phase.

Specifically, the technique used in this work is based on (ZHENG; XIONG; LUO, 2014) and (XIONG; ZHENG, 2014), which control a quadcopter without a payload. It should be noted that, compared to these works, the solution developed, in addition to adding the effect of the suspended payload to the model, innovates by proposing alternative definitions for the sliding variables and control parameters.

#### Controller Deduction

The intuition behind the controller is similar to that used in quadcopter control, as illustrated previously in Figure 3.2: $u_{2}$ is aimed at reducing the error along ${\overrightarrow{e}}_{y}^{b}$, while $u_{3}$ acts to reduce the error along ${\overrightarrow{e}}_{x}^{b}$. Thus, the error in the position of the aircraft projected on the plane $xy$ is calculated as:

|                                        |        |
|----------------------------------------|--------|
| $$\left\{ \begin{array}{r}             
 {\widetilde{x}}_{b} \\                  
 {\widetilde{y}}_{b}                     
 \end{array} \right\} = \begin{bmatrix}  
 \cos\psi & \sin\psi \\                  
  - \sin\psi & \cos\psi                  
 \end{bmatrix}\left\{ \begin{array}{r}   
 x_{d} - x \\                            
 y_{d} - y                               
 \end{array} \right\}$$                  | (3.67) |

In Eq. (3.67), it is assumed that $\psi$ is time-invariant, i.e., $\dot{\psi} = \ddot{\psi} = 0$. Therefore, the derivatives of Eq. (3.67) are given by:

|                                        |        |
|----------------------------------------|--------|
| $$\left\{ \begin{array}{r}             
 {\dot{\widetilde{x}}}_{b} \\            
 {\dot{\widetilde{y}}}_{b}               
 \end{array} \right\} = \begin{bmatrix}  
 \cos\psi & \sin\psi \\                  
  - \sin\psi & \cos\psi                  
 \end{bmatrix}\left\{ \begin{array}{r}   
 {\dot{x}}_{d} - \dot{x} \\              
 {\dot{y}}_{d} - \dot{y}                 
 \end{array} \right\}$$                  | (3.68) |
| $$\left\{ \begin{array}{r}             
 {\ddot{\widetilde{x}}}_{b} \\           
 {\ddot{\widetilde{y}}}_{b}              
 \end{array} \right\} = \begin{bmatrix}  
 \cos\psi & \sin\psi \\                  
  - \sin\psi & \cos\psi                  
 \end{bmatrix}\left\{ \begin{array}{r}   
 {\ddot{x}}_{d} - \ddot{x} \\            
 {\ddot{y}}_{d} - \ddot{y}               
 \end{array} \right\}$$                  | (3.69) |

This consideration seems reasonable, given that the control conditions $\psi$ are favorable for achieving convergence in the short term (XIONG; ZHENG, 2014).

With this, the sliding variables and their derivatives are defined as:

|                                                                                                                                                                                                                                |        |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$s_{3} = \lambda_{1}{\dot{\widetilde{x}}}_{b} + \lambda_{2}{\widetilde{x}}_{b} + \lambda_{3}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \lambda_{4}\left( \theta_{d} - \theta \right)$$                                | (3.70) |
| $$s_{4} = \lambda_{5}{\dot{\widetilde{y}}}_{b} + \lambda_{6}{\widetilde{y}}_{b} + \lambda_{7}\left( {\dot{\phi}}_{d} - \dot{\phi} \right) + \lambda_{8}\left( \phi_{d} - \phi \right)$$                                        | (3.71) |
| $${\dot{s}}_{3} = \lambda_{1}{\ddot{\widetilde{x}}}_{b} + \lambda_{2}{\dot{\widetilde{x}}}_{b} + \lambda_{3}\left( {\ddot{\theta}}_{d} - \ddot{\theta} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right)$$ | (3.72) |
| $${\dot{s}}_{4} = \lambda_{5}{\ddot{\widetilde{y}}}_{b} + \lambda_{6}{\dot{\widetilde{y}}}_{b} + \lambda_{7}\left( {\ddot{\phi}}_{d} - \ddot{\phi} \right) + \lambda_{8}\left( {\dot{\phi}}_{d} - \dot{\phi} \right)$$         | (3.73) |

It can be observed that the sliding variables are defined as the linear combination of the errors of the system variables. Unlike the sliding variables defined for the fully actuated subsystem, the stability condition for when $s_{3} = 0$ and $s_{4} = 0$ is not trivial. At this point, it is assumed that stability will be guaranteed, and in Section 3.2.2.3, a stability analysis is performed for the sliding condition.

It can be observed that the equations that describe the derivatives of the sliding variables contain the accelerations $\ddot{\phi}$ and $\ddot{\theta}$, which are defined as a function of the inputs $u_{2}$ and $u_{3}$ according to the system of equations (3.52). Thus, it is expected to be possible to derive ${\dot{s}}_{4}$ and ${\dot{s}}_{5}$ in order to stabilize $s_{4}$ and $s_{5}$ as desired. It is also noted that the terms ${\ddot{\widetilde{x}}}_{b}$ and ${\ddot{\widetilde{y}}}_{b}$ contain the terms $\ddot{x}$ and $\ddot{y}$, which are defined as a function of $u_{1}$, to which the value already determined by the autopilot for this input is assigned, behaving as if it were a constant in this context (XIONG; ZHENG, 2014).

Thus, similarly to what was done for the fully actuated subsystem, it is desired that:

|                                                                            |        |
|----------------------------------------------------------------------------|--------|
| $${\dot{s}}_{3} = - \kappa_{3}s_{3} - \eta_{3}{sign}\left( s_{3} \right)$$ | (3.74) |
| $${\dot{s}}_{4} = - \kappa_{4}s_{4} - \eta_{4}{sign}\left( s_{4} \right)$$ | (3.75) |

Equating equations (3.72) and (3.73) with (3.74) and (3.75), substituting $\ddot{\psi}$ and $\ddot{\theta}$ with the model (3.52) and isolating $u_{2}$ and $u_{3}$, we obtain:

|                                                                                                                                                                                                                                                                                                          |        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$u_{3} = \frac{\lambda_{1}{\ddot{\widetilde{x}}}_{b} + \lambda_{2}{\dot{\widetilde{x}}}_{b} + \lambda_{3}\left( {\ddot{\theta}}_{d} - f_{\theta} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \kappa_{3}s_{3} + \eta_{3}{sign}\left( s_{3} \right)}{\lambda_{3}b_{\theta}}$$ | (3.76) |
| $$u_{2} = \frac{\lambda_{5}{\ddot{\widetilde{y}}}_{b} + \lambda_{6}{\dot{\widetilde{y}}}_{b} + \lambda_{7}\left( {\ddot{\phi}}_{d} - f_{\phi} \right) + \lambda_{8}\left( {\dot{\phi}}_{d} - \dot{\phi} \right) + \kappa_{4}s_{4} + \eta_{4}{sign}\left( s_{4} \right)}{\lambda_{7}b_{\phi}}$$           | (3.77) |

#### Analysis of the Stability of Shifting Variables

As made in the CMD of the fully enabled subsystem, the stability of the sliding variables is determined by the Lyapunov stability theory. Therefore, the following Lyapunov candidate functions are defined:

|                                                      |        |
|------------------------------------------------------|--------|
| $$V_{3}\left( s_{3} \right) = \frac{1}{2}s_{3}^{2}$$ | (3.78) |
| $$V_{4}\left( s_{4} \right) = \frac{1}{2}s_{4}^{2}$$ | (3.79) |

It can be observed that the conditions (a) $V(0) = 0$ and (b) $V(x) > 0\ (x \neq 0$) are satisfied, since $V_{3}$ and $V_{4}$ consist of quadratic functions. Taking the derivative of $V_{3}$, we obtain:

|                                                                                                                                                                                                                                                                                                                                                                                                                                                     |        |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $${{\dot{V}}_{3} = s_{3}{\dot{\mathbf{s}}}_{\mathbf{3}} = s_{3}\left\lbrack \lambda_{1}{\ddot{\widetilde{\mathbf{x}}}}_{\mathbf{b}} + \lambda_{2}{\dot{\widetilde{x}}}_{b} + \lambda_{3}\left( {\ddot{\theta}}_{d} - \ddot{\mathbf{\theta}} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) \right\rbrack                                                                                                                     
 }{= s_{1}\left\lbrack \lambda_{1}\left( \cos\psi\left( {\ddot{x}}_{d} - f_{x} - b_{x}u_{1}\  - d_{x} \right) + \sin\psi\left( {\ddot{y}}_{d} - f_{y} - b_{y}u_{1}\  - d_{y} \right) \right)\  + \lambda_{2}{\dot{\widetilde{x}}}_{b}\ \ \  + \lambda_{3}\left( {\ddot{\theta}}_{d} - f_{\theta} - b_{\theta}\mathbf{u}_{\mathbf{3}} - d_{\theta} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) \right\rbrack                 
 }{{\dot{V}}_{3} = - \kappa_{3}s_{3}^{2} - \eta_{3}\left| s_{3} \right| - \left\lbrack \left( \lambda_{1} + \lambda_{3} \right)\left( \cos\psi d_{x} + \sin\psi d_{y} \right) + d_{\theta} \right\rbrack\mathbf{s}_{\mathbf{3}} \leq - \kappa_{3}s_{3}^{2} - \eta_{3}\left| s_{3} \right| + \left( \left| \lambda_{1} + \lambda_{3} \right|\left| \cos\psi d_{x} + \sin\psi d_{y} \right| + \left| d_{\theta} \right| \right)\left| s_{3} \right|}$$  | (3.80) |

Taking $D_{{\widetilde{x}}_{b}} = \max\left( \left| \lambda_{1} + \lambda_{3} \right|\left| \cos\psi d_{x} + \sin\psi d_{y} \right| + \left| d_{\theta} \right| \right)$:

|                                                                                                                          |        |
|--------------------------------------------------------------------------------------------------------------------------|--------|
| $${{\dot{V}}_{3} \leq - \kappa_{3}s_{3}^{2} - \eta_{3}\left| s_{3} \right| + D_{{\widetilde{x}}_{b}}\left| s_{3} \right| 
 }{{\dot{V}}_{3} \leq - \kappa_{3}s_{3}^{2} + \left( D_{{\widetilde{x}}_{b}} - \eta_{3} \right)\left| s_{3} \right|}$$     | (3.81) |

It can be observed that ${\dot{V}}_{3} < 0$ corresponds to $\kappa_{3} \geq 0$ and $\eta_{3} > D_{{\widetilde{x}}_{b}}$. Similarly, ${\dot{V}}_{4} < 0$ corresponds to $\kappa_{4} \geq 0$ and $\eta_{4} > D_{{\widetilde{y}}_{b}}$, with $D_{{\widetilde{y}}_{b}} = \max\left( \left| \lambda_{5} + \lambda_{7} \right|\left| - \sin\psi d_{x} + \cos\psi d_{y} \right| + \left| d_{\phi} \right| \right)$. Therefore, under these conditions, the subsystem is asymptotically stable in the Lyapunov sense (for the variables $s_{3}$ and $s_{4}$) and robust against disturbances that do not exceed the defined limits.

#### Analysis of System Stability on Slippery Surfaces

The most trivial way to prove the stability of the system during the sliding phase would be to observe that the system takes the form $\dot{x} = - \mathbf{K}x$, where $\mathbf{K}$ has only positive values on the main diagonal, such that the system is asymptotically stable in the variable $x$. However, the characteristic of sub-optimality and the form of definition of the sliding variables does not favor this scenario. Another suitable approach would be to define a Lyapunov function $V(x)$ candidate under the sliding condition and observe that $V(x) < 0$ (SANKARANARAYANAN; MAHINDRAKAR, 2009). However, this approach does not appear to be trivial. Finally, one resorts to local stability analysis around the equilibrium point based on the work of (ASHRAFIUON; ERWIN, 2008; ZHENG; XIONG; LUO, 2014).

The central idea consists of determining the coefficients $\lambda_{1}$ through $\lambda_{8}$ based on the stability condition of *Routh-Hurwitz* applied to the linearized sliding surface equations around the equilibrium point.

##### Stability in the Slide at $s_{3}$

Firstly, the equations (3.72) and (3.70) are rearranged for the sliding conditions, where ${\dot{s}}_{3} = 0$ and $s_{3} = 0$:

|                                                                                                                                                                                                                                                    |        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $${\ddot{\theta}}_{d} - \ddot{\theta} = - \frac{\lambda_{1}}{\lambda_{3}}{\ddot{\widetilde{x}}}_{b} - \frac{\lambda_{2}}{\lambda_{3}}{\dot{\widetilde{x}}}_{b} - \frac{\lambda_{4}}{\lambda_{3}}\left( {\dot{\theta}}_{d} - \dot{\theta} \right)$$ | (3.82) |
| $${\dot{\widetilde{x}}}_{b} = - \frac{\lambda_{2}}{\lambda_{1}}{\widetilde{x}}_{b} - \frac{\lambda_{3}}{\lambda_{1}}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) - \frac{\lambda_{4}}{\lambda_{1}}\left( \theta_{d} - \theta \right)$$         | (3.83) |

Replacing Eq. (3.83) in Eq. (3.82), results in:

|                                                                                                                                                                                                                                                                                                                                                                                                                                       |        |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $${\ddot{\theta}}_{d} - \ddot{\theta} = - \frac{\lambda_{1}}{\lambda_{3}}{\ddot{\widetilde{x}}}_{b} + \frac{\lambda_{2}^{2}}{\lambda_{1}\lambda_{3}}{\widetilde{x}}_{b} + \left( \frac{\lambda_{2}}{\lambda_{1}} - \frac{\lambda_{4}}{\lambda_{3}} \right)\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \frac{\lambda_{2}\lambda_{4}}{\lambda_{1}\lambda_{3}}\left( \theta_{d} - \theta \right)$$ | (3.84) |

Then, a variable redefinition is performed, $y_{1} = \theta_{d} - \theta$, $y_{2} = {\dot{\theta}}_{d} - \dot{\theta}$, and $y_{3} = {\widetilde{x}}_{b}$, resulting in the following system:

| $$\left\{ \begin{aligned}                                                                                                                                                                                                                                                                                             
 {\dot{y}}_{1} & = y_{2} \\                                                                                                                                                                                                                                                                                             
 {\dot{y}}_{2} & = - \frac{\lambda_{1}}{\lambda_{3}}{\ddot{\widetilde{x}}}_{b}\left( y_{1},y_{2} \right) + \frac{\lambda_{2}\lambda_{4}}{\lambda_{1}\lambda_{3}}y_{1} + \left( \frac{\lambda_{2}}{\lambda_{1}} - \frac{\lambda_{4}}{\lambda_{3}} \right)y_{2} + \frac{\lambda_{2}^{2}}{\lambda_{1}\lambda_{3}}y_{3} \\  
 {\dot{y}}_{3} & = - \frac{\lambda_{2}}{\lambda_{1}}y_{1} - \frac{\lambda_{3}}{\lambda_{1}}y_{2} - \frac{\lambda_{4}}{\lambda_{1}}y_{3}                                                                                                                                                                                 
 \end{aligned} \right.\ $$                                                                                                                                                                                                                                                                                              | (3.85) |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|

In Eq. (3.85), ${\ddot{\widetilde{x}}}_{b}\left( y_{1},y_{2} \right)$ is given by Eq. (3.69), such that $\theta = \theta_{d} - y_{1}$ and $\dot{\theta} = {\dot{\theta}}_{d} - y_{2}$. It can be observed that, $y_{1} \rightarrow 0$, $y_{2} \rightarrow 0$ and $y_{3} \rightarrow 0$ when the variables are close to their equilibrium points, i.e., $\theta \rightarrow \theta_{d}$, $\dot{\theta} \rightarrow {\dot{\theta}}_{d}$ and ${\widetilde{x}}_{b} \rightarrow 0$. Defining the vector $\overrightarrow{y} = \left\{ y_{1},y_{2},y_{3} \right\}^{T}$, the equilibrium point ${\overrightarrow{y}}_{e} = \left\{ 0,0,0 \right\}$ and $\dot{\overrightarrow{y}} = f\left( \overrightarrow{y} \right)$ (system 3.85), the linearization of $f\left( \overrightarrow{y} \right)$ around the equilibrium point is given by:

|                                                                                                                                                                                       |        |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$f'\left( \overrightarrow{y} \right) = \left. \ \mathbf{J}(f) \right|_{\overrightarrow{y} = \left\{ 0,0,0 \right\}}\ \overrightarrow{y} + f\left( {\overrightarrow{y}}_{e} \right)$$ | (3.86) |

At Eq. (3.86), $\left. \ \mathbf{J}(f) \right|_{\overrightarrow{y} = {\overrightarrow{y}}_{e}}$, or simply $\mathbf{J}$**,** it is the Jacobian of the function $f(y)$ evaluated at the equilibrium point $y_{e}$, defined as:

| $$\mathbf{J} = \begin{bmatrix}                                                                                                                                                                                                     
 \frac{\partial f_{1}}{\partial y_{1}} & \frac{\partial f_{1}}{\partial y_{2}} & \frac{\partial f_{1}}{\partial y_{3}} \\                                                                                                            
 \frac{\partial f_{2}}{\partial y_{1}} & \frac{\partial f_{2}}{\partial y_{2}} & \frac{\partial f_{2}}{\partial y_{3}} \\                                                                                                            
 \frac{\partial f_{3}}{\partial y_{1}} & \frac{\partial f_{3}}{\partial y_{2}} & \frac{\partial f_{3}}{\partial y_{3}}                                                                                                               
 \end{bmatrix}_{y = y_{e}} = \begin{bmatrix}                                                                                                                                                                                         
 0 & 1 & 0 \\                                                                                                                                                                                                                        
  - \frac{\lambda_{1}}{\lambda_{3}}F + \frac{\lambda_{2}\lambda_{4}}{\lambda_{1}\lambda_{3}} & \left( \frac{\lambda_{2}}{\lambda_{1}} - \frac{\lambda_{4}}{\lambda_{3}} \right) & \frac{\lambda_{2}^{2}}{\lambda_{1}\lambda_{3}} \\  
  - \frac{\lambda_{4}}{\lambda_{1}} & - \frac{\lambda_{3}}{\lambda_{1}} & - \frac{\lambda_{2}}{\lambda_{1}}                                                                                                                          
 \end{bmatrix}$$                                                                                                                                                                                                                     | (3.87) |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|

Given that:

|                                                                                                                                                                                                                                          |        |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$F = \frac{\partial{\ddot{\widetilde{x}}}_{b}}{\partial y_{1}} = \frac{u_{1}c(\phi)m}{M(M + m)}\left\lbrack c\theta_{d}\ A\left( \psi,\phi_{L},\theta_{L} \right) - s\theta_{d}B\left( \psi,\phi_{L},\theta_{L} \right) \right\rbrack$$ | (3.88) |
| $$A\left( \psi,\phi_{L},\theta_{L} \right) = \frac{M}{m} + \left\lbrack \left( c\psi c\theta_{L} + s\psi s\phi_{L}s\theta_{L} \right)^{2} + s\psi^{2}c\phi_{L}^{2} \right\rbrack$$                                                       | (3.89) |
| $$B\left( \psi,\phi_{L},\theta_{L} \right) = c\phi_{L}c\theta_{L}\left( s\psi s\phi_{L}c\theta_{L} - c\psi s\theta_{L} \right)$$                                                                                                         | (3.90) |

Thus, the linear system described by Eq. (3.86) is stable if the eigenvalues of the Jacobian matrix given by Eq. (3.87) are all negative, such that $\overrightarrow{y}$ and $\dot{\overrightarrow{y}}$ exhibit asymptotic convergence to the equilibrium point. It is expected that this condition can be achieved by adjusting the coefficients $\lambda_{1}$, $\lambda_{2}$, $\lambda_{3}$, and $\lambda_{4}$. With this, the characteristic polynomial of $\mathbf{J}$ is calculated, doing:

|                                                                                                                                                                          |        |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\det\left( p\mathbb{I} - \mathbf{J} \right) = 0$$                                                                                                                      | (3.91) |
| $$p^{3} + \left( \frac{\lambda_{4}}{\lambda_{3}} \right)p^{2} + \left( \frac{\lambda_{1}}{\lambda_{3}}F \right)p + \left( \frac{\lambda_{2}}{\lambda_{3}}F \right) = 0$$ | (3.92) |

The eigenvalues are given by the roots of the characteristic polynomial. However, instead of calculating the roots directly, the Routh-Hurwitz stability criterion is applied, which infers the stability of the system by simply evaluating the coefficients of the characteristic polynomial. Given the third-degree characteristic polynomial written in the form $p^{3} + a_{2}p^{2} + a_{1}p + a_{0} = 0$, it is a necessary and sufficient condition for the associated time-invariant linear system to be stable: $a_{2} > 0$, $a_{0} > 0$ and $a_{2}a_{1} > a_{0}$ (NISE, 2011). Therefore, for the characteristic polynomial (3.92):

|                                                                                              |        |
|----------------------------------------------------------------------------------------------|--------|
| $$1.\ \ \frac{\lambda_{4}}{\lambda_{3}} > 0$$                                                | (3.93) |
| $$2.\ \ \frac{\lambda_{2}}{\lambda_{3}}F > 0$$                                               | (3.94) |
| $$3.\ \ \frac{\lambda_{1}\lambda_{4}}{\lambda_{3}^{2}}F > \frac{\lambda_{2}}{\lambda_{3}}F$$ | (3.95) |

It is observed that the two last conditions depend on the behavior of $F$. Therefore, it is worthwhile to study this function in order to define the control parameters. Assuming that $u_{1} > 0$ (positive propulsion) and $- \frac{\pi\ }{2} < \left\lbrack \psi,\theta\text{,}\phi_{L},\theta_{L} \right\rbrack < \frac{\pi\ }{2}$, it is possible to make $F$ greater than zero by conditioning the value given to $\theta_{d}$, which is a domain of the trajectory generator.

Firstly, it is observed that the term in question Eq. (3.88) ($u_{1}\cos\phi$) is always positive for the operating conditions imposed. Secondly, it is verified that $\theta_{d}$ assigns weights to $A$ and $B$ such that $A$ is maximized and $B$ is neutralized for $\theta_{d} = 0$, and the opposite occurs for $\theta_{d} = \pm \frac{\pi}{2}$.

Another observed behavior is that $A\left( \psi,\phi_{L},\theta_{L} \right)$ is always positive, and its minimum possible value is $\frac{M}{m}$. Therefore, $\cos\theta_{d}A$ is greater than $0$ under operating conditions. Conversely, the term $B\left( \psi,\phi_{L},\theta_{L} \right)$ can take both positive and negative values, but is limited to $\pm \frac{1}{2}$, and therefore $\left| \sin\theta_{d}B \right|$ is less than $\frac{1}{2}$ under operating conditions.

This behavior led to the suspicion that there might be a maximum value for $\theta_{d}$ such that $F$ becomes positive regardless of the values of $A$ and $B$. Indeed, the case where $A$ assumes its minimum value and $B$ assumes its maximum value (in magnitude) depicts the scenario where $F$ assumes the smallest possible value. Therefore, the value of $\theta_{d}$ that cancels out $F$ under these extreme conditions ($\theta_{d}^{*}$) is obtained by making $F = 0$:

|                                                                                                                                                                                                                                       |        |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
| $$\cos{\left( \theta_{d}^{*} \right)\min\left( |A| \right)} - \sin\left( \theta_{d}^{*} \right)\max\left( |B| \right) = 0 \rightarrow \tan\left( \theta_{d}^{*} \right) = \pm \frac{\min\left( |A| \right)}{\max\left( |B| \right)}$$ | (3.96) |
| $$\theta_{d}^{*} = \pm {atan}\left( \frac{2M}{m} \right)$$                                                                                                                                                                            | (3.97) |

Therefore, one can state that:

|                                                                                  |        |
|----------------------------------------------------------------------------------|--------|
| $$F > 0\ \ se\ \ \left| \theta_{d} \right| < {atan}\left( \frac{2M}{m} \right)$$ | (3.98) |

In reality, the definition of this limit is conservative, as there is no possible state $\left\lbrack \psi,\phi_{L},\theta_{L} \right\rbrack$ that would cause $A$ and $B$ to take on their minimum and maximum (in absolute value) values simultaneously. The actual limit occurs, among other states, when $\psi = 0$, $\phi_{L} = 0$, and $\theta_{L} = \pm \frac{\pi}{4}$, in which $A\left( 0,0, \pm \frac{\pi}{4} \right) = \frac{M}{m} + \frac{1}{2}$ and $B\left( 0,0, \pm \frac{\pi}{4} \right) = \frac{1}{2}$, leading to $\theta_{d}^{*} = \pm {atan}\left( \frac{2M}{m} + 1 \right)$ and a more flexible stability condition in which:

|                                                                                            |        |
|--------------------------------------------------------------------------------------------|--------|
| $$F \geq 0\ \ se\ \ \left| \theta_{d} \right| \leq {atan}\left( \frac{2M}{m} + 1 \right)$$ | (3.99) |

Thus, assuming $F > 0$, the stability relationships of the controller coefficients (Equations 3.93, 3.94, and 3.95) can be summarized as:

|                                                                           |         |
|---------------------------------------------------------------------------|---------|
| $$\frac{\lambda_{4}}{\lambda_{3}} > \frac{\lambda_{2}}{\lambda_{1}} > 0$$ | (3.100) |

It is observed that the relationship between the coefficients associated with $\phi$ and $\dot{\phi}$ should be greater than the relationship between the coefficients associated with ${\widetilde{y}}_{b}$ and ${\dot{\widetilde{y}}}_{b}$, and that no coefficient can take a value of zero or have a sign different from the others.

##### Stability in the Slide at $s_{4}$

Basically, the same procedure is applied as for $s_{3}$, in which ${\widetilde{y}}_{b}$ is similar to ${\widetilde{x}}_{b}$ and $\phi$ is similar to $\theta$. The Jacobian (Eq. 3.87), the characteristic polynomial (Eq. (3.92)) and the stability conditions (Equations 3.93, 3.94 and 3.95) are identical, replacing $\lambda_{1},\ \lambda_{2},\ \lambda_{3}$ and $\lambda_{4}$ with $\lambda_{5},\lambda_{6},\lambda_{7}$ and $\lambda_{8}$ respectively, and the term $F$ by the term $G$, as follows:

|                                                                                                                                                                                                                                          |         |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$G = \frac{\partial{\ddot{\widetilde{y}}}_{b}}{\partial y_{1}} = \frac{u_{1}\ m}{M(M + m)}\left\lbrack - c\phi_{d}\ C\left( \psi,\phi_{L},\theta_{L} \right) + s\phi_{d}\ D\left( \psi,\phi_{L},\theta_{L} \right) \right\rbrack$$      | (3.101) |
| $$C = \frac{M}{m} + \left\lbrack \left( s\psi c\theta_{L} + c\psi s\phi_{L}s\theta_{L} \right)^{2} + c\psi^{2}c\phi_{L}^{2} \right\rbrack$$                                                                                              | (3.102) |
| $$D = c\theta c\phi_{L}c\theta_{L}\left( c\psi s\phi_{L}c\theta_{L} + s\psi s\theta_{L} \right) + s\theta\left\lbrack s\phi_{L}s\theta_{L}c\theta_{L}\left( s\psi^{2} - c\psi^{2} \right) - c\phi_{L}^{2}s\theta_{L}^{2} \right\rbrack$$ | (3.103) |

Similarly, the term in evidence is always positive for the imposed operating conditions, and the term within the brackets, $\phi_{d}$, assigns weights to $C$ and $D$ such that the module of $C$ is maximized and $D$ is neutralized for $\phi_{d} = 0$, and the opposite occurs for $\phi_{d} = \pm \frac{\pi}{2}$. It is also possible to verify that $C \geq \frac{M}{m}$ and $|D| \leq \frac{1}{2}$. However, unlike what occurs for $F$, the strictly positive term $C$ is multiplied by $- \cos\phi_{d}$, which is always negative for the operating conditions.

Thus, the goal is to ensure that $G$ is always less than zero by conditioning the value of $\phi_{d}$. It is suspected that there may be a maximum value for $\phi_{d}$ that makes $G$ negative regardless of the values of $C$ and $D$. Indeed, the case in which $C$ assumes its minimum value and $D$ assumes its maximum value (in magnitude) represents the scenario in which $G$ assumes the largest possible value. Therefore, the value of $\phi_{d}$ that cancels $G$ under these extreme conditions ($\phi_{d}^{*}$) is obtained by setting $G = 0$:

|                                                                                                                                                                                                                                     |         |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$- \cos{\left( \phi_{d}^{*} \right)\min\left( |C| \right)} + \sin\left( \phi_{d}^{*} \right)\max\left( |D| \right) = 0 \rightarrow \tan\left( \theta_{d}^{*} \right) = \pm \frac{\min\left( |C| \right)}{\max\left( |D| \right)}$$ | (3.104) |
| $$\phi_{d}^{*} = \pm {atan}\left( \frac{2M}{m} \right)$$                                                                                                                                                                            | (3.105) |

Can be stated that:

|                                                                                      |         |
|--------------------------------------------------------------------------------------|---------|
| $$G < 0\ \ \ \ se\ \ \ \left| \phi_{d} \right| < {atan}\left( \frac{2M}{m} \right)$$ | (3.106) |

So, assuming $G < 0$, the stability relationships of the controller coefficients (Equations 3.93, 3.94, and 3.95) can be summarized as:

|                                                                                                                                                                                                                                                                                                                |         |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$\frac{\lambda_{8}}{\lambda_{7}} > 0\ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \  \rightarrow \ \ \ \ \ \ \ \ {sign}\left( \lambda_{8} \right) = {sign}\left( \lambda_{7} \right)$$                                                                                                                                    | (3.107) |
| $$\frac{\lambda_{5}\lambda_{8}}{\lambda_{7}^{2}} < \frac{\lambda_{6}}{\lambda_{7}} < 0\ \ \ \  \rightarrow \ \ \ \ \ \ \ {sign}\left( \lambda_{5} \right) = {sign}\left( \lambda_{6} \right) \neq {sign}\left( \lambda_{7} \right),\ \ \frac{\lambda_{8}}{\lambda_{7}} > \frac{\lambda_{6}}{\lambda_{5}} > 0$$ | (3.108) |

In summary, it is observed that the relationship between the coefficients associated with $\phi$ and $\dot{\phi}$ should be greater than the relationship between the coefficients associated with ${\widetilde{y}}_{b}$ and ${\dot{\widetilde{y}}}_{b}$. Additionally, it is required that the coefficients associated with $\phi$ and $\dot{\phi}$ have opposite signs to the coefficients associated with ${\widetilde{y}}_{b}$ and ${\dot{\widetilde{y}}}_{b}$.

### Summary

<table border="0">
<colgroup>
<col style="width: 100%" />
</colgroup>
<thead border="0">
<tr class="header">
<th style="padding-left: 10px;">
<p>Variables: </p>
<table border="0">
<colgroup>
<col style="width: 100%" />
</colgroup>
<tbody border="0">
<tr class="odd">
<td style="padding-left: 10px;">
<span class="math display">
<em>s₁</em>= (
<em>ż</em><sub><em>d</em></sub>−
<em>ż</em>) +
<em>λ</em><sub><em>z</em></sub>(
<em>z</em><sub><em>d</em></sub>−
<em>z</em>), 
<em>λ</em><sub><em>z</em></sub> &gt; 0
</span>
</td>
</tr>
<tr class="even">
<td style="padding-left: 10px;">
<span class="math display">
<em>s₂</em>= (
<em>ψ̇</em><sub><em>d</em></sub>−
<em>ψ̇</em>) +
<em>λ</em><sub><em>ψ</em></sub>(
<em>ψ</em><sub><em>d</em></sub>−
<em>ψ</em>), 
<em>λ</em><sub><em>ψ</em></sub> &gt; 0
</span>
</td>
</tr>
<tr class="odd">
<td style="padding-left: 10px;">
<span class="math display">$$s_{3} = \lambda_{1}{\dot{\widetilde{x}}}_{b} + \lambda_{2}{\widetilde{x}}_{b} + \lambda_{3}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \lambda_{4}\left( \theta_{d} - \theta \right),\ \ \frac{\lambda_{4}}{\lambda_{3}} &gt; \frac{\lambda_{2}}{\lambda_{1}} &gt; 0$$</span>
</td>
</tr>
<tr class="even">
<td style="padding-left: 10px;">
<span class="math display">$$s_{4} = \lambda_{5}{\dot{\widetilde{y}}}_{b} + \lambda_{6}{\widetilde{y}}_{b} + \lambda_{7}\left( {\dot{\phi}}_{d} - \dot{\phi} \right) + \lambda_{8}\left( \phi_{d} - \phi \right),\ \ \frac{\lambda_{8}}{\lambda_{7}} &gt; 0,\ \ \frac{\lambda_{5}\lambda_{8}}{\lambda_{7}^{2}} &lt; \frac{\lambda_{6}}{\lambda_{7}} &lt; 0$$</span>
</td>
</tr>
</tbody>
</table>
</th>
</tr>
</tbody>
</table>
<p>Auxiliary variable: </p>
<p>
<span class="math display">$$\left\{ \begin{array}{r}
{\widetilde{x}}_{b}^{(n)} \\
{\widetilde{y}}_{b}^{(n)}
\end{array} \right\} = \begin{bmatrix}
\cos\psi &amp; \sin\psi \\
 - \sin\psi &amp; \cos\psi
\end{bmatrix}\left\{ \begin{array}{r}
x_{d}^{(n)} - x^{(n)} \\
y_{d}^{(n)} - y^{(n)}
\end{array} \right\},\ \ \dot{\psi} = \ddot{\psi} = 0,\ \ \ \ \ n = 0,\ 1,\ 2$$</span>
</p>
<p>Inputs:</p>
<table border="0">
<colgroup>
<col style="width: 100%" />
</colgroup>
<tbody border="0">
<tr class="odd">
<td style="padding-left: 10px;">
<span class="math display">$$u_{1} = \frac{{\ddot{z}}_{d} - f_{z} + \kappa_{1}s_{1} + \eta_{1}{sign}\left( s_{1} \right)}{b_{z}},\ \ \kappa_{1} &gt; 0,\ \ \eta_{1} &gt; \max\left( \left| d_{z} \right| \right),\ \ b_{z} \neq 0$$</span>
</td>
</tr>
<tr class="even">
<td style="padding-left: 10px;">
<span class="math display">$$u_{4} = \frac{{\ddot{\psi}}_{d} - f_{\psi} + \kappa_{2}s_{2} + \eta_{2}{sign}\left( s_{2} \right)\ }{b_{\psi}},\ \ \kappa_{2} &gt; 0,\ \ \eta_{2} &gt; \max\left( \left| d_{\psi} \right| \right),\ \ b_{\psi} \neq 0$$</span>
</td>
</tr>
<tr class="odd">
<td style="padding-left: 10px;">
<p>
<span class="math display">$$u_{3} = \frac{\lambda_{1}{\ddot{\widetilde{x}}}_{b} + \lambda_{2}{\dot{\widetilde{x}}}_{b} + \lambda_{3}\left( {\ddot{\theta}}_{d} - f_{\theta} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \kappa_{3}s_{3} + \eta_{3}{sign}\left( s_{3} \right)}{\lambda_{3}b_{\theta}},\ $$</span>
</p>
<p>
<span class="math display">κ₃ &gt; 0, η₃ &gt; max(|λ₁ + λ₃|cos(ψdₓ + sin(ψdᴨ) + |dθ|, bθ ≠ 0)</span>
</p>
</td>
</tr>
<tr class="even">
<td style="padding-left: 10px;">
<p>
<span class="math display">$$u_{2} = \frac{\lambda_{5}{\ddot{\widetilde{y}}}_{b} + \lambda_{6}{\dot{\widetilde{y}}}_{b} + \lambda_{7}\left( {\ddot{\phi}}_{d} - f_{\phi} \right) + \lambda_{8}\left( {\dot{\phi}}_{d} - \dot{\phi} \right) + \kappa_{4}s_{4} + \eta_{4}{sign}\left( s_{4} \right)}{\lambda_{7}b_{\phi}},$$</span>
</p>
<p>
<span class="math display">κ₄ &gt; 0, η₄ &gt; max(|λ₅ + λ₇||−sin(ψdₓ + cos(ψdᴨ) + |dϕ|, bϕ ≠ 0)</span>
</p>
</td>
</tr>
</tbody>
</table>

## Simulation

To evaluate the performance of the controller, a simulation of the system is performed using the MATLAB software, in which the dynamic equation described in Chapter 2 is integrated with the input signals calculated by the controller.

The physical parameters of the simulation, when not explicitly specified, are assumed as shown in Table 3.1:

Table 3.1 - Simulation physical parameters

| **Parameter**       | **Value**                 | **Parameter**     | **Value**         |
|---------------------|---------------------------|-------------------|-------------------|
| $$M$$               | $$2,4\ kg$$               | $$g$$             | $$9,81\ m/s^{2}$$ |
| $$m$$               | $$1,0\ kg$$               | $$c_{x},\ c_{y}$$ | $$0,2\ kg/s$$     |
| $$l$$               | $$1,0\ m$$                | $$c_{z}$$         | $$0,5\ kg/s$$     |
| $$I_{xx},\ I_{yy}$$ | $$0,055\ kg \cdot m^{2}$$ | $$c_{L}$$         | $$0,1\ kg/s$$     |
| $$I_{zz}$$          | $$0,1\ kg \cdot m^{2}$$   |                   |                   |

The values presented in Table 3.1 are approximately the specifications of the commercial drone *DJI Matrice 100*, which has a relatively high load capacity, with great potential for the proposed application (JEAONG et al., 2018).

The control parameters, when not explicitly specified, are defined as shown in Table 3.2:

Table 3.2 - Simulation control parameters.

| **Parameter**   | **Value** | **Parameter**      | **Value** |
|-----------------|-----------|--------------------|-----------|
| $$\lambda_{z}$$ | $$5$$     | $$\lambda_{\psi}$$ | $$2$$     |
| $$\kappa_{z}$$  | $$1$$     | $$\kappa_{\psi}$$  | $$1$$     |
| $$\eta_{z}$$    | $$2$$     | $$\eta_{\psi}$$    | $$2$$     |
| $$\lambda_{1}$$ | $$2$$     | $$\lambda_{5}$$    | $$- 2$$   |
| $$\lambda_{2}$$ | $$1$$     | $$\lambda_{6}$$    | $$- 1$$   |
| $$\lambda_{3}$$ | $$5$$     | $$\lambda_{7}$$    | $$5$$     |
| $$\lambda_{4}$$ | $$0,1$$   | $$\lambda_{8}$$    | $$0,1$$   |
| $$\kappa_{1}$$  | $$1$$     | $$\kappa_{2}$$     | $$1$$     |
| $$\eta_{1}$$    | $$2$$     | $$\eta_{2}$$       | $$2$$     |

It can be observed that the selected parameters respect the specified limits. In particular, it can be observed that $\frac{\lambda_{1}}{\lambda_{2}} = \frac{\lambda_{5}}{\lambda_{6}} = 2$ is smaller than $\frac{\lambda_{3}}{\lambda_{4}} = \frac{\lambda_{6}}{\lambda_{7}} = 10$ and that $\lambda_{5}$ and $\lambda_{6}$ are opposite to $\lambda_{1}$ and $\lambda_{2}$. It is also verified that the limit of stability for the reference angles of attitude ($\phi_{d}^{*}$ and $\theta_{d}^{*}$) according to equations (3.105) and (3.97) is approximately $1.4\ rad$ ($\approx 80{^\circ}$). The parameters for stabilizing the sliding variables $\kappa_{z}$, $\kappa_{\psi}$, $\kappa_{1}$, $\kappa_{2}$, $\eta_{z}$, $\eta_{\psi}$, $\eta_{1}$ and $\eta_{2}$ were designed to counteract the drag disturbance of the model and internal disturbances from the approximations made in the controller equations. In order to reduce the impact of the discontinuity in the control signal, the function $sign(s)$ is replaced with $\tanh(50s)$.

It is important to note that, despite one of the main characteristics of the sliding mode control technique being its robustness against disturbances with known limits, this aspect is not evaluated in detail, as this is not the focus of this work. This analysis is left for future work.

With this, the performance of the controller is evaluated for step input and for initial condition out of equilibrium, in order to evaluate the stability conditions derived in the previous section.

### System response for single step input

Figures Figure 5.6, Figure 5.7, and Figure 5.8 illustrate the dynamic behavior of the position, orientation of the aircraft, and cable orientation when a single step input is applied to the position and aircraft yaw angle with zero velocities and accelerations.

<img src="media/image12.emf" style="width:4.68504in;height:3.81197in" />

Figure 3.7 – Aircraft position and speed for single step entry.

<img src="media/image13.emf" style="width:4.68504in;height:3.72401in" />

Figure 3.8 - Aircraft orientation and angular velocity for zero-rate entry to $\phi$ and $\theta$, and a single step for $\psi$.

<img src="media/image14.emf" style="width:4.59167in;height:2.48958in" />

Figure 3.9 - Cable orientation for single-unit step entrance.

In Figure 3.7, it can be observed that the position of the aircraft tends to converge quickly to the desired state, presenting an error that is always decreasing with no apparent oscillation.

In Figure 3.8, it is observed that the roll angles ($\phi$) and pitch angles ($\theta$) exhibit significant oscillations, with more aggressive behavior in the transient regime (approximately up to 2 seconds after the input signal is applied), and then oscillate in a regular manner with a slight tendency towards the equilibrium point. On the other hand, the yaw angle ($\psi$) exhibits smooth behavior similar to that observed for the aircraft's position, which was expected, given the full actuation condition around the aircraft's vertical axis.

Already in Figure 3.9, it can be observed that the movement of the load exhibits oscillation from beginning to end, but despite the controller not acting explicitly to contain this oscillation, it shows a slight tendency towards convergence. Given the observation of the convergence of the drone's state variables, this behavior is expected due to the dynamic coupling that exists between the load's state and the drone's state, as well as the damping effect that air drag causes.

Finally, figures Figure 3.10 and Figure 3.10 present the behavior of the sliding variables and the control signals obtained in the analysis simulation.

<img src="media/image15.emf" style="width:3.56698in;height:2.67466in" />

Figure 3.10 – Behavior of sliding variables for step input.

<img src="media/image16.emf" style="width:4.3125in;height:4.59691in" />

Figure 3.11 - Control signals for CMD for single-step input.

Observing the Figure 3.10, it can be seen that, at the moment of applying the step input, which causes a deviation in the system state position relative to the equilibrium point, the sliding variables jump to values different from zero. However, they immediately begin to decrease, performing the approximation phase until they reach zero and enter the sliding phase. It is possible to observe the effect of the two stabilization portions of the sliding variables in the curves presented: the exponential decay combined with linear decay, ensuring rapid and finite convergence.

In Figure 3.11, it is observed that the torque follows a profile that is partially consistent with the behavior of the system variables. The thrust force $u_{1}\ $ and the torque $u_{4}$ exhibit smoother curves, while the torques related to the angles $\phi$ and $\theta$ are more oscillatory. However, it is noted that the signals $u_{1}$ and $u_{4}$ exhibit a negative jump after the moment when a unit reference deviation is provoked. This moment coincides with the moment of stabilization of the sliding variables. It is understood that this effect is related to the abrupt transition of the sliding variable to zero caused by the discontinuous terms of the input signal.

Finally, it is worth noting that the moment when the system enters the sliding phase coincides with the moment when the rolling and pitching angles stop oscillating significantly (between 2s and 4s). This means that the behavior observed from this point forward is mainly determined by the constants $\lambda_{1}$ to $\lambda_{8}$, which, for the defined values, led to the rapid stabilization of the position and a damped oscillatory behavior of the drone's attitude.

### Assessing the stability condition on the sliding surface

To illustrate the validity of the stability conditions for the control parameters of the subsystem, it is proposed to compare the system's behavior from a point near the equilibrium condition for three parameter settings: one stable, one unstable, and one on the verge of stability.

To do this, we take $\phi(0) = \theta(0) = 10{^\circ}$ (approximately 0.1745 radians) as the initial state and zero for all other system variables. By fixing the control parameters $\lambda_{2}$, $\lambda_{3}$, $\lambda_{4}$, $\lambda_{6}$ and $\ \lambda_{8}$ to 2 and $\lambda_{6} = - 2$, we generate $\lambda_{5} = - \lambda_{1}$ and vary $\lambda_{1}$ between 1, 2 and 4, in order to generate the scenarios where $\left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack > \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack$ (unstable), $\left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack = \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack$ (marginally stable) and $\left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack < \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack$ (stable) respectively. Figures Figure 3.12, Figure 3.13 and Figure 3.14 show the behavior of the most influential variables $x$,$y$, $\phi$,$\theta$, $\phi_{L}$ and $\theta_{L}$, which are directly related to the dynamics along the plan $xy$.

<img src="media/image17.emf" style="width:5.90551in;height:2.43045in" />

Figure 3.12 - System behavior on the $xy$ plane for $\lambda_{1} = 1$ $\left( \left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack > \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack \right)$.

<img src="media/image18.emf" style="width:5.90551in;height:2.43045in" />

Figure 3.13 - System behavior on the $xy$ plane for $\lambda_{1} = 2$ $\left( \left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack < \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack \right)$.

<img src="media/image19.emf" style="width:5.90551in;height:2.41502in" />

Figure 3.14 – System behavior on the $xy$ plane for $\lambda_{1} = 4$ $\left( \left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack < \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack \right)$.

As expected, in the first scenario (Figure 3.12), the variables diverge indefinitely. In the second configuration (Figure 3.13), the variables oscillate uniformly, without demonstrating a clear trend of convergence or divergence. Finally, when adjusting the variables within the stable conditions obtained (Figure 3.14), the variables exhibited clear convergence behavior.

4.  

# TRAJECTORY GENERATION

Although the lightweight controller developed takes into account the drone's dynamics coupled with the load, it only focuses on controlling the aircraft's position, ignoring the load's behavior. However, depending on the trajectory specified to the aircraft, the load can oscillate at high amplitudes and speeds, thereby degrading the overall performance of the movement.

To reduce this effect, it is decided to act on trajectory generation in order to induce the aircraft to move in a way that stabilizes the load. Specifically, the combination of two distinct techniques is explored: the first consists of determining reference trajectories for the drone based on position trajectories for the load and the aircraft's yaw angle, based on the system's differential planarity property. The second technique refers to *input shaping*, which filters the input signal based on the knowledge of the system's vibration dynamics to generate outputs with attenuated vibration.

## Trajectory Generation Based on the System's Differential Topography

Initially, the technique for trajectory generation developed by (SREENATH; MICHAEL; KUMAR, 2013) and (MELLINGER, 2012; MELLINGER; KUMAR, 2011) is presented. They demonstrate that the system is a differentially flat system, which allows determining the aircraft's position given the desired trajectories for the load and the aircraft's yaw angle.

Firstly, the definition of differential planarity of the system and how it is used to generate the system's trajectories is presented, with the definition of each variable of the system being detailed. It is emphasized that this section reproduces the central idea of trajectory generation from the reference works[^2], differing by adopting an alternative coordinate system, presenting more details in the development stages of the equations, and by adding the linear drag force to the model.

### Differential Planarity of the System

Given a system with state $x \in \mathbb{R}^{n}$ and input $u \in \mathbb{R}^{m}$, it is said to be differentially flat if there exists a finite set of variables $y \in \mathbb{R}^{m}$, called flat outputs, that are described in terms of the state, the input, and their derivatives up to a finite order $p$:

|                                                    |         |
|----------------------------------------------------|---------|
| $$y = y\left( x,u,\dot{u},\ldots,u^{(p)} \right)$$ | (4.109) |

in such a way that the state and system inputs can be written as continuous functions of these outputs and their derivatives up to a finite order $q$:

|                                                  |         |
|--------------------------------------------------|---------|
| $$x = x\left( y,\dot{y},\ldots,y^{(q)} \right)$$ | (4.110) |
| $$u = u\left( y,\dot{y},\ldots,y^{(q)} \right)$$ | (4.111) |

This property enhances trajectory planning, as it allows desired states to be determined from trajectories defined in the domain of flat trajectories (FLIESS et al., 1993).

In this context, it is possible to show that the state and the drone system inputs with suspended payload can be written as a function of the payload position and the aircraft's yaw angle and its derivatives up to a certain order. In other words, $\left\lbrack {\overrightarrow{x}}_{L},\psi \right\rbrack$ is a set of flat outputs for the system.

### Determining System Variables

#### Determining the Aircraft's Position $\overrightarrow{\mathbf{r}}$ and its Derivatives

Deriving Eq. (2.20) *n* times and isolating the corresponding term for the quadcopter's position, we get:

|                                                                                               |         |
|-----------------------------------------------------------------------------------------------|---------|
| $${\overrightarrow{r}}^{(n)} = {\overrightarrow{r}}_{L}^{(n)} - l{\overrightarrow{p}}^{(n)}$$ | (4.112) |

Therefore, to determine the nth derivative of $\overrightarrow{r}$, it is sufficient to have knowledge of the nth derivative of ${\overrightarrow{r}}_{L}$ and $\overrightarrow{p}$. The vector $\overrightarrow{p}$ can be determined as a function of the flat outputs from the dynamic load equation (Eq. 2.24)). By isolating the term related to the cable tension, one obtains:

|                                                                                                                                |         |
|--------------------------------------------------------------------------------------------------------------------------------|---------|
| $$\overrightarrow{T} = - m{\ddot{\overrightarrow{r}}}_{L} - mg{\overrightarrow{e}}_{z} - C_{L}{\dot{\overrightarrow{r}}}_{L}$$ | (4.113) |

Through the definition of $\overrightarrow{p}$, one obtains:

|                                                                                         |         |
|-----------------------------------------------------------------------------------------|---------|
| $$\overrightarrow{p} = \frac{\overrightarrow{T}}{\left\| \overrightarrow{T} \right\|}$$ | (4.114) |
| $$\left\| \overrightarrow{T} \right\| = \overrightarrow{T} \cdot \overrightarrow{p}$$   | (4.115) |

Note that the derivative of Eq. (4.115) does not depend on $\dot{\overrightarrow{p}}$:

|                                                                                                   |         |
|---------------------------------------------------------------------------------------------------|---------|
| $$\dot{\left\| \overrightarrow{T} \right\|} = \dot{\overrightarrow{T}} \cdot \overrightarrow{p}$$ | (4.116) |

Thus, it is possible to determine $\dot{\overrightarrow{p}}$ in Eq. (4.114), since it is also known that $\dot{\overrightarrow{T}}$ is known by Eq. (4.113) and the derivatives of the flat output ${\overrightarrow{r}}_{L}$. Repeating this process, it is possible to conclude that ${\overrightarrow{p}}^{(n)}$ can be described in terms of ${\overrightarrow{r}}_{L}^{(n + 2)}$ and its lower derivatives up to ${\dot{\overrightarrow{r}}}_{L}$. Given that it is desired to define up to $\ddot{\overrightarrow{r}}$, it is verified that Eq. (4.112) requires calculating $\overrightarrow{p}$ up to its second derivative, which in turn requires the knowledge of ${\overrightarrow{r}}_{L}$ up to its fourth derivative.

#### Determining the Orientation $\overrightarrow{\mathbf{\eta}}$ and Thrust Force $\mathbf{F}_{\mathbf{b}}^{\mathbf{z}}$

The equation of aircraft dynamics (Eq. 2.23) can be rearranged as follows:

|                                                                                                                                                                                  |         |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$M\ddot{\overrightarrow{r}} - T\overrightarrow{p} + Mg{\overrightarrow{e}}_{z} + C\dot{\overrightarrow{r}} = F_{z}^{b}{\overrightarrow{\mathbf{e}}}_{\mathbf{z}}^{\mathbf{b}}$$ | (4.117) |

It is verified that:

|                                                                                                                                                                                                      |         |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$\mathbf{R}{\overrightarrow{e}}_{z} = {\overrightarrow{e}}_{z}^{b} = \frac{\overrightarrow{t}}{\left\| \overrightarrow{t} \right\|},\ \ \ \ onde\ \ \ \overrightarrow{t} = \left\{ \begin{array}{r} 
 \ddot{x} + c_{x}\dot{x} - \frac{T_{x}}{M} \\                                                                                                                                                          
 \ddot{y} + c_{y}\dot{y} - \frac{T_{y}}{M} \\                                                                                                                                                          
 \ddot{z} + g + c_{z}\dot{z} - \frac{T_{z}}{M}                                                                                                                                                         
 \end{array} \right\}$$                                                                                                                                                                                | (4.118) |

Thus, an auxiliary coordinate system $\Sigma_{c} = \left\lbrack {\overrightarrow{e}}_{x}^{c},{\overrightarrow{e}}_{y}^{c},{\overrightarrow{e}}_{z}^{c} \right\rbrack$ is defined, which corresponds to the rotating inertial coordinate system of $\psi$ around ${\overrightarrow{e}}_{z}$, as shown in the Figure 4.1.

<img src="media/image20.emf" style="width:5.30535in;height:2.28302in" />

Figure 4.1 - Illustration of the auxiliary coordinate system $\Sigma_{c}$.

Analyzing Figure 4.1, it is possible to verify that:

|                                                                                         |         |
|-----------------------------------------------------------------------------------------|---------|
| $${\overrightarrow{e}}_{y}^{c} = \left\lbrack - \sin\psi,\cos\psi,0 \right\rbrack^{T}$$ | (4.119) |

From ${\overrightarrow{e}}_{y}^{c}$, it is possible to determine the other unit vectors that make up the body's coordinate system, such as:

|                                                                                                                                                                                               |         |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $${\overrightarrow{e}}_{x}^{b} = \frac{{\overrightarrow{e}}_{y}^{c} \times {\overrightarrow{e}}_{z}^{b}}{\left\| {\overrightarrow{e}}_{y}^{c} \times {\overrightarrow{e}}_{z}^{b} \right\|}$$ | (4.120) |
| $${\overrightarrow{e}}_{y}^{b} = {\overrightarrow{e}}_{z}^{b} \times {\overrightarrow{e}}_{x}^{b}$$                                                                                           | (4.121) |

Thus, we have the rotation matrix given by:

|                                                                                                                                    |         |
|------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$\mathbf{R} = \left\lbrack {\overrightarrow{e}}_{x}^{b},{\overrightarrow{e}}_{y}^{b},{\overrightarrow{e}}_{z}^{b} \right\rbrack$$ | (4.122) |

From the rotation matrix, it is possible to determine the Euler angles (SLABAUGH, 1999). In short, $F_{z}^{b}$ is defined by replacing $\mathbf{R}{\overrightarrow{e}}_{z}$ in Eq. (4.117).

#### Determining Angular Velocity $\overrightarrow{\mathbf{\omega}}$

Deriving the aircraft's motion equation (Eq. (4.117)), we obtain:

|                                                                                                                                                                                                                                 |         |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$M\ \dddot{\overrightarrow{r}} + C\ddot{\overrightarrow{x}} - \dot{\overrightarrow{T}} = {\dot{F}}_{z}^{b}{\overrightarrow{e}}_{z}^{b} + F_{z}^{b}\left( \overrightarrow{\omega} \times {\overrightarrow{e}}_{z}^{b} \right)$$ | (4.123) |

Designing this expression over ${\overrightarrow{e}}_{z}^{b}$, one obtains that:

|                                                                                                                                                             |         |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$\dot{F_{z}^{b}} = \left( M\dddot{\overrightarrow{r}} + C\ddot{\overrightarrow{x}} - \dot{\overrightarrow{T}} \right) \cdot {\overrightarrow{e}}_{z}^{b}$$ | (4.124) |

In the direction perpendicular to ${\overrightarrow{e}}_{z}^{b}$ and $\overrightarrow{\omega}$, along which there is:

|                                                                                                                                                                                                                                                                                                    |         |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $${\overrightarrow{h}}_{\omega} = \overrightarrow{\omega} \times {\overrightarrow{e}}_{z}^{b} = \frac{1}{F_{z}^{b}}\left\{ M\dddot{\overrightarrow{r}} + C\ddot{\overrightarrow{r}} - \dot{\overrightarrow{T}} - \dot{\mathbf{F}_{\mathbf{z}}^{\mathbf{b}}}{\overrightarrow{e}}_{z}^{b} \right\}$$ | (4.125) |

It can be observed that ${\overrightarrow{h}}_{\omega}$ is the projection of $\overrightarrow{\omega}$ onto the plane $x_{b}y_{b}$, rotated by 90°, so it is possible to determine the components of the angular velocity in this plane as$:$

|                                                                            |         |
|----------------------------------------------------------------------------|---------|
| $$p = - {\overrightarrow{h}}_{\omega} \cdot {\overrightarrow{e}}_{y}^{b}$$ | (4.126) |
| $$q = {\overrightarrow{h}}_{\omega} \cdot {\overrightarrow{e}}_{x}^{b}$$   | (4.127) |

Finally, the known $p$, $q$, and $\dot{\psi}$, the third component of the vector $\overrightarrow{\omega}$, is obtained from the third component of Eq. (2.2):

|                                                              |         |
|--------------------------------------------------------------|---------|
| $$r = \frac{\cos\theta\dot{\psi} - \sin\phi q}{\cos\phi}\ $$ | (4.128) |

#### Determining Angular Acceleration $\dot{\overrightarrow{\mathbf{\omega}}}$ and Moment of Impact ${\overrightarrow{\mathbf{\tau}}}_{\mathbf{b}}$

To determine $\dot{\overrightarrow{\omega}}$, a procedure similar to the one used to find $\overrightarrow{\omega}$ is applied. First, the dynamic equation is derived again:

|                                                                                                                                                                                                                                                                                                                                                                                                                                              |         |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $M\ \ddddot{\overrightarrow{r}} + C\dddot{\overrightarrow{r}} - \ddot{\overrightarrow{T}} = {\ddot{F}}_{z}^{b}{\overrightarrow{e}}_{z}^{b} + 2\left( \overrightarrow{\omega} \times {\dot{F}}_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right) + \dot{\overrightarrow{\omega}} \times F_{z}^{b}{\overrightarrow{e}}_{z}^{b} + \overrightarrow{\omega} \times \left( \overrightarrow{\omega} \times F_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right)$ | (4.129) |

Designing this expression over ${\overrightarrow{e}}_{z}^{b}$, we find that:

|                                                                                                                                                                                                                                                                                                                       |         |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $${\ddot{F}}_{z}^{b} = \left\lbrack \left( M\ \ddddot{\overrightarrow{r}} + C\dddot{\overrightarrow{r}}\  - \ddot{\overrightarrow{T}} \right) - \overrightarrow{\omega} \times \left( \overrightarrow{\omega} \times F_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right) \right\rbrack \cdot {\overrightarrow{e}}_{z}^{b}$$ | (4.130) |

In the direction perpendicular to ${\overrightarrow{e}}_{z}^{b}$ and $\dot{\overrightarrow{\omega}}$, along which there is:

|                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |         |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| $${\overrightarrow{h}}_{\alpha} = \dot{\overrightarrow{\omega}} \times {\overrightarrow{e}}_{z}^{b} = \frac{1}{F_{z}^{b}}\left\{ M\ \ddddot{\overrightarrow{r}} + C\dddot{\overrightarrow{r}}\  - {\ddot{\mathbf{F}}}_{\mathbf{z}}^{\mathbf{b}}{\overrightarrow{e}}_{z}^{b} - 2\left( \overrightarrow{\omega} \times {\dot{F}}_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right) - \overrightarrow{\omega} \times \left( \overrightarrow{\omega} \times F_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right) \right\}$$ | (4.131) |

It can be observed that ${\overrightarrow{h}}_{\alpha}$ corresponds to the projection of $\dot{\overrightarrow{\omega}}$ on the plane $x_{b}y_{b}$ rotated by 90°, such that it is possible to conclude that:

|                                                                                  |         |
|----------------------------------------------------------------------------------|---------|
| $$\dot{p} = - {\overrightarrow{h}}_{\alpha} \cdot {\overrightarrow{e}}_{y}^{b}$$ | (4.132) |
| $$\dot{q} = {\overrightarrow{h}}_{\alpha} \cdot {\overrightarrow{e}}_{x}^{b}$$   | (4.133) |

Finally, the known $\dot{p}$, $\dot{q}$, and $\ddot{\psi}$, the third component of the vector $\dot{\overrightarrow{\omega}}$, is determined from the third component of the derivative of Eq. (2.2):

|                                                                                                                                       |         |
|---------------------------------------------------------------------------------------------------------------------------------------|---------|
| $$\dot{r} = \frac{\cos\theta\ddot{\psi} - \sin\phi\dot{q} - \dot{\theta}\left( \dot{\phi} + \sin\theta\dot{\psi} \right)}{\cos\phi}$$ | (4.134) |

It should be noted that to arrive at the definition of $\dot{\overrightarrow{\omega}}$ and ${\overrightarrow{\tau}}_{b}$, it was necessary to have knowledge of $\ \ddddot{\overrightarrow{r}}$, which in turn requires knowledge up to ${\overrightarrow{r}}_{L}^{(6)}$ (see section 4.1.2.1). In other words, to determine all the system variables and the angular accelerations and torques, it is necessary to have knowledge up to the sixth derivative of the position of the load.

### Defining Load Trajectories

Given the differential flatness of the system, with the aim of reducing the load imbalance, the goal is to design a stable trajectory for the load so that the aircraft's state can be obtained and passed as a reference to the controller. If the controller is able to successfully follow the reference, it is expected that the load will follow its programmed trajectory.

Thus, polynomial trajectories of 13th order are designed that pass through arbitrary points where the derivatives up to the sixth order are continuous (constraints derived in the previous section), being zero at the points of interest (APPENDIX II – PARTIAL POLYNOMIAL INTERPOLATION). Figure 4.2 presents the position, velocity, and acceleration of an example trajectory with these constraints for the passing points $q_{d} = \lbrack 3,0,2\rbrack$ and times $t_{d} = \lbrack 0,5,10\rbrack\ s$.

<img src="media/image21.emf" style="width:3.65625in;height:3.06322in" />

Figure 4.2 - Example of a point-to-point 13th-degree polynomial trajectory with zero derivatives up to the sixth order for the waypoints $q_{d} = \lbrack 3,0,2\rbrack$ at the times $t_{d} = \lbrack 0,5,10\rbrack\ s$.

Observing that this type of interpolation generates smooth curves with well-defined stops at the points of interest. Defining trajectories in this way in three-dimensional space results in straight lines between the points.

The differential flatness property allows one to determine the necessary state of the *drone* so that the load has the desired behavior, but there is no guarantee that the resulting trajectories are compatible with the model's constraints, such as the tension of the cable always being positive and the angles of yaw and pitch being less than 90°, nor that they are feasible to be executed by the controller.

In fact, the polynomial trajectory generation structure defined does not reflect the characteristics of the system in its formulation, and may produce outputs that are dissonant with the system's dynamics.

One natural way to solve this problem would be to treat the definition of the load state as an optimization problem that considers convenient constraints for the *drone*. However, this process does not appear to be trivial (CRUZ; FIERRO, 2017; SREENATH; LEE; KUMAR, 2013).

Alternatively, it proposes applying the *input shaping* technique, which stands out for its simplicity and effectiveness in reducing oscillation in the system's output by embedding oscillations in the input signal that take into account the system's natural frequency.

## Input Shaping

### Theoretical Foundation

The technique of *input shaping* was inspired by the work of experienced crane operators who are able to maneuver loads without causing excessive swings by simply pressing the activation button repeatedly at specific moments (SINGH; SINGHOSE, 2002). The principle of this method consists of introducing one or more impulsive signals into the system input to generate a vibration in the output, which is subsequently compensated for by introducing other impulsive elements that would cause an opposing vibration (QIAN; YI, 2015). The Figure 4.3 illustrates the idea for two impulsive signals.

<img src="media/image22.emf" style="width:2.98958in;height:2.35194in" />

Figure 4.3 - Illustration of a system's response under the action of two properly selected impulses using the *input shaping* technique (Adapted from SINGH; SINGHOSE, 2002).

In Figure 4.3, the blue curve represents the system's response when only impulse *A<sub>1</sub>* is applied. The red curve represents the system's response when only impulse *A<sub>2</sub>* is applied. Both impulses generate an oscillation with the same frequency and decay rate, which is expected for second-order systems with damping. *A<sub>2</sub>* is precisely scaled at the moment of inversion of the motion caused by $A_{1}$, producing an opposite response. This configuration, in turn, generates a trajectory without oscillation, as illustrated by the black line.

To obtain the two-impulse sequence as illustrated in Figure 4.3, one starts from the description of the residual vibration of a system with natural frequency $\omega_{n}$ and damping factor $\zeta$, given by:

|                                                                                                                   |         |
|-------------------------------------------------------------------------------------------------------------------|---------|
| $$V\left( \omega_{n},\zeta \right) = e^{- \zeta\omega\_ nt_{N}}\sqrt{C(\omega,\zeta)^{2} + S(\omega,\zeta)^{2}}$$ | (4.135) |

In what:

|                                                                                                                          |         |
|--------------------------------------------------------------------------------------------------------------------------|---------|
| $$C\left( \omega_{n},\zeta \right) = \sum_{i = 1}^{I}{A_{i}e^{\zeta\omega_{n}t_{i}}\cos\left( \omega_{d}t_{i} \right)}$$ | (4.136) |
| $$S(\omega,\zeta) = \sum_{i = 1}^{N}{A_{i}e^{\zeta\omega_{n}t_{i}}\sin\left( \omega_{d}t_{i} \right)}$$                  | (4.137) |

$A_{i}$ and $t_{i}$ are the amplitudes and the times at which the pulses occur, and $\omega_{d} = \omega_{n}\sqrt{1 - \zeta^{2}}$ is the natural damped frequency. Setting the percentage of residual vibration to zero, imposing the constraint $\sum A_{i} = 1$ and taking the position of the first pulse as $t_{1} = 0$, it is possible to determine the amplitudes of the pulses $(A_{1}$ and $A_{2}$) and the time at which the second pulse occurs: $t_{2}$ (BISGAARD; COUR-HARBO; BENDTSEN, 2008).

The filter with two pulses presented is called a *zero vibration shaper* (or *ZV shaper*). However, in this work, we opt to apply a three-pulse shaper called a *ZVD shaper* (*Zero Vibration and Derivative shaper*) in which the zero derivative constraint is added to the residual vibration:

|                                                                                                                        |         |
|------------------------------------------------------------------------------------------------------------------------|---------|
| $$\frac{d}{d\omega_{n}}V\left( \omega_{n},\zeta \right) = 0,\ \ \frac{d}{d\zeta}V\left( \omega_{n},\zeta \right) = 0$$ | (4.138) |

Thus, the amplitudes and times of the pulses are given by:

|                                                                                                                     |         |
|---------------------------------------------------------------------------------------------------------------------|---------|
| $$t_{1} = 0,\ \ t_{2} = \frac{T_{d}}{2},\ \ t_{3} = \frac{T_{d}}{3}$$                                               | (4.139) |
| $$A_{1} = \frac{1}{1 + 2K + K^{2}},\ \ A_{2} = \frac{2K}{1 + 2K + K^{2}},\ \ A_{3} = \frac{K^{2}}{1 + 2K + K^{2}}$$ | (4.140) |

Being that $T_{d}$ is the amortized period and $K$ is given by:

|                                                                    |         |
|--------------------------------------------------------------------|---------|
| $$K = \exp\left( - \frac{\zeta\pi}{\sqrt{1 - \zeta^{2}}} \right)$$ | (4.141) |

In comparison to the ZV *shaper*, the ZVD presents significantly greater robustness with respect to uncertainties in the natural frequency of the system. On the other hand, it introduces a larger delay in the system's output, which is the trend as larger-order models are considered (SINGH; SINGHOSE, 2002).

### Input shaping applied to the problem

Taking this technique to the proposed problem, first, the natural frequency of the system must be determined. To do this, the natural frequency of the load balance is obtained, obtained from the linearized dynamic equations of $\phi_{L}$ and $\theta_{L}$ around the equilibrium point, in which the following is obtained:

|                                               |         |
|-----------------------------------------------|---------|
| $$\omega_{n} = \sqrt{\frac{(M + m)g}{Ml}\ }$$ | (4.142) |

Given that the slider-type controller compensates for drag as a disturbance component, the damping coefficient $\zeta$ is considered to be zero. Thus, considering the standard physical parameters defined in Section 3.3, the values presented in Table 4.1 are obtained:

Table 4.1 – *Input shaping* parameters for the example system.

| **Parameter** | **Value**         | **Parameter** | **Value**         |
|----------------|-------------------|----------------|-------------------|
| $$\omega_{n}$$ | $$3,7279\ rad/s$$ | $$\omega_{d}$$ | $$3,7279\ rad/s$$ |
| $$\zeta$$      | $$0$$             | $$T_{d}$$      | $$1,6856\ s$$     |
| $$A_{1}$$      | $$0,25$$          | $$t_{1}$$      | $$0$$             |
| $$A_{2}$$      | $$0,5$$           | $$t_{2}$$      | $$0,8428\ s$$     |
| $$A_{3}$$      | $$0,25$$          | $$t_{3}$$      | $$1,6856\ s$$     |

Thus, to transfer this behavior to arbitrary input signals, it is sufficient to perform the convolution of the designed impulse sequence on the input signal of the system. For the problem at hand, it is initially proposed to define points and angles of steering by which the drone should pass at certain time instants, perform a polynomial interpolation by restricting velocities and accelerations to zero at these points, and then apply *input shaping* to the resulting curves and pass them as references to the controller. In this case, the references for the drone's orientation are taken as zero. This strategy has already been successfully used in other works on controlling VANTs with suspended loads by cable, such as (BISGAARD; COUR-HARBO; BENDTSEN, 2008; KLAUSEN; FOSSEN; JOHANSEN, 2017)

Additionally, it is also proposed to combine this technique with the one based on the differential flatness of the system, as will be presented in the next section.

## Trajectories Based on the Differential Flatness of the System with *Input Shaping*

A Figure 4.4 illustrates the new generation trajectory strategy proposed.

<img src="media/image23.emf" style="width:6.47309in;height:1.44136in" />

Figure 4.4 - Final solution trajectory generation structure.

As illustrated in Figure 4.4, the final trajectory generation strategy receives the desired stopping points for the load position and for the drone's yaw angle as input, and then performs a polynomial interpolation, restricting all derivatives up to the sixth order for the load position and up to the second order for the yaw angle to be zero at these points. Thus, *input shaping* is applied to the resulting curves from the interpolation, obtaining the flat outputs that are used to calculate the desired state for the drone, which is passed as a reference to the controller (Figure 3.4, Section 3.2).

To exemplify the effect of *input shaping* on the output of the trajectory generator, a polynomial trajectory is defined for the load and the drone's yaw angle, starting from point ${{\overrightarrow{r}}_{L}}_{i} = \lbrack 0,0, - l\rbrack$ and $\psi_{i} = 0$ to ${{\overrightarrow{r}}_{L}}_{f} = \lbrack 2,2,1\rbrack$ $\psi_{f} = \frac{\pi}{3}$ in 4 seconds, to which *input shaping* is applied with the parameters presented in Table 4.1, in Section 4.2.2. First, Figure 4.5 presents the trajectory defined for each component of the load's position, as well as the resulting drone's position, for the cases where *input shaping* is applied and when it is not applied at the input.

<img src="media/image24.emf" style="width:4.64501in;height:3.89307in" />

Figure 4.5 - Comparison between a polynomial trajectory defined point-by-point for the position of the load and the one obtained for the drone according to the differentially flat model, for the configurations in which *input shaping* is applied and does not apply to the trajectory.

In Figure 4.5, when comparing the trajectory defined for the load with the one obtained for the drone, it is observed that the latter exhibits a slight braking approximately in the middle of the ascent time. In relation to the effect of *input shaping*, it is observed that the filter causes a certain curvature of the curves, anticipating the acceleration and deceleration of the movement at the beginning and end of the trajectory. It is also possible to verify, mainly in the height profile $z$, that *input shaping* promotes a slight attenuation of the transient effect. These effects can also be observed in three-dimensional space, as shown in Figure 4.6.

<img src="media/image25.png" style="width:3.82639in;height:3.26209in" />

Figure 4.6 – Spatial visualization of the load and drone trajectory for the configurations in which *input shaping* is applied and not applied to the input trajectory.

A Figure 4.6 shows that, without applying *input shaping*, the drone executes a maneuver with a larger amplitude, presenting a sharp oscillation in the middle of the trajectory. When the filter is applied, this curve approaches a straight line more closely, but it is possible to observe small oscillations along the entire trajectory.

Finally, it is important to observe the behavior of other system variables. Figures Figure 4.7, Figure 4.8 and Figure 4.9 show the profile of the variables that describe the drone's orientation, the input forces, and the cable's orientation angles, respectively.

In general, it is noted that *input shaping* promotes a significant reduction in the amplitude of the system variables, with the exception of the aircraft's yaw angle, which follows a behavior similar to that observed for the drone's position, as it is a predefined flat output as a polynomial curve. For example, when observing the first graph in Figure 4.8, it is observed that the thrust force $u_{1}$ reaches values of up to approximately 45 N when *input shaping* is not applied, but then assumes values below 40 N when the filter is applied. On the other hand, the emergence of additional oscillations, but of little intensity, in the behavior of the cable's orientation angles, as shown in Figure 4.9, was verified.

<img src="media/image27.emf" style="width:3.10586in;height:2.79248in" />

Figure 4.7 – State variables that define the drone's orientation, obtained through the differentially flat model, when *input shaping* is applied in a point-to-point polynomial trajectory defined for the load position, and when it is not.

<img src="media/image28.emf" style="width:3.54331in;height:3.88049in" />

Figure 4.8 - Control signals obtained using the differentially flat model for when and when not to apply *input shaping* in a point-to-point polynomial trajectory defined for the load position.

<img src="media/image29.emf" style="width:3.24306in;height:2.56271in" />

Figure 4.9 - Variables that describe the cable orientation when *input shaping* is applied and not applied in a point-to-point polynomial trajectory defined for the load position.

5.  

# TRAJECTORY GENERATING CONTROLLER

This chapter presents an analysis of the performance of the combined controller, along with the trajectory generation techniques addressed in the implementation of the proposed objective.

## Analysis Structure

To evaluate the performance of the final control solution, a comparative analysis is proposed between three different configurations:

1) Polynomial trajectories for the drone;

2) Polynomial trajectories for drone *input shaping*;

3) Polynomial trajectories for *input shaping* model, differentially flat.

Define a common point-to-point trajectory for the settings, considering the load reference points (for configuration III) below the drone reference positions (configurations I and II), corresponding to the cable length. Table 5.1 presents the list of trajectory points to be passed as input to the polynomial interpolators for each configuration, while Figure 5.1 shows a graphical representation of the path for the load and for the drone.

Table 5.1 – Reference points for the polynomial interpolator for comparison across different trajectory generation configurations.

| **Configuration** | **Variable**    | **Value**                                                                            |
|------------------|-----------------|--------------------------------------------------------------------------------------|
| I, II and III      | $$x,x_{L}$$     | $$\lbrack 0,4,4,1,0\rbrack\ m$$                                                      |
| I, II and III      | $$y,\ y_{L}$$   | $$\lbrack 0,6,6,9,0\rbrack\ m$$                                                      |
| I and II           | $$z$$           | $$\lbrack 0,5,5,2,0\rbrack\ m$$                                                      |
| III              | $$z_{L}$$       | $$\lbrack - 1,\ 4,\ 4,\ 1, - 1\rbrack\ m$$                                           |
| I, II and III      | $$\psi$$        | $$\left\lbrack 0,\frac{\pi}{3},\frac{\pi}{3}, - \frac{\pi}{4},0 \right\rbrack\ rad$$ |
| I and II           | $$\phi,\theta$$ | $$\lbrack 0,0,0,0,0\rbrack\ rad$$                                                    |

<img src="media/image30.png" style="width:5.125in;height:3.87248in" />

Figure 5.1 – Reference test route for the drone (settings I and II) and for the payload (configuration III)

It is assumed that these points occur at equally spaced times with an additional wait time of two seconds at the beginning and an additional time of five seconds for accommodation. Therefore, the system simulation is performed for each configuration, with the total maneuver time (excluding the wait and accommodation times) varying to verify the system's performance for different levels of maneuver aggressiveness. Specifically, the maneuver is considered to be executed in $T = \lbrack 18,\ 15,\ 12,\ 10\rbrack\ s$. It is noted that the same simulation parameters specified in Tables Table 3.1 and Table 3.2 in Section 3.3 are adopted.

To compare the results, a set of metrics are defined that are calculated on the simulation output variables sampled at intervals of 0.01 s. The Table 5.2 lists the metrics used for the comparison.

Table 5.2 – List of metrics for comparing the behavior of the controlled system under different trajectory generation settings.

| **Symbol**                 | **Metric**                                                                                                                           |
|-----------------------------|---------------------------------------------------------------------------------------------------------------------------------------|
| $$r_{RMS}$$                 | Effective value of the drone's position error.                                                                                             |
| $$\beta_{RMS}$$             | Effective value of the angle between the inertial (${\overrightarrow{e}}_{z}$) and non-inertial (${\overrightarrow{e}}_{z}^{b}$) axes. |
| $$\alpha_{RMS}$$            | Effective value of the cable angle relative to the vertical during the settling time.                                                   |
| $${\overline{f}}_{\omega}$$ | Average frequency of the angular frequency module $\overrightarrow{\omega}$

The first measure ($r_{RMS}$) aims to quantify the average error of the aircraft's position control. The value $\beta_{RMS}$ seeks to access the average tilt angle of the *drone* along the flight path. When $\beta_{RMS}$ is greater, it means that the *drone* exhibited larger and longer angles of roll and yaw, compared to that. The measure $\alpha_{RMS}$ aims to quantify the degree of oscillation of the load after executing the planned flight path, ideally equaling zero. Finally, ${\overline{f}}_{\omega}$ captures the degree of oscillation of the *drone's* attitude in terms of frequency.

It should be noted that all defined metrics are better when they are smaller. Their absolute values do not have much meaning. The analysis is more valid when evaluated in a comparative way and in conjunction with graphical analyses.

## Results Analysis

A Table 5.3 presents the results of the calculations of the analysis parameters for each simulation scenario.

Table 5.3 – Performance comparison table based on metrics between trajectory generation configurations being analyzed for different trajectory reference execution times.

```
<table style="width:100%;">
<colgroup>
<col style="width: 12%" />
<col style="width: 10%" />
<col style="width: 10%" />
<col style="width: 10%" />
<col style="width: 10%" />
<col style="width: 10%" />
<col style="width: 10%" />
<col style="width: 10%" />
<col style="width: 10%" />
</colgroup>
<thead>
<tr class="header">
<th><span class="math inline"><strong>T</strong></span></th>
<th colspan="4"><strong>18 s</strong></th>
<th colspan="4"><strong>15 s</strong></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td><strong>Metric</strong></td>
<td><p><span class="math display"><strong>r</strong><sub><strong>RMS</strong><strong>S</strong></sub></span></p>
<p>[m]</p></td>
<td><p><span class="math display"><strong>β</strong><sub><strong>RMS</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display"><strong>α</strong><sub><strong>RMS</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display">$${\overline{\mathbf{f}}}_{\mathbf{\omega}}$$</span></p>
<p>[Hz]</p></td>
<td><p><span class="math display"><strong>r</strong><sub><strong>RMS</strong><strong>S</strong></sub></span></p>
<p>[m]</p></td>
<td><p><span class="math display"><strong>β</strong><sub><strong>RMS</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display"><strong>α</strong><sub><strong>RMS</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display">$${\overline{\mathbf{f}}}_{\mathbf{\omega}}$$</span></p>
<p>[Hz]</p></td>
</tr>
<tr class="even">
<td>I</td>
<td>0.0235</td>
<td>0.1238</td>
<td>0.1920</td>
<td>0.1218</td>
<td>0.0290</td>
<td>0.1698</td>
<td>0.0765</td>
<td>0.1920</td>
</tr>
<tr class="odd">
<td>II</td>
<td>0.0203</td>
<td>0.0892</td>
<td>0.0186</td>
<td>0.0829</td>
<td>0.0238</td>
<td>0.1155</td>
<td>0.0230</td>
<td>0.1184</td>
</tr>
<tr class="even">
<td>III</td>
<td>0.0021</td>
<td>0.1029</td>
<td>0.0061</td>
<td>0.0653</td>
<td>0.0022</td>
<td>0.1208</td>
<td>0.0158</td>
<td>0.0788</td>
</tr>
<tr class="odd">
<td><span class="math display"><strong>T</strong></span></td>
<td colspan="4"><strong>12 s</strong></th>
<td colspan="4"><strong>10 s</strong></th>
</tr>
<tr class="even">
<td><strong>Metric</strong></td>
<td><p><span class="math display"><strong>r</strong><sub><strong>RMS</strong><strong>S</strong></sub></span></p>
<p>[m]</p></td>
<td><p><span class="math display"><strong>β</strong><sub><strong>RMS</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display"><strong>α</strong><sub><strong>RMS</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display">$${\overline{\mathbf{f}}}_{\mathbf{\omega}}$$</span></p>
<p>[Hz]</p></td>
<td><p><span class="math display"><strong>r</strong><sub><strong>RMS</strong><strong>S</strong></sub></span></p>
<p>[m]</p></td>
<td><p><span class="math display"><strong>β</strong><sub><strong>RMS</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display"><strong>α</strong><sub><strong>RMS</strong><strong>S</strong></sub></span></p>
<p>[rad]</p></td>
<td><p><span class="math display">$${\overline{\mathbf{f}}}_{\mathbf{\omega}}$$</span></p>
<p>[Hz]</p></td>
</tr>
<tr class="odd">
<td>I</td>
<td>0.0404</td>
<td>0.3063</td>
<td>0.9676</td>
<td>0.3913</td>
<td>0.0514</td>
<td>0.4251</td>
<td>1.1075</td>
<td>0.3937</td>
</tr>
<tr class="even">
<td>II</td>
<td>0.0280</td>
<td>0.1525</td>
<td>0.0708</td>
<td>0.2058</td>
<td>0.0312</td>
<td>0.1860</td>
<td>0.1758</td>
<td>0.3479</td>
</tr>
<tr class="odd">
<td>III</td>
<td>0.0024</td>
<td>0.1490</td>
<td>0.0437</td>
<td>0.1916</td>
<td>0.0029</td>
<td>0.2480</td>
<td>0.0848</td>
<td>1.4683</td>
</tr>
</tbody>
</table>

Analyzing the results of Table 5.3 globally, it is observed that configuration I, which does not have any explicit strategy for reducing the load imbalance, is the one that presents the highest position error ($\alpha_{RMS}$) and load oscillation intensity in all simulations. It is also observed that configuration III, which takes into account the differentially flat model, presents the lowest position error and the degree of load oscillation in all simulations. Despite the numerical differences in the position error ($r_{RMS}$), it is observed that it is low for all scenarios (no more than 5 cm), demonstrating the efficiency of the controller in achieving its objective.

To aid in further analysis, Figures Figure 5.2, Figure 5.3, and Figure 5.4 show the path taken by the *drone* and the payload for the simulation results with paths of 15 and 10 seconds for the three configurations.

<img src="media/image32.png" style="width:3.05118in;height:2.91544in" /><img src="media/image34.png" style="width:3.05118in;height:2.91481in" />

Figure 5.2 - Path followed by the drone and the load for the 10 and 15 second simulation with configuration I.

<img src="media/image36.png" style="width:3.05069in;height:3in" /><img src="media/image38.png" style="width:3.05118in;height:3.00821in" />

Figure 5.3 - Path followed by the drone and the load for the 10 and 15-second simulation with configuration II.

<img src="media/image36.png" style="width:3.05118in;height:2.9749in" /><img src="media/image40.png" style="width:3.05118in;height:2.97151in" />

Figure 5.4 - Path traveled by the drone and the payload for the 10 and 15 second simulation with configuration III.

When analyzing the Figure 5.2, in which a specialized trajectory generation strategy (configuration I) is not applied, it reinforces the conclusion that, although the controller is capable of maintaining the drone's position stably, the load is very heavy, especially for the aggressive 10-second maneuver.

Although the controller with *input shaping* (Figure 5.3) did not show any $\alpha_{RMS}$ in any of the simulations, it presented convincing practical performance in this aspect. When compared to the configuration in which *input shaping* is not applied (Figure 5.2), it is observed that the technique was effective in promoting load balancing, and only slight deviations were observed at the end points of the 10-second simulation. It is noted that the vertical deviation of the load position relative to the reference observed between the passing points is inherent in the load movement, and the dip in the drone's path at the second passing point is a consequence of applying *input shaping*.

When *input shaping* is applied to the load trajectory (Figure 5.4), it is not possible to observe the presence of oscillations at the extreme points, nor even in a 10-second simulation. A droop is observed in the load trajectory as it passes through the second point of the trajectory in the 10-second simulation, but this occurs due to the application of *input shaping*. It is also possible to observe the trajectory deviation that the *drone* makes with respect to the straight lines that connect the passing points, especially for the most aggressive maneuver.

The load attenuation behavior becomes more apparent when analyzing graphically the state of the cable orientation. Figure 5.5 presents the behavior of the variables that describe the cable orientation for a trajectory simulation of 12 seconds with configurations I, II, and III.

<img src="media/image42.emf" style="width:2.23958in;height:3.54236in" /><img src="media/image43.emf" style="width:2.09375in;height:3.54167in" /><img src="media/image44.emf" style="width:2.13542in;height:3.54167in" />

Figure 5.5 - State of the cable orientation for a 12-second trajectory simulation with settings I (without *input shaping*), II (with *input shaping*) and III (with *input shaping* and differentially flat model)

Observing that, after the dashed vertical line, starting from where the measurement $\alpha_{RMS}$ begins, the case where no trajectory generation technique is applied exhibits oscillations greater than $1\ rad$ ($\approx 57{^\circ}$). When *input shaping* (configuration II) is applied, the imbalance is significantly mitigated, but still present in the order of up to $0,1\ rad$ ($\approx 5,7{^\circ}$) approximately. The response of configuration III practically does not exhibit oscillation in a steady state.

To aid in the analysis of drone attitude control performance, Figures Figure 5.6 and Figure 5.7 present the behavior of the variables that describe the drone's orientation for the 15 and 10-second simulations with configurations II and III, respectively.

<img src="media/image45.emf" style="width:2.8125in;height:4.78264in" /> <img src="media/image46.emf" style="width:2.70359in;height:4.78346in" />

Figure 5.6 - Aircraft attitude behavior for 15 and 10-second simulations with configuration II.

<img src="media/image47.emf" style="width:2.44942in;height:4.33071in" /><img src="media/image48.emf" style="width:2.47017in;height:4.33071in" />

Figure 5.7 – Aircraft attitude behavior for 15 and 10-second simulations with configuration III.

Comparing Figures Figure 5.6 and Figure 5.7, it can be seen that, for a 15-second simulation, while the controller with only *input shaping* (Figure 5.6) receives a null reference for the roll and pitch angles, these variables exhibit a similar behavior to the second, when well-defined trajectories are used based on the differentially flat model.

In the 10-second simulation, however, the second solution presented more aggressive and amplified behavior, which also reflected in Table 5.3 through the metrics $\beta_{RMS}$ and ${\overline{f}}_{\omega}$. Although it presented smaller values for these parameters, the first solution (configuration II) presents a noisy profile that becomes clearer when observing the angular speeds, as shown in Figures Figure 5.8 and Figure 5.9.

In both configurations, the yaw control was effective, showing a slight deviation towards configuration III (Figure 5.6) of the 10-second simulation.

<img src="media/image45.emf" style="width:2.66667in;height:4.03472in" /><img src="media/image46.emf" style="width:2.3125in;height:4.03472in" />

Figure 5.8 – Angular speed of the drone for 15s and 10s simulations with configuration II.

<img src="media/image49.emf" style="width:2.67708in;height:4.03403in" /><img src="media/image48.emf" style="width:2.24167in;height:4.0352in" />

Figure 5.9 - Angular velocity of the drone for 15s and 10s simulations with configuration III.

Finally, it is worth comparing the input signal obtained for configurations II and III. Figures Figure 5.10 and Figure 5.11 show the forces produced by the controller for simulations with trajectories of 15 and 10 seconds, and for configuration III (Figure 5.11), the ideal forces obtained through the differentially flat model are also presented.

<img src="media/image50.emf" style="width:3.075in;height:5.29245in" /> <img src="media/image51.emf" style="width:3.34583in;height:5.28302in" />

Figure 5.10 – Control efforts for 15s and 10s simulations with configuration II.

<img src="media/image52.emf" style="width:3.34583in;height:5.29245in" /><img src="media/image53.emf" style="width:3.04653in;height:5.29245in" />

Figure 5.11 - Efforts to control simulations of 15s and 10s with configuration III.

In general, it is observed that the torque produced by solution III (Figure 5.10) oscillates at a high frequency around low values for both operating times, but that it reaches peaks higher than those observed in the behavior of configuration IV (Figure 5.11) for the 15-second trajectory. This behavior influences the behavior of the angular speed (Figure 5.8) and the metric ${\overline{f}}_{\omega}$ of Table 5.3.

Regarding thrust, both scenarios show similar behavior for the 15-second trajectory. However, for the 10-second trajectory, the controller based on the differentially flat model (configuration IV) exhibits relatively high amplitude and frequency oscillations (justifying the high value of ${\overline{f}}_{\omega}$ in Table 5.3). This effect is even more pronounced for the torques.

Analyzing the Figure 5.11, it is observed that the signals obtained by the differentially flat model with those calculated by the controller are very close, especially for the less aggressive maneuver. This behavior was not observed for the steering torque for the 10-second simulation, in which the signal generated by the controller does not reach the peaks predicted by the model. This deviation is consistent with the observation made in the steering angular position and speed profile shown in Figures Figure 5.7 and Figure 5.9, and this behavior may be associated with an insufficiency of the control parameters defined, especially $\kappa_{\psi}$ and $\eta_{\psi}$, which are directly related to robustness against external disturbances and internal uncertainties.

In summary, it was verified that the controller performs well in controlling the position of the *drone*, even in conditions of intense load swing, but it does not dampen the oscillations for arbitrarily defined trajectories. The trajectory generation solutions evaluated to address this problem showed good performance in attenuating the load oscillation, with particular emphasis on the configuration that incorporates the system's differential flatness property. However, this solution tended to degrade the aircraft's reference trajectory at a certain degree of maneuvering speed (which is not determined in this work, but could be explored in the future). The solution that applies only *input shaping*, on the other hand, maintained stable performance in all aspects (with the caveat of high-frequency components in the aircraft's attitude) for all evaluated maneuvering speeds.

6.  

# CONCLUSIONS

Firstly, this work develops a dynamic model of the drone system with a suspended load, explicitly stating the equations for the aircraft's acceleration and the angles that describe the cable's orientation so that it can be used by the developed controller.

Subsequently, the derivation of the sliding mode control (SMC) solution is presented, developed with a focus on addressing the high degree of underactuation of the system by leveraging the knowledge of the dynamic model. A classic SMC is defined to control the altitude and heading angle of the aircraft, and another that explicitly accounts for the underactuation characteristic of the aircraft's horizontal displacement dynamics. Through the concept of Lyapunov stability, it is demonstrated that the system is stable under the effect of disturbances whose limits are known. It is also demonstrated that the system is locally stable in horizontal displacement through the Routh-Hurwitz stability criterion applied to the sliding and linearized phase of the system around the equilibrium point, deducing constraints for the control gains and reference attitude values. With this, the behavior of the convergence of the controller was analyzed for some characteristic scenarios, such as in cases where the derived conditions for the control parameters are not satisfied.

To reduce the load balance and, consequently, its effect on the aircraft's dynamics, investments are made in trajectory generation techniques based on the system's differential flatness property and *input shaping*. It has been observed that by defining polynomial trajectories point-to-point with zero derivatives at the load points, the differentially flat model generates valid trajectories for the aircraft, but which tend to oscillate prohibitively for the controller in more aggressive maneuvers. Thus, as an alternative to more sophisticated optimization solutions, the application of *input shaping* on the defined trajectory for the load was proposed, embedding the knowledge of the balance dynamics in the input. It was observed that this strategy led to a smoothing of the resulting trajectories from the *drone*, enabling the execution of more aggressive maneuvers.

Finally, the controller's performance was evaluated in executing a point-to-point trajectory defined by polynomial interpolation, with different configurations of handling these trajectories and varying the maneuver time. In the first configuration, the input curve is passed directly as a reference to the drone, then *input shaping* is applied to this curve and passed as a reference to the aircraft, and finally, a combined strategy is applied, in which *input shaping* is applied to the trajectory defined for the load, followed by applying the differential flat model and generating references for the aircraft.

It was found that the controller is capable of stabilizing the aircraft's position in all cases, fulfilling its primary objective. However, the load exhibits significant oscillations when the reference signal is not properly addressed. Solutions that apply *input shaping* promote a significant reduction in the load's sway. Among these solutions, the one that considers the differentially flat model performed best, especially for low and medium aggressiveness maneuvers, exhibiting a better attenuation factor, lower position error, and aircraft attitude rate. However, its performance decreases from a certain maneuver speed threshold, when the resulting reference signal for the drone becomes excessively aggressive for the application. The solution that applies *input shaping* directly to the reference trajectory yields the best results in this scenario. In reality, this configuration is the one that showed the greatest consistency across different maneuvers with satisfactory performance in terms of load attenuation, although it exhibited high-frequency components in the aircraft's attitude state over longer time intervals.

Finally, it is important to raise some points that can still be explored in the future to consolidate the proposed solution. As already presented throughout the text, first, it proposes to identify the conditions in which the combination of *input shaping* with the differentially flat model leads to infeasible trajectories and to analyze the robustness of the system, verifying its performance in the presence of external disturbances, parameter uncertainty, and internal perturbations. Among other numerous possible research opportunities, it is suggested to update the controller to incorporate the feedback of the load state in order to promote an active and not fully rely on open-loop solutions, as well as to explore the state estimation problem, especially the position of the load, which is one of the biggest challenges of this application, and then move on to testing with the real system.

# BIBLIOGRAPHIC REFERENCES

**Aerial Crane & Helicopter Lifting Services**. text/html. Available at: <http://www.pdgaviationservices.com/services/aerial-crane>. Accessed on: August 17, 2018.

ALEXIS, K. et al. Robust Model Predictive Flight Control of Unmanned Rotorcrafts. **Journal of Intelligent & Robotic Systems**, v. 81, n. 3–4, p. 443–469, Mar. 2016.

ASHRAFIUON, H.; ERWIN, R. S. **Sliding control approach to underactuated multibody systems**. Proceedings of the 2004 American Control Conference. **Proceedings**... In: PROCEEDINGS OF THE 2004 AMERICAN CONTROL CONFERENCE. June 2004

ASHRAFIUON, H.; ERWIN, R. S. Sliding mode control of underactuated multibody systems and its application to shape change control. **International Journal of Control**, v. 81, n. 12, p. 1849–1858, 1 December 2008.

BISGAARD, M. **Modeling, Estimation, and Control of Helicopter Slung Load System**. \[s.l.\] Department of Control Engineering, Aalborg University, 2008.

BISGAARD, M.; COUR-HARBO, A. LA; BENDTSEN, J. D. **Input Shaping for Helicopter Slung Load Swing Reduction**. AIAA Guidance, Navigation and Control Conference and Exhibit. **Proceedings**... In: AIAA GUIDANCE, NAVIGATION AND CONTROL CONFERENCE AND EXHIBIT. Honolulu, Hawaii: American Institute of Aeronautics and Astronautics, August 18, 2008. Available at: <https://arc.aiaa.org/doi/abs/10.2514/6.2008-6964>. Accessed: July 29, 2018

BISGAARD, M.; LA COUR-HARBO, A.; DIMON BENDTSEN, J. Adaptive control system for autonomous helicopter slung load operations. **Control Engineering Practice**, Special Issue on Aerial Robotics. v. 18, n. 7, p. 800–811, 1 Jul. 2010.

BOB, B. **Soldiers Train On Bambi Bucket To Fight Fires In Kosovo \| Helicopter Firefighting buckets and belly tanks \| Pinterest**. Available at: \<https://br.pinterest.com/pin/863213453549609958/\>. Accessed on: August 17, 2018.

BRESCIANI, T. **Modeling, Identification, and Control of a Quadrotor Helicopter**. \[Place of Publication: Publisher].

CROUSAZ, C. D.; FARSHIDIAN, F.; BUCHLI, J. **Aggressive optimal control for agile flight with a slung load**. in IROS 2014 Workshop on Machine Learning in Planning and Control of Robot Motion. **Proceedings**...2014

CROUSAZ, C. DE et al. **Unified motion control for dynamic quadrotor maneuvers demonstrated on slung load and rotor failure tasks**. 2015 IEEE International Conference on Robotics and Automation (ICRA). **Proceedings**... In: 2015 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION (ICRA). May 2015

CRUZ, P. J.; FIERRO, R. Cable-suspended load lifting by a quadrotor UAV: hybrid model, trajectory generation, and control. **Autonomous Robots**, v. 41, n. 8, p. 1629–1643, 1 Dec. 2017.

DAI, S.; LEE, T.; BERNSTEIN, D. S. **Adaptive control of a quadrotor UAV transporting a cable-suspended load with unknown mass**. 53rd IEEE Conference on Decision and Control. **Proceedings**... In: 53RD IEEE CONFERENCE ON DECISION AND CONTROL. Los Angeles, CA, USA: IEEE, December 2014

DING, X. et al. A review of aerial manipulation of small-scale rotorcraft unmanned robotic systems. **Chinese Journal of Aeronautics**, June 22, 2018.

FAUST, A. et al. **Learning swing-free trajectories for UAVs with a suspended load**. 2013 IEEE International Conference on Robotics and Automation. **Proceedings**... In: 2013 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION. May 2013

FAUST, A. et al. Automated aerial suspended cargo delivery through reinforcement learning. **Artificial Intelligence**, Special Issue on AI and Robotics. v. 247, p. 381–398, 1 June 2017.

FENG, Y. et al. **Adaptive controller design for generic quadrotor aircraft platform subject to slung load**. 2015 IEEE 28th Canadian Conference on Electrical and Computer Engineering (CCECE). **Proceedings**... In: 2015 IEEE 28TH CANADIAN CONFERENCE ON ELECTRICAL AND COMPUTER ENGINEERING (CCECE). May 2015

FLIESS, M. et al. ON DIFFERENTIALLY FLAT NONLINEAR SYSTEMS. In: FLIESS, M. (Ed.). . **Nonlinear Control Systems Design 1992**. IFAC Symposia Series. Oxford: Pergamon, 1993. p. 159–163.

FREDDI, A.; LANZON, A.; LONGHI, S. A Feedback Linearization Approach to Fault Tolerance in Quadrotor Vehicles. **IFAC Proceedings Volumes**, 18th IFAC World Congress. v. 44, n. 1, p. 5413–5418, 1 Jan. 2011.

GHAZALI, R. et al. Performance Comparison between Sliding Mode Control with PID Sliding Surface and PID Controller for an Electro-hydraulic Positioning System. **International Journal on Advanced Science, Engineering and Information Technology**, v. 1, n. 4, p. 447-452–452, 2011.

GOODARZI, F. A.; LEE, D.; LEE, T. **Geometric stabilization of a quadrotor UAV with a payload connected by flexible cable**. 2014 American Control Conference. **Proceedings**... In: 2014 AMERICAN CONTROL CONFERENCE. June 2014

GUERRERO, M. E. et al. **IDA-PBC methodology for a quadrotor UAV transporting a cable-suspended payload**. 2015 International Conference on Unmanned Aircraft Systems (ICUAS). **Proceedings**... In: 2015 INTERNATIONAL CONFERENCE ON UNMANNED AIRCRAFT SYSTEMS (ICUAS). Denver, CO, USA: IEEE, June 2015

GUERRERO, M. E. et al. **Passivity-based control for a quadrotor UAV transporting a cable-suspended payload with minimum swing**. 2015 54th IEEE Conference on Decision and Control (CDC). **Proceedings**... In: 2015 54TH IEEE CONFERENCE ON DECISION AND CONTROL (CDC). Dec. 2015b

GUERRERO-SÁNCHEZ, M. E. et al. Swing-attenuation for a quadrotor transporting a cable-suspended payload. **ISA Transactions**, vol. 68, pp. 433–449, May 1, 2017a.

GUERRERO-SÁNCHEZ, M. E. et al. Swing-attenuation for a quadrotor transporting a cable-suspended payload. **ISA Transactions**, v. 68, p. 433–449, May 1, 2017b.

HOSSAIN, E. et al. Sliding Mode Controller and Lyapunov Redesign Controller to Improve Microgrid Stability: A Comparative Analysis with CPL Power Variation. **Energies**, v. 10, n. 12, p. 1959, December 2017.

JEAONG, H. et al. **Simulation and Flight Experiment of a Quadrotor Using Disturbance Observer Based Control**. 2018

KLAUSEN, K.; FOSSEN, T. I.; JOHANSEN, T. A. **Nonlinear control of a multirotor UAV with suspended load**. 2015 International Conference on Unmanned Aircraft Systems (ICUAS). **Proceedings**... In: 2015 INTERNATIONAL CONFERENCE ON UNMANNED AIRCRAFT SYSTEMS (ICUAS). Denver, CO, USA: IEEE, June 2015

KLAUSEN, K.; FOSSEN, T. I.; JOHANSEN, T. A. Nonlinear Control with Swing Damping of a Multirotor UAV with Suspended Load. **Journal of Intelligent & Robotic Systems**, v. 88, n. 2–4, p. 379–394, 1 Dec. 2017.

KOTARU, P.; WU, G.; SREENATH, K. **Dynamics and control of a quadrotor with a payload suspended through an elastic cable**. 2017 American Control Conference (ACC). **Proceedings**... In: 2017 AMERICAN CONTROL CONFERENCE (ACC). May 2017

KUI, Y. et al. **Sliding mode control for a quadrotor slung load system**. 2017 36th Chinese Control Conference (CCC). **Proceedings**... In: 2017 36TH CHINESE CONTROL CONFERENCE (CCC). July 2017

LEE, T.; LEOK, M.; MCCLAMROCH, N. H. **Geometric tracking control of a quadrotor UAV on SE(3)**. 49th IEEE Conference on Decision and Control (CDC). **Proceedings**... In: 49TH IEEE CONFERENCE ON DECISION AND CONTROL (CDC). December 2010

MELLINGER, D. **Trajectory generation and control for quadrotors**. PhD—\[s.l.\]. University of Pennsylvania, 2012.

MELLINGER, D.; KUMAR, V. **Minimum snap trajectory generation and control for quadrotors**. 2011 IEEE International Conference on Robotics and Automation. **Proceedings**... In: 2011 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION. Shanghai, China: IEEE, May 2011. Available at: <http://ieeexplore.ieee.org/abstract/document/5980409/>

NISE, N. S. **Control Systems Engineering**. 6th ed ed. Hoboken, NJ: John Wiley & Sons, Incorporated, 2011.

NOTTER, S. et al. Modelling, Simulation and Flight Test of a Model Predictive Controlled Multirotor with Heavy Slung Load. **IFAC-PapersOnLine**, 20th IFAC Symposium on Automatic Control in Aerospace ACA 2016. v. 49, n. 17, p. 182–187, 1 Jan. 2016.

ORE, J.-P. et al. Autonomous Aerial Water Sampling. **Journal of Field Robotics**, v. 32, n. 8, p. 1095–1113, 2015.

PALUNKO, I.; FIERRO, R.; CRUZ, P. **Trajectory generation for swing-free maneuvers of a quadrotor with suspended payload: A dynamic programming approach**. 2012 IEEE International Conference on Robotics and Automation. **Proceedings**. In: 2012 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION. May 2012

PERKOWSKI, M. **Helicopters cleared for Christmas tree harvest**. Available at: \<http://www.capitalpress.com/Oregon/20151118/helicopters-cleared-for-christmas-tree-harvest\>. Accessed on: Aug 17, 2018.

PROUTY, R. W. **Helicopter Performance, Stability, and Control**. 2002 edition ed. Malabar, Fla.: Krieger Pub Co, 2001.

QIAN, D.; YI, J. **Hierarchical Sliding Mode Control for Under-actuated Cranes: Design, Analysis and Simulation**. Berlin Heidelberg: Springer-Verlag, 2015.

RAFFO, G. V.; ALMEIDA, M. M. DE. **Nonlinear robust control of a quadrotor UAV for load transportation with swing improvement**. 2016 American Control Conference (ACC). **Proceedings**... In: 2016 AMERICAN CONTROL CONFERENCE (ACC). July 2016

RAMLI, L. et al. Control strategies for crane systems: A comprehensive review. **Mechanical Systems and Signal Processing**, v. 95, p. 1–23, 1 Oct. 2017.

SANKARANARAYANAN, V.; MAHINDRAKAR, A. D. Control of a Class of Underactuated Mechanical Systems Using Sliding Modes. **IEEE Transactions on Robotics**, v. 25, n. 2, p. 459–467, April 2009.

SHEPHERD, J.; JARVIS, A.; HUNT, T. **BC Hydro Delivers Power and Progress**. Available at: \<https://www.tdworld.com/transmission/bc-hydro-delivers-power-and-progress\>. Accessed on: August 17, 2018.

SHTESSEL, Y. et al. **Sliding Mode Control and Observation**. 2013 edition ed. New York: Birkhäuser, 2013.

SINGH, T.; SINGHOSE, W. **Input shaping/time delay control of maneuvering flexible structures**. Proceedings of the 2002 American Control Conference (IEEE Cat. No.CH37301). **Proceedings**... In: PROCEEDINGS OF THE 2002 AMERICAN CONTROL CONFERENCE (IEEE CAT. NO.CH37301). May 2002

SLABAUGH, G. G. Computing Euler angles from a rotation matrix. **Retrieved on August**, v. 6, n. 2000, p. 39–63, 1999.

SPONG, M. W.; HUTCHINSON, S.; VIDYASAGAR, M. **Robot Modeling and Control**. 1st edition. Hoboken, NJ: Wiley, 2005.

SREENATH, K.; LEE, T.; KUMAR, V. **Geometric control and differential flatness of a quadrotor UAV with a cable-suspended load**. 52nd IEEE Conference on Decision and Control. **Proceedings**... In: 52ND IEEE CONFERENCE ON DECISION AND CONTROL. Florence, Italy: IEEE, Dec. 2013

SREENATH, K.; MICHAEL, N.; KUMAR, V. **Trajectory generation and control of a quadrotor with a cable-suspended load - A differentially-flat hybrid system**. 2013 IEEE International Conference on Robotics and Automation. **Proceedings**... In: 2013 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION. May 2013

UTKIN, V.; GULDNER, J.; SHI, J. **Sliding Mode Control in Electro-Mechanical Systems**. \[s.l.\]. CRC Press, 2009.

VARGAS MORENO, A. E. **Machine learning techniques to estimate the dynamics of a slung load multirotor UAV system**. PhD—\[s.l.\]. University of Glasgow, 2017.

WANG, W. et al. Design of a stable sliding-mode controller for a class of second-order underactuated systems. **IEE Proceedings - Control Theory and Applications**, v. 151, n. 6, p. 683–690, November 2004.

WANG, W.; LIU, X. D; YI, J. Q. Structure design of two types of sliding-mode controllers for a class of under-actuated mechanical systems. **IET Control Theory Applications**, v. 1, n. 1, p. 163–172, January 2007.

XIONG, J.-J.; ZHENG, E.-H. Position and attitude tracking control for a quadrotor UAV. **ISA Transactions**, v. 53, n. 3, p. 725–731, May 1, 2014.

XU, R.; ÖZGÜNER, Ü. Sliding mode control of a class of underactuated systems. **Automatica**, v. 44, n. 1, p. 233–241, 1 Jan. 2008.

ZHENG, E.-H.; XIONG, J.-J.; LUO, J.-L. Second order sliding mode control for a quadrotor UAV. **ISA Transactions**, Disturbance Estimation and Mitigation. v. 53, n. 4, p. 1350–1356, 1 Jul. 2014.

ZHOU, X. et al. **Stabilization of a Quadrotor With Uncertain Suspended Load Using Sliding Mode Control**. ASME Proceedings \| 40th Mechanisms and Robotics Conference. **Proceedings**... In: 40TH MECHANISMS AND ROBOTICS CONFERENCE. Charlotte, North Carolina, USA: ASME, August 21, 2016. Available at: <http://dx.doi.org/10.1115/DETC2016-60060>. Accessed on: March 10, 2018

ZÜRN, M. et al. **MPC-controlled multirotor with suspended slung load: System architecture and visual load detection**. 2016 IEEE Aerospace Conference. **Proceedings**... In: 2016 IEEE AEROSPACE CONFERENCE. Mar. 2016

# APPENDIX I – CINEMATIC TRANSFORMATIONS

This appendix presents the procedure for obtaining the kinematic transformations between the inertial coordinate system $\Sigma_{i}$ and the non-inertial coordinate system $\Sigma_{b}$, represented in the text by equations (2.1) and (2.2). It should be noted that the content of this section is based on (BRESCIANI, 2008).

## Euler Angles

As stated in section 2.1, to represent the orientation of the non-inertial coordinate system $\Sigma_{b}$ with respect to the fixed reference frame $\Sigma_{i}$, the Euler notation is used, in which, starting from the inertial reference frame, three consecutive rotations are applied, as illustrated in Figure I-1:

<img src="media/image54.emf" style="width:5.78014in;height:1.78125in" />

Figure I-1 – Sequence of rotations between the inertial coordinate axis and the non-inertial axis.

As illustrated in Figure I-1, first, you rotate around the z-axis (${\overrightarrow{e}}_{z}$) by an angle $\psi$, resulting in the intermediate coordinate system $\Sigma_{c} = \left\lbrack {\overrightarrow{e}}_{x}^{c},{\overrightarrow{e}}_{y}^{c},{\overrightarrow{e}}_{z}^{c} \right\rbrack$, with ${\overrightarrow{e}}_{z}^{c} = {\overrightarrow{e}}_{z}$. Then, you rotate around ${\overrightarrow{e}}_{y}^{c}$ by an angle $\theta$, resulting in the intermediate reference frame $\Sigma_{d} = \left\lbrack {\overrightarrow{e}}_{x}^{d},{\overrightarrow{e}}_{y}^{d},{\overrightarrow{e}}_{z}^{d} \right\rbrack$, ${\overrightarrow{e}}_{y}^{d} = {\overrightarrow{e}}_{y}^{c}$. Finally, you rotate around ${\overrightarrow{e}}_{x}^{c}$ by an angle $\phi$, resulting in the non-inertial coordinate system $\Sigma_{b} = \left\lbrack {\overrightarrow{e}}_{x}^{b},{\overrightarrow{e}}_{y}^{b},{\overrightarrow{e}}_{z}^{b} \right\rbrack$.

Thus, a vector written in a given coordinate system that is rotated around one of the orthogonal axes of another reference frame can be transferred to this by multiplying it by the rotation matrix associated with that axis. Equations (I-1), (I-2) and (I-3) present the rotation matrices for the axes $x$, $y$ and $z$, respectively.

|                                                     |       |
|-----------------------------------------------------|-------|
| $$\mathbf{R}_{\mathbf{x}}(\phi) = \begin{bmatrix}   
 1 & 0 & 0 \\                                         
 0 & \cos\phi & - \sin\phi \\                         
 0 & \sin\phi & \cos\phi                              
 \end{bmatrix}$$                                      | (I-1) |
| $$\mathbf{R}_{\mathbf{y}}(\theta) = \begin{bmatrix} 
 \cos\theta & 0 & \sin\theta \\                       
 0 & 1 & 0 \\                                         
  - \sin\theta & 0 & \cos\theta                       
 \end{bmatrix}$$                                      | (I-2) |
| $$\mathbf{R}_{\mathbf{z}}(\psi) = \begin{bmatrix}   
 \cos\psi & - \sin\psi & 0 \\                         
 \sin\psi & \cos\psi & 0 \\                           
 0 & 0 & 1                                            
 \end{bmatrix}$$                                      | (I-3) |

Thus, the rotation matrix from the body frame to the inertial frame $\mathbf{R}$ presented in the text (Eq. (2.1)) is obtained by multiplying the simple rotation matrices in the defined order:

|                                                                                                            |       |
|------------------------------------------------------------------------------------------------------------|-------|
| $$\mathbf{R} = \mathbf{R}_{\mathbf{z}}(\psi)\mathbf{R}_{\mathbf{y}}(\theta)\mathbf{R}_{\mathbf{x}}(\phi)$$ | (I-1) |

It should be noted that, to perform the inverse transformation: from the inertial reference frame to the non-inertial one, you simply need to perform the same calculation with the inverse of the matrix $\mathbf{R}$, which corresponds to its transpose.

## Angular Velocity Transformation

The control law requires the determination of the angular rate of Euler angles $\overrightarrow{\Omega} = \left\lbrack \dot{\phi},\dot{\theta},\dot{\psi} \right\rbrack$, but the angular velocities are read and appear in the dynamic equations in the body frame, thus requiring a transformation between these coordinates. The rotation matrix does not apply in this case, as the angular rates of the Euler angles are not defined in the inertial frame. Figure I-1 shows that $\dot{\psi}$ occurs on the ${\overrightarrow{e}}_{z}$ axis, $\dot{\theta}$ occurs on the ${\overrightarrow{e}}_{y}^{c}$ axis, and $\dot{\phi}$ happens on the ${\overrightarrow{e}}_{x}^{b}$ axis. Therefore, the required transformation matrix is obtained as shown in Eq. (I-1):

|                                                                                 |       |
|---------------------------------------------------------------------------------|-------|
| $$\left\{ \begin{array}{r}                                                      
 p \\                                                                             
 q \\                                                                             
 r                                                                                
 \end{array} \right\} = \left\{ \begin{array}{r}                                  
 \dot{\phi} \\                                                                    
 0 \\                                                                             
 0                                                                                
 \end{array} \right\} + R_{x}^{T}(\phi)\left\{ \begin{array}{r}                   
 0 \\                                                                             
 \dot{\theta} \\                                                                  
 0                                                                                
 \end{array} \right\} + R_{x}^{T}(\phi)R_{y}^{T}(\theta)\left\{ \begin{array}{r}  
 0 \\                                                                             
 0 \\                                                                             
 \dot{\psi}                                                                       
 \end{array} \right\}$$                                                           | (I-1) |

Obtaining:

|                                        |       |
|----------------------------------------|-------|
| $$\left\{ \begin{array}{r}             
 p \\                                    
 q \\                                    
 r                                       
 \end{array} \right\} = \begin{bmatrix}  
 1 & 0 & - \sin\theta \\                 
 0 & \cos\phi & \sin\phi\cos\theta \\    
 0 & - \sin\phi & \cos\phi\cos\theta     
 \end{bmatrix}\left\{ \begin{array}{r}   
 \dot{\phi} \\                           
 \dot{\theta} \\                         
 \dot{\psi}                              
 \end{array} \right\}$$                  | (I-2) |

Observes that Eq. (I-2) corresponds to the inverse of Eq. (2.2).

# APPENDIX II – POLYNOMIAL INTERPOLATION BY PARTS

This section presents the technique of piecewise polynomial interpolation, commonly called a *spline*, used in the solution of trajectory planning for this work. First, the concept is illustrated for polynomials of 3rd order with continuous velocity, and then the analysis is extended to polynomials of higher order. It is noted that the content of this section is based on (SPONG; HUTCHINSON; VIDYASAGAR, 2005).

The procedure starts with defining the position between two points $q\left( t_{0} \right)$ and $q\left( t_{f} \right)$ as a cubic polynomial function of time:

|                                                     |        |
|-----------------------------------------------------|--------|
| $$q(t) = a_{0} + a_{1}t + a_{2}t^{2} + a_{3}t^{3}$$ | (II-1) |

Deriving from Eq. (II-II-1), the expression for the velocity is obtained:

|                                          |        |
|------------------------------------------|--------|
| $$v(t) = a_{1} + 2a_{2}t + 3a_{3}t^{2}$$ | (II-2) |

It can be observed that there are four unknown coefficients: $a_{0}$, $a_{1}$, $a_{2}$ and $a_{3}$; requiring four constraints to define them. In addition to specifying the final and initial positions, desired values for the speeds at these points are determined, so that:

|                                                                  |        |
|------------------------------------------------------------------|--------|
| $$q_{0} = a_{0} + a_{1}t_{0} + a_{2}t_{0}^{2} + a_{3}t_{0}^{3}$$ | (II-2) |
| $$q_{f} = a_{0} + a_{1}t_{f} + a_{2}t_{f}^{2} + a_{3}t_{f}^{3}$$ | (II-3) |
| $$v_{0} = a_{1} + 2a_{2}t_{0} + 3a_{3}t_{0}^{2}$$                | (II-4) |
| $$v_{f} = a_{1} + 2a_{2}t_{f} + 3a_{3}t_{f}^{2}$$                | (II-5) |

The equations II-2 to II-5 form a linear system (Eq. II-6) from which the values of the coefficients can be obtained:

|                                                 |        |
|-------------------------------------------------|--------|
| $$\begin{bmatrix}                               
 1 & t_{0} & t_{0}^{2} & t_{0}^{3} \\             
 0 & 1 & 2t_{0} & 3t_{0}^{2} \\                   
 1 & t_{f} & t_{f}^{2} & t_{f}^{3} \\             
 0 & 1 & 2t_{f} & 3t_{f}^{2}                      
 \end{bmatrix}\left\{ \begin{array}{r}            
 a_{0} \\                                         
 a_{1} \\                                         
 a_{2} \\                                         
 a_{3}                                            
 \end{array} \right\} = \left\{ \begin{array}{r}  
 q_{0} \\                                         
 v_{0} \\                                         
 q_{f} \\                                         
 v_{f}                                            
 \end{array} \right\}$$                           | (II-6) |

The Figure II-1 presents a curve generated by this method for the times $\left\lbrack t_{0},t_{f} \right\rbrack = \lbrack 0,4\rbrack$ and constraints $\left\lbrack q_{0},q_{f},v_{0},v_{f} \right\rbrack = \lbrack 0,4,0,0\rbrack$.

<img src="media/image55.emf" style="width:3.69792in;height:2.77178in" />

Figure II-1 – Example of a cubic polynomial trajectory with position and speed constraints at the extreme points.

To define a trajectory with multiple waypoints, simply generate polynomials in parts such that the position and initial velocity of one part coincide with the position and final velocity of the previous curve.

To extend the concept to polynomials of higher order, it is sufficient to identify the pattern of the solution. It can be verified that by specifying the position and velocity of the two points, 4 constraints are completed, which solve the determination of the 4 coefficients of the polynomial that determines the position over time. Thus, for each additional differentiation, the number of constraints and the order of the position polynomial increase by two. If $p$ is the order of the position polynomial and $d$ is the order of the specified highest derivative at the points of interest, then $p = 2d + 1$. Therefore, trajectories with specification up to the sixth derivative are obtained with polynomials of 13th order.

[^1]: Alternatively, you can substitute Eq. (2.24) in Eq. (2.23) and obtain an equation in terms of the charge position ${\overrightarrow{r}}_{L}$. The result is similar to Eq. (2.24).

[^2]: No de respeito a geração de trajetória, (SREENATH; MICHAEL; KUMAR, 2013) também considera momentos em que a tensão no cabo é nula, desenvolvendo um modelo dinâmico híbrido. O presente trabalho faz referência apenas à análise do modelo quando a tensão no cabo não é nula.