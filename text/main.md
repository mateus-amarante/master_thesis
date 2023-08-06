---
title: "Sliding Mode Control of a Quadrotor with a Suspended Load for Trajectories based on the Differential Flatness Property of the System and Input Shaping"
subtitle: "English Version"
author: "Mateus Amarante Araujo"
date: "December 12th, 2019"
lang: en
abstract-title: "**Abstract**"
description: "An informal translation of my master's dissertation"
abstract: |
  In the high demand for autonomous aircraft in cargo transport applications, this work presents a motion control and trajectory generation solution for a system composed of a quadrotor with a cable-suspended load to control the aircraft's position and reduce the load swing. First, the dynamic model of the system is derived using the Newton-Euler and Euler-Lagrange methods and divided into two parts: a fully actuated subsystem associated with the robot altitude and yaw angle and an underactuated subsystem associated with the other state variables of the quadrotor. Each subsystem is controlled by a sliding mode controller, which is proved to be stable in Lyapunov\'s sense for driving the system to the sliding surfaces and staying on them. It is demonstrated by the Routh-Hurwitz stability criterion that the sliding surfaces associated with the underactuated subsystem are locally stable, given some constraint rules obtained for the control parameters that make the tuning process more manageable. Finally, a new trajectory generation structure is proposed to suppress the load balance, which consists of building a polynomial trajectory for the load, applying input shaping on it, and computing the desired state of the aircraft by using the differential flatness property of the system. The controller is tested with the designed trajectory generator in simulation for a point-to-point trajectory and different durations. The controller effectively controls the aircraft state even when no filter is applied to the input reference, and the trajectory generator greatly reduces the load swing. However, the aircraft trajectory obtained by the differentially flat model becomes prohibitive for the controller when the reference speed and acceleration reach certain limits for which the alternative solution of only applying input shaping to the aircraft trajectory presents moderate performance.

  Araujo, M. A. **Sliding Mode Control of a Quadrotor with a Suspended Load for Trajectories based on the Differential Flatness Property of the System and Input Shaping.** 105 p. M. Sc. Dissertation, Federal University of Uberlandia, Uberlandia.
documentclass: article
thanks: |
  I would like to express my deepest gratitude to my supervisor, Roberto Finzi, for his confidence in my work, his commitment to distance communication, and his understanding and flexibility in allowing me to work in another state while working on my master’s degree.

  I am also grateful to Professor Leonardo Sanches, who was my co-supervisor and teacher in the first stage of the course. I learned many concepts used in this work from him.

  I would like to thank the Graduate Program of the School of Mechanical Engineering (Faculdade de Engenharia Mecânica - FEMEC) at the Federal University of Uberlândia (UFU) for providing a structure for study and research quality. This made me feel connected to the scientific universe.

  My thanks also go to my Autonomous Aircraft Laboratory (LAA) colleagues, especially Ivan Tarifa, Felipe Machini, and Douglas Costa. I shared relevant practical experiences in aeronautical robotics and entrepreneurship with them.

  I am grateful to CAPES for the scholarship offered during the first months of the program, enabling me to focus on academic studies.

  I would like to thank the psychologist Vanda for providing me with psychological support when I was anxious about starting graduate school and deciding to work and do my master’s degree in parallel.

  I am grateful to SENAI CIMATEC for providing me with a paid opportunity to work on research and development projects in robotics during my master’s degree. This served as an incentive to pursue post-graduation studies. This experience allowed me to work with competent professionals and state-of-the-art technology, contributing to my professional and academic growth.

  Special thanks go to my friend Murilo Mendonça, who promptly reviewed my work in the final stage.

  I would like to express my deepest gratitude to my parents Antonio and Cleide. They have always supported me unconditionally in all the challenging moments of life, which were many in the last two years.

  My heartfelt thanks go to my wife Camilla. We have lived together for almost a year, and her support has pushed me to finish the work on time.

  I am grateful to God for promoting circumstances of much learning, conquests, and joy in my life through sometimes tortuous lines but always right.

  In memory of my brother Filipe, who inspired me to conclude this work with his remarkable energy and determination.
bibliography: bibliography.bib
---

**Symbols**

| Symbol | Description |
| --- | --- |
| $$\alpha_{RMS}$$ | RMS value of the cable angle relative to the vertical during the settling time for samples from a simulation |
| $$\beta_{RMS}$$ | RMS value of the angle between the inertial vertical axis (${\overrightarrow{e}}_{z}$) and the non-inertial (${\overrightarrow{e}}_{z}^{b}$) for samples from a simulation |
| $$\mathbf{B}\left( \overrightarrow{q} \right)$$ | Input matrix of the equation that describes the dynamics associated with the translation of the aircraft and orientation of the cable |
| $$c_{x},\ c_{y},c_{z}$$ | Linear translational drag coefficient of the aircraft on the x, y and z axes of the body reference frame respectively |
| $$c_{d}$$ | Linear translational drag coefficient of the load |
| $$\mathbf{C}\left( \overrightarrow{q},\dot{\overrightarrow{q}} \right)$$ | Centrifugal and Coriolis force matrix of the equation that describes the dynamics associated with the translation of the aircraft and orientation of the cable |
| $$d$$ | Distance between opposite rotors |
| $$d_{x},d_{y},d_{x}$$ | Disturbance on quadcopter translational accelerations |
| $$d_{\phi},d_{\theta},d_{\psi}$$ | Disturbance on Euler angle accelerations describing aircraft orientation |
| $$d_{\phi_{L}},\ d_{\theta_{L}}$$ | Disturbance on accelerations of angles $\phi_{L}$ and $\theta_{L}$, which describe cable orientation |
| $${\overrightarrow{D}}_{F} = \left\lbrack D_{F}^{x},D_{F}^{y},D_{F}^{z} \right\rbrack^{T}$$ | Force disturbance on system in inertial reference frame |
| $${\overrightarrow{D}}_{\tau} = \left\lbrack D_{\tau}^{x},D_{\tau}^{y},D_{\tau}^{z} \right\rbrack^{T}$$ | Moment disturbance on aircraft in non-inertial reference frame |
| $$\Sigma^{i} = \left\lbrack {\overrightarrow{e}}_{x},{\overrightarrow{e}}_{y},{\overrightarrow{e}}_{z} \right\rbrack$$ | Inertial coordinate system |
| $$\Sigma^{b} = \left\lbrack {\overrightarrow{e}}_{x}^{b},{\overrightarrow{e}}_{y}^{b},{\overrightarrow{e}}_{z}^{b} \right\rbrack$$ | Non-inertial coordinate system located at center of mass of aircraft |
| $$\Sigma^{c} = \left\lbrack {\overrightarrow{e}}_{x}^{c},{\overrightarrow{e}}_{y}^{c},{\overrightarrow{e}}_{z}^{c} \right\rbrack$$ | Intermediate coordinate system resulting from rotation by angle $\psi$ around z-axis of inertial reference frame |
| $$\Sigma^{d} = \left\lbrack {\overrightarrow{e}}_{x}^{d},{\overrightarrow{e}}_{y}^{d},{\overrightarrow{e}}_{z}^{d} \right\rbrack$$ | Intermediate coordinate system resulting from rotation by angle $\theta$ around y-axis of $\Sigma^{c}$ reference frame |
| $${\overrightarrow{F}}_{b} = \left\lbrack 0,0,F_{z}^{b} \right\rbrack^{T}$$ | Resulting propulsion force on aircraft in body reference frame |
| $$F_{z}^{b} = \sum_{i = 1}^{4}F_{i}$$ | Propulsion force along ${{\overrightarrow{e}}_{b}}_{z}$, with $F_{i}$ being propulsion force generated by each rotor |
| $${\overrightarrow{F}}_{d}$$ | Linear translational drag force in system |
| $${\overline{f}}_{\omega}$$ | Average frequency of angular velocity $\overrightarrow{\omega}$ module calculated over samples from a simulation |
| $$g$$ | Gravitational acceleration |
| $$\mathbf{G}\left( \overrightarrow{q} \right)$$ | Vector associated with gravitational effort of equation that describes dynamics associated with translation of aircraft and orientation of cable |
| $${\overrightarrow{h}}_{\omega},{\overrightarrow{h}}_{\alpha}$$ | Result of cross product of angular velocity and acceleration in body reference frame with ${\overrightarrow{e}}_{z}^{b}$, respectively |
| $$\eta_{z},\eta_{\psi},\eta_{1},\eta_{2}$$ | Control parameters that ensure robustness against disturbances whose thresholds are known |
| $$\overrightarrow{\eta} = \lbrack\phi,\theta,\psi\rbrack^{T}$$ | Angles that define drone orientation according to Euler notation: roll, pitch and yaw angles, respectively |
| $${\overrightarrow{\eta}}_{L} = \left\lbrack \phi_{L},\theta_{L} \right\rbrack^{T}$$ | Angles that define cable orientation obtained by rotation around x-axis ($\phi_{L}$) followed by another around y-axis ($\theta_{L}$) |
| $$\mathbf{I}$$ | Quadcopter inertia tensor |
| $$\mathbf{J}$$ | Jacobian of subsystems associated with sliding variables of under-actuated subsystem controller in sliding condition |
| $$k_{t},k_{m}$$ | Propulsion and drag constants for propellers |
| $$\kappa_{z},\kappa_{\psi},\kappa_{1},\kappa_{2}$$ | Control parameters that linearly multiply sliding variables |
| $$l$$ | Cable length. |
| $$L$$ | Lagrangian associated with system translational energy. |
| $$\lambda_{z},\lambda_{\psi},\ \lambda_{i = 1,2,\ldots,8}$$ | Control parameters present in definition of sliding variables |
| $$M,\ m$$ | Masses of aircraft and load, respectively |
| $$\mathbf{M}\left( \overrightarrow{q} \right)$$ | Inertia matrix of equation that describes dynamics associated with translation of aircraft and orientation of cable |
| $$\overrightarrow{p}$$ | Unit vector from drone center of mass to center of load, describing cable orientation |
| $$\mathbf{P}\left( \overrightarrow{q} \right)$$ | Linear translational drag force matrix of equation that describes dynamics associated with translation of aircraft and orientation of cable |
| $$\overrightarrow{q} = \left\lbrack x,y,z,\phi_{L},\theta_{L} \right\rbrack$$ | Generalized coordinates for dynamic model associated with system translational energy |
| $$Q_{i}$$ | Generalized efforts for dynamic model associated with system translational energy |
| $$\overrightarrow{r} = \lbrack x,y,z\rbrack$$ | Drone position in inertial reference frame |
| $${\overrightarrow{r}}_{L} = \left\lbrack x_{L},y_{L},z_{L} \right\rbrack$$ | Load position in inertial reference frame |
| $$s_{1},s_{2},s_{3},s_{4}$$ | Sliding variables for controller |
| $$\overrightarrow{T} = T\space \overrightarrow{p}$$ | Traction force on cable with module $T$ along $\overrightarrow{p}$ |
| $${\overrightarrow{\tau}}_{b} = \left\lbrack \tau_{x}^{b},\ \tau_{y}^{b},\ \tau_{z}^{b} \right\rbrack^{T}$$ | Resulting moment around center of mass produced by rotor propulsion force and propeller drag |
| $$\overrightarrow{u} = \left\lbrack u_{1},u_{2},u_{3},u_{4} \right\rbrack^{T}$$ | Control signal, corresponding to system input efforts $\left\lbrack F_{b},\tau_{x}^{b},\tau_{y}^{b},\tau_{z}^{b} \right\rbrack^{T}$ |
| $$\overrightarrow{v} = \lbrack u,v,w\rbrack$$ | Drone translational velocity in non-inertial reference frame |
| $$V(x)$$ | Lyapunov function relative to variable $x$ |
| $${\widetilde{x}}_{b},{\widetilde{y}}_{b}$$ | $x$ and $y$ components for aircraft position error relative to non-inertial reference frame projected onto $xy$ plane |
| $$\omega_{n}$$ | Natural frequency for pendulum in linearized model |
| $$\overrightarrow{\omega} = \lbrack p,q,r\rbrack^{T}$$ | Drone angular velocity relative to non-inertial reference frame |
| $$\zeta$$ | Damping coefficient for pendulum in linearized model |



SUMÁRIO

1.  [INTRODUÇÃO [11](#_Toc29157956)](#_Toc29157956)

[1.1 Motivação e Aplicações [11](#motivação-e-aplicações)](#motivação-e-aplicações)

[1.2 Trabalhos Relacionados [14](#trabalhos-relacionados)](#trabalhos-relacionados)

[1.2.1 Controle em malha aberta [15](#controle-em-malha-aberta)](#controle-em-malha-aberta)

[1.2.2 Controle em Malha Fechada [17](#controle-em-malha-fechada)](#controle-em-malha-fechada)

[1.3 Objetivo e Contribuições [19](#objetivo-e-contribuições)](#objetivo-e-contribuições)

[1.4 Estrutura do Documento [21](#estrutura-do-documento)](#estrutura-do-documento)

2.  [MODELO DINÂMICO [22](#modelo-dinâmico)](#modelo-dinâmico)

[2.1 Drone sem Carga [22](#drone-sem-carga)](#drone-sem-carga)

[2.2 *Drone* com Carga Suspensa por Cabo [26](#drone-com-carga-suspensa-por-cabo)](#drone-com-carga-suspensa-por-cabo)

3.  [CONTROLE [33](#controle)](#controle)

[3.1 Introdução [33](#introdução-1)](#introdução-1)

[3.1.1 Características de Atuação [33](#características-de-atuação)](#características-de-atuação)

[3.1.2 Controle por Modos Deslizantes (CMD) [35](#controle-por-modos-deslizantes-cmd)](#controle-por-modos-deslizantes-cmd)

[3.2 Estratégia de Controle [38](#estratégia-de-controle)](#estratégia-de-controle)

[3.2.1 CMD do Subsistema Totalmente Atuado ($z,\psi$) [39](#cmd-do-subsistema-totalmente-atuado-zpsi)](#cmd-do-subsistema-totalmente-atuado-zpsi)

[3.2.2 CMD do Subsistema Sub-atuado [44](#cmd-do-subsistema-sub-atuado)](#cmd-do-subsistema-sub-atuado)

[3.2.3 Resumo [52](#resumo)](#resumo)

[3.3 Simulação [54](#simulação)](#simulação)

[3.3.1 Resposta do sistema para entrada degrau unitário [55](#resposta-do-sistema-para-entrada-degrau-unitário)](#resposta-do-sistema-para-entrada-degrau-unitário)

[3.3.2 Avaliação da condição de estabilidade na superfície deslizante [60](#avaliação-da-condição-de-estabilidade-na-superfície-deslizante)](#avaliação-da-condição-de-estabilidade-na-superfície-deslizante)

4.  [GERAÇÃO DE TRAJETÓRIAS [62](#geração-de-trajetórias)](#geração-de-trajetórias)

[4.1 Geração de Trajetória com Base na Planicidade Diferencial do Sistema [62](#geração-de-trajetória-com-base-na-planicidade-diferencial-do-sistema)](#geração-de-trajetória-com-base-na-planicidade-diferencial-do-sistema)

[4.1.1 Planicidade Diferencial do Sistema [63](#planicidade-diferencial-do-sistema)](#planicidade-diferencial-do-sistema)

[4.1.2 Determinação das Variáveis do Sistema [64](#determinação-das-variáveis-do-sistema)](#determinação-das-variáveis-do-sistema)

[4.1.3 Definição de Trajetórias para a Carga [68](#definição-de-trajetórias-para-a-carga)](#definição-de-trajetórias-para-a-carga)

[4.2 Input Shaping [69](#input-shaping)](#input-shaping)

[4.2.1 Fundamentação Teórica [69](#fundamentação-teórica)](#fundamentação-teórica)

[4.2.2 Input shaping aplicado ao problema [71](#input-shaping-aplicado-ao-problema)](#input-shaping-aplicado-ao-problema)

[4.3 Trajetórias Baseadas na Planicidade Diferencial do Sistema com *Input Shaping* [73](#trajetórias-baseadas-na-planicidade-diferencial-do-sistema-com-input-shaping)](#trajetórias-baseadas-na-planicidade-diferencial-do-sistema-com-input-shaping)

5.  [CONTROLADOR COM GERADOR DE TRAJETÓRIA [78](#controlador-com-gerador-de-trajetória)](#controlador-com-gerador-de-trajetória)

[5.1 Estrutura da Análise [78](#estrutura-da-análise)](#estrutura-da-análise)

[5.2 Análise dos Resultados [81](#análise-dos-resultados)](#análise-dos-resultados)

6.  [CONCLUSÕES [91](#conclusões)](#conclusões)

[REFERÊNCIAS BIBLIOGRÁFICAS [94](#referências-bibliográficas)](#referências-bibliográficas)[APÊNDICE I -- TRANSFORMAÇÕES CINEMÁTICAS [100](#apêndice-i-transformações-cinemáticas)](#apêndice-i-transformações-cinemáticas)[Ângulos de Euler [100](#ângulos-de-euler)](#ângulos-de-euler)

[Transformação da Velocidade Angular [101](#transformação-da-velocidade-angular)](#transformação-da-velocidade-angular)

[APÊNDICE II -- INTERPOLAÇÃO POLINOMIAL POR PARTES [103](#apêndice-ii-interpolação-polinomial-por-partes)](#apêndice-ii-interpolação-polinomial-por-partes)

# Introduction

The development of Unmanned Aerial Vehicles (UAVs), also called drones, has been very prominent in academic and business research in recent years. Compared to manned aerial vehicles, UAVs eliminate the risk to the pilot, reduce size and cost, and have a wide range of applications. They are used for vehicle traffic monitoring, transmission line inspection, remote sensing, and agricultural spraying (VALAVANIS; VACHTSEVANOS, 2015).

However, more recent research has focused on performing increasingly complex tasks. According to Ding et al. (2018), one branch of study with drones that has grown significantly in recent years is aerial manipulation, in which the aircraft physically interacts with the environment. This area involves various applications, such as cargo transportation, construction, and contact inspection. In addition to presenting great potential applications, this problem attracts the interest of researchers due to the engineering challenges involved, especially in modeling and control.

In this context, in alignment with state-of-the-art in air control and robotics, the present work proposes to perform multi-rotor aircraft control in cable-suspended cargo transportation.

## Motivation and Applications

Currently, helicopters equipped with load-hoisting elements are used in many applications, such as to transport timber in regions with difficult land access (Figure 1.1-a), to collect and launch water in firefighting missions (Figure 1.1-b), and to manipulate large structures (Figure 1.1-c), such as transmission towers (VARGAS MORENO, 2017; PDG Aviation Services, 2018).

![](./media/image2.emf){width="5.905511811023622in" height="2.6643044619422573in"}

Figure 1.1 - Applications of suspended cargo transport by helicopter. (a) timber transport[^1], (b) firefighting[^2] and (c) transmission tower transport[^3].

However, performing this type of operation can be challenging and dangerous, as the load swing significantly alters the helicopter's flight characteristics, conferring risk to the pilot, equipment, and surroundings. Therefore, there is significant interest from the aerospace industry in technologies that address the challenge of operating aircraft with cable-suspended loads more safely and accurately (BISGAARD; LA COUR-HARBO; DIMON BENDTSEN, 2010).

Autonomous aircraft also enables the aerial transport of smaller loads (Figure 1.2-a). It can be exploited for home deliveries, sending supplies to hard-to-reach regions in disaster situations (FAUST et al., 2017), rescuing people and animals in risky situations, and launching exploration robots in remote areas.

![](./media/image3.emf){width="5.905511811023622in" height="2.7103751093613297in"}

Figure 1.2 - Specific applications of drone with cable-suspended load. (a) supply transport (FAUST et al., 2017), (b) mine detection (BISGAARD, 2008), and (c) water sampling (ORE et al., 2015).

In addition to serving general cargo transportation, this system can have specific uses. For example, Bisgaard (2008) develops a complete control solution for a helicopter to handle mine location equipment (Figure 1.2-b). The aircraft keeps the apparatus stabilized near the ground, eliminating the risk of explosion and increasing the speed of the operation. Drones with cables are used to collect water samples for analysis in rivers and lakes, as shown in Figure 1.2-c). This eliminates the need to mobilize teams with boats, reducing cost and increasing the agility and safety of the operation (ORE et al., 2015).

Alternatively, loads can be transported using claws, but the additional inertia confers slower responses to aircraft orientation changes. The cable configuration maintains the aircraft’s rotational agility and the ability to transport cargo (SREENATH; MICHAEL; KUMAR, 2013). On the other hand, the suspended load adds more complexity to the system, presenting movement that is not directly controlled and sensitive to external disturbances, causing significant perturbations in the aircraft’s dynamics. Thus, it is necessary to develop specialized controllers considering this dynamic coupling to produce desired movements for the drone and cargo set.

## Related work

In the last decade, the academic community has shown significant interest in the proposed topic. Inspired by the studies of crane control done by Qian and Yi (2015) and Ramli et al. p. 20 (2017), drone control strategies with cable-suspended loads are classified into two categories: open-loop and closed-loop, as shown in Table 1.1.

Table 1.1 - Control techniques applied to the drone system with cable-suspended load.

| Category | Strategy | References |
| --- | --- | --- |
| Open Loop | Input Shaping | (BISGAARD; COUR-HARBO; BENDTSEN, 2008; KLAUSEN; FOSSEN; JOHANSEN, 2015, 2017) |
| | Trajectory optimization - dynamic programming | (PALUNKO; FIERRO; CRUZ, 2012) |
| | Trajectory generation by reinforcement learning | (FAUST et al., 2013, 2017) |
| | Trajectory generation based on the differential flatness property of the system | (SREENATH; LEE; KUMAR, 2013; SREENATH; MICHAEL; KUMAR, 2013) |
| Closed Loop | Backstepping | (KLAUSEN; FOSSEN; JOHANSEN, 2015, 2017) |
| | Sliding Mode Control (SMC) | (KUI et al., 2017; ZHOU et al., 2016) |
| | Passivity-based control (Model Based Control - MPC) | (GUERRERO et al., 2015a, 2015b; GUERRERO-SÁNCHEZ et al., 2017) |
| | Feedback Linearization | (DE ALMEIDA; RAFFO, 2015; PIZETTA; BRANDÃO; SARCINELLI-FILHO, 2015; SADR; MOOSAVIAN; ZARAFSHAN, 2014) |
| | Optimal control based on Linear Quadratic Regulator | (CROUSAZ; FARSHIDIAN; BUCHLI, 2014; CROUSAZ et al., 2015) |
| | Model Predictive Control (MPC) | (ALEXIS et al., 2016; NOTTER et al., 2016; SANTOS et al., 2017; ZÜRN et al., 2016) |
| | H~2~/H∞ | (RAFFO; ALMEIDA, 2016; REGO; RAFFO, 2016, 2019) |
| | Geometric Control | (GOODARZI; LEE; LEE, 2014; KOTARU; WU; SREENATH, 2017; SREENATH; LEE; KUMAR, 2013; SREENATH; MICHAEL; KUMAR, 2013) |

Despite the separation presented, solutions share multiple techniques. Many works combine different open-loop and closed-loop techniques, as will be done in this work.


### Open-loop control

Open-loop control techniques act on the modification of the reference signal based on prior information about the system's behavior without using feedback information from sensors. Among the techniques explored for this system are: input shaping, trajectory optimization, reinforcement learning, and analytical method based on the definition of a differentially flat system.

Input shaping is based on provoking transient oscillations in the system and then canceling them by inserting an input that would produce an opposite oscillation. This is done by convolving the reference signal with appropriately selected impulsive signals based on the natural frequency of the system (QIAN; YI, 2015). Several works (BISGAARD; COUR-HARBO; BENDTSEN, 2008; KLAUSEN; FOSSEN; JOHANSEN, 2015, 2017) apply this filter to trajectories defined for the aircraft, and the resulting swing of the load is substantially reduced compared to when the filter is not applied, using only closed-loop control strategies. Figure 1.3 exemplifies a simulation result demonstrating the effect of input shaping combined with a backstepping controller.

![](media/image4.png){width="2.965115923009624in" height="2.456608705161855in"}

Figure 1.3 - Simulation of the system with backstepping controller when using (orange) and not using (blue) input shaping (Source: KLAUSEN; FOSSEN; JOHANSEN, 2015).

Palunko; Fierro; Cruz (2012) apply an offline optimization procedure by dynamic programming to determine reference trajectories for the drone that, based on cost functions obtained on the response of a linearized and discrete system model, aim to minimize oscillations in the response. Faust et al. (2013, 2017) develop a non-oscillating trajectory generator through a reinforcement learning process with an approximate value iteration algorithm so that the inferred policy extends to domains beyond the training situation, being robust to noise and uncertainties in the model.

Sreenath; Lee; Kumar (2013) and Sreenath; Michael; Kumar (2013) demonstrate that the drone system with suspended load is differentially flat so that the state of the aircraft and input efforts can be completely determined if known the state of the load and yaw angle of the aircraft up to a certain derivative. Figure 1.4 presents an example of trajectories defined based on this property.

![](media/image5.png){width="3.8953488626421695in" height="2.7823917322834646in"}

Figure 1.4 - Illustration of drone trajectory calculated through differential flatness property of system from a circular trajectory defined for the load (Source: SREENATH; MICHAEL; KUMAR, 2013).

With this, they construct an optimization problem to determine trajectories for the load that minimize the intensity of the fourth derivative of aircraft position (snap), resulting in feasible curves to be performed."

### Closed-loop control

Since the system is non-linear, most control solutions are based on non-linear system analysis tools, such as the Lyapunov stability criterion. For example, Klausen; Fossen; Johansen (2015, 2017) develop a backstepping controller that guarantees compliance with arbitrary trajectories regardless of the pendulum movement, demonstrating that the equilibrium point is globally and asymptotically stable. Kui et al. (2017) implement a sliding mode controller for the position in cascade with another for the aircraft orientation, presenting robustness against external disturbances caused by wind gusts. Zhou et al. (2016) also developed a sliding mode controller for the aircraft position but in cascade to a proportional derivative controller for orientation.

Guerrero et al. (2015a, 2015b, 2017a) developed a controller based on the passivity principle. They describe the system dynamics using Hamilton’s formalism and design the inputs to shape the system’s energy behavior, injecting damping into it. They demonstrate that the solution can control the system from one point to another while maintaining reduced load balance in parameter uncertainty and without directly measuring the load position since the control law does not depend on it.

Another technique commonly applied to non-linear systems already explored in the context of cable-suspended load drone control is feedback linearization. For example, Pizetta; Brandão; Sarcinelli (2015) apply partial feedback linearization, where the load dynamics are considered a modeled disturbance, to stabilize the aircraft state without guaranteeing a reduction of load oscillation.

Many existing solutions express the control problem as an optimization problem. For example, Crousaz; Farshidian; Buchli (2014), and Crousaz et al. (2015) solve linear-quadratic problems on the model of the system linearized around reference trajectory points and a cost function, repeating this iteration multiple times in order to optimize trajectory and closed-loop controller simultaneously in fulfilling a specific task. The developed model considers moments when the cable is tensioned and free, demonstrating that the controller can control the system to pass the drone and load through a window whose height is less than that of the set, as illustrated in Figure 1.5.

![](media/image6.png){width="2.418604549431321in" height="2.111091426071741in"}

Figure 1.5 - Illustration of the maneuver to pass the quadcopter and load assembly through a window with dimensions smaller than the cable (Source: CROUSAZ et al., 2015).

Other authors propose various variations of model-based predictive control (Model Predictive Control - MPC), which is based on the principle of generating optimal inputs that consider the present state and predicted behavior of the system over a finite horizon (ALEXIS et al., 2016; NOTTER et al., 2016; SANTOS et al., 2017; ZÜRN et al., 2016). For example, Santos et al. (2017) develop an MPC that controls the position of the load and the yaw angle of the aircraft, which has only two variable-angle rotors (tiltrotor), as illustrated in Figure 1.6. In addition to aiming to comply with load trajectories and stabilize the aircraft, the solution compensates for parameter uncertainty, external disturbances on the load and variation in cable length and load mass during takeoff and landing maneuvers.

![](media/image7.png){width="2.5567399387576555in" height="2.1113331146106735in"}

Figure 1.6 - Two-propeller tiltrotor aircraft with suspended load (Source: SANTOS et al., 2017).

Rego; Raffo (2016, 2019) also control the system with the bi-rotor aircraft illustrated in Figure 1.6 from the perspective of load movement. For this, they develop a discrete-time H~2~ / H*∞* controller with pole location restriction with guaranteed properties imposed on the time response and robust to unmodeled dynamic behaviors, parameter uncertainty and external disturbances.

Many authors also apply geometric control, which avoids singularities and definition of coordinates in the system, being popular in the control of drones (without load) to perform aggressive maneuvers (LEE; LEOK; MCCLAMROCH, 2010). Goodarzi; Lee; Lee (2014) manages to control the drone to perform agile maneuvers while stabilizing the position of the cable and load set, modeled as a multi-joint arm in series. Sreenath; Lee; Kumar (2013); Sreenath; Michael; Kumar (2013) already propose control of the load position with trajectories defined based on the differential flatness property of the system.

## Objective and Contributions

The system under analysis meets various demands of society and presents a complexity that challenges researchers in the area of dynamics and control. There is a significant amount of recent work on this subject, but there is still no dominant solution, and there is much room to explore new techniques. Given this scenario, this work aims to develop a new solution that brings innovations, mainly in defining the closed-loop controller and the trajectory generation strategy.

This work applies a specific variation of the sliding mode control technique to control the drone's position, compensating for external disturbances and those caused by load movement, along with the combination of two open-loop techniques to mitigate load oscillation: input shaping and trajectory generation based on the differential flatness property of the system.

Sliding mode control (SMC) has already been successfully applied in controlling drones without load and overhead cranes. Two recent review articles on drone control (MO; FARID, 2018; ÖZBEK; ÖNKOL;EFE, 2016) select controllers based on sliding modes as one of those with the best cost-benefit among the solutions analyzed, and several reviews on overhead crane control point out this technique as one of the main ones for application (QIAN; YI, 2015; RAMLI et al., 2017). In general, SMC stands out for its simplicity, low computational cost, and robustness against external disturbances and model uncertainties, so its stability is generally demonstrated by Lyapunov's stability concept. The technique can also be combined with other methods and applied to under-actuated systems. On the other hand, SMC has the disadvantage of introducing high-frequency noise at the input that can cause system instability. However, fortunately, there are several strategies to circumvent this effect (SHTESSEL et al., 2013).

Despite its success in these related applications, this technique was little explored in the target application of this work. Only two works used sliding mode control directly (KUI et al., 2017; ZHOU et al., 2016). Both apply the technique to control the drone's position, compensating for external disturbances and those caused by load oscillation; however, they do not aim at stabilizing it.

Specifically, the control technique developed in this work is based on Zheng; Xiong; Luo (2014), and Xiong; Zheng (2014), who performed control of a quadrotor without load. Compared to these reference works, the solution innovates by adding the effect of suspended load to the model, proposing an alternative definition to sliding variables, and developing a strategy for determining control parameters so that the system is locally stable based on the Routh-Hurwitz stability criterion.

Regarding trajectory generation strategy, this work tests the application of input shaping on polynomial trajectories defined for the drone and the load. In case a trajectory is defined for load, a reference for the drone is obtained by using the differential flatness property of the system. It is demonstrated that this combination is a simple alternative to elaborate optimization strategies to generate feasible trajectories for the drone to be followed by the controller. No work combining these two techniques for this application was identified.

## Document Structure

This work is structured into six chapters:

- **Chapter I:** presents the motivations behind the research, a brief bibliographic review, and the presentation of the research's contribution points;

- **Chapter II:** deduction of the dynamic model of the system through the Newton-Euler and Euler-Lagrange methods.

- **Chapter III:** presents the developed control solution in detail in dealing with the system's under-actuation nature and stability analysis, also presenting a verification of the behavior of the controlled system in simulation;

- **Chapter IV:** discusses trajectory generation techniques based on the differential flatness property of the system and input shaping to then present the proposed combined solution;

- **Chapter V:** analysis of controller performance in simulation for different trajectory configurations, including proposed new combination,

- **Chapter VI:** compilation of results and indication of future work.

# DYNAMIC MODEL

This chapter presents the development of the dynamic model of the system, which, in general terms, is obtained by applying Newton-Euler and Euler-Lagrange equations (O'REILLY, 2008). It starts from understanding the dynamics of a drone without load to derive equations of the complete system so that the effects of adding load are evident.

## Drone without Load

Figure 2.1 schematizes a quadcopter, indicating coordinate systems and efforts applied to it.

![](media/image8.emf){width="3.0047298775153104in" height="2.6329593175853017in"}

Figure 2.1 - Schematic representation of drone without load indicating coordinate systems, applied forces, and moments, in addition to the direction of propeller rotation.

As indicated in Figure 2.1, an inertial reference frame $\Sigma_{i} = \left\lbrack {\overrightarrow{e}}_{x},{\overrightarrow{e}}_{y},{\overrightarrow{e}}_{z} \right\rbrack$ and a non-inertial (or body) $\Sigma_{b} = \left\lbrack {\overrightarrow{e}}_{x}^{b},{\overrightarrow{e}}_{y}^{b},{\overrightarrow{e}}_{z}^{b} \right\rbrack$ located at the center of mass of the aircraft, assumed coincident with the geometric center. Its position relative to the inertial reference frame is represented by $\overrightarrow{r} = \left\lbrack x,y,z \right\rbrack^{T}$, vector $\overrightarrow{v} = \left\lbrack u,v,w \right\rbrack^{T}$ represents linear velocity in non-inertial reference frame and vector $\overrightarrow{\omega} = \left\lbrack p,q,r \right\rbrack^{T}$ represents its angular velocity in non-inertial reference frame. The vehicle’s mass is represented by $M$, while d is the distance between one rotor and its opposite, and $g$ is gravitational acceleration.

The vehicle’s orientation is defined by Euler angles $\overrightarrow{\eta} = \left\lbrack \phi,\theta,\psi \right\rbrack^{T}$, also called roll, pitch and yaw angles, so that non-inertial coordinate system is obtained through three consecutive rotations around axes z, y and x respectively. The transformation of vector quantities defined in body reference frame to inertial reference frame is given by rotation matrix (APPENDIX I - KINEMATIC TRANSFORMATIONS):

  ------------------------------------------------------------------------------------------------------------------------- -----
  $$\mathbf{R} = \begin{bmatrix}                                                                                            (.)
  \cos\theta\cos\psi & \sin\phi\sin\theta\cos\psi - \cos\phi\sin\psi & \sin\phi\sin\theta + \cos\phi\sin\theta\cos\psi \\
  \cos\theta\sin\psi & \sin\phi\sin\theta\sin\psi + \cos\phi\cos\psi & \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \\
   - \sin\theta & \sin\phi\cos\theta & \cos\phi\cos\theta \\
  \end{bmatrix}$$

  ------------------------------------------------------------------------------------------------------------------------- -----

The transformation between angular velocities in non-inertial reference frame and rate of change of Euler angles is given by transformation matrix (APPENDIX I - KINEMATIC TRANSFORMATIONS):

  ------------------------------------------------------------------ -----
  $$\left\lbrack \begin{array}{r}                                    (.)
  \dot{\phi} \\
  \dot{\theta} \\
  \dot{\psi} \\
  \end{array} \right\rbrack = \begin{bmatrix}
  1 & \sin\phi{tg}\theta & \cos\phi{tg}\theta \\
  0 & \cos\phi & - \sin\phi \\
  0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta} \\
  \end{bmatrix}\left\lbrack \begin{array}{r}
  p \\
  q \\
  r \\
  \end{array} \right\rbrack$$

  ------------------------------------------------------------------ -----

The transformation between angular velocities in non-inertial reference frame and rate of change of Euler angles is given by transformation matrix (APPENDIX I - KINEMATIC TRANSFORMATIONS):

 ------------------------------------------------------------------ -----
 $$\left\lbrack \begin{array}{r} (.)
 \dot{\phi} \\
 \dot{\theta} \\
 \dot{\psi} \\
 \end{array} \right\rbrack = \begin{bmatrix}
 1 & \sin\phi{tg}\theta & \cos\phi{tg}\theta \\
 0 & \cos\phi & - \sin\phi \\
 0 & \frac{\sin\phi}{\cos\theta} & \frac{\cos\phi}{\cos\theta} \\
 \end{bmatrix}\left\lbrack \begin{array}{r}
 p \\
 q \\
 r \\
 \end{array} \right\rbrack$$

 ------------------------------------------------------------------ -----

It is worth noting that transformation matrix of angular velocities (Eq. 2.2) is singular for $\theta = \pm \frac{\pi}{2}$. Thus, restrictions for model are taken as $|\theta| < \frac{\pi}{2}$. To maintain symmetry, it is also imposed that $|\phi| < \frac{\pi}{2}$.

Figure 2.1 also indicates main efforts existing in aircraft. Main forces acting on system are weight force $- Mg{\overrightarrow{e}}_{z}$ and propulsion forces of rotors, indicated as $F_{1},F_{2},F_{3}$ and $F_{4}$, applied along ${\overrightarrow{e}}_{z}^{b}$. Resulting propulsion force is given by:

 ----------------------------------------------------------------------------- --------
 $$F_{z}^{b} = F_{1} + F_{2} + F_{3} + F_{4}$$ (.)

 $${\overrightarrow{F}}_{b} = \left\lbrack 0,0,F_{z}^{b} \right\rbrack^{T}$$ (.)
 ----------------------------------------------------------------------------- --------

Aircraft also suffers moment action on three axes: propulsion forces cause moments around ${\overrightarrow{e}}_{x}^{b}$ and ${\overrightarrow{e}}_{y}^{b}$ axes, while drag on propellers, which acts opposite to rotation movement of same, causes moment around ${\overrightarrow{e}}_{z}^{b}$ axis. Being $d$ distance between one rotor and its opposite and $\tau_{i}$ drag moment on each propeller, resulting moment on aircraft is given by:

  --------------------------------------------------------------------------------------------------------- -----
  $$\tau_{x}^{b} = d\left( F_{2} - F_{4} \right)$$                                                          (.)

  $$\tau_{y}^{b} = d\left( F_{1} - F_{3} \right)$$                                                          (.)

  $$\tau_{z}^{b} = \tau_{1} - \tau_{2} + \tau_{3} - \tau_{4}$$                                              (.)

  $${\overrightarrow{\tau}}_{b} = \left\lbrack \tau_{x}^{b},\tau_{y}^{b},\tau_{z}^{b} \right\rbrack^{T}$$   (.)
  --------------------------------------------------------------------------------------------------------- -----

The propulsion forces and the torque on the propellers due to drag, in turn, are proportional to the square of the rotation speed of each propeller $\Omega_{i}$:

  ----------------------------------------------------------------- -----
  $$F_{i} = k_{t}\Omega_{i}^{2}$$                                   (.)

  $$\tau_{i} = k_{m}\Omega_{i}^{2}$$                                (.)
  ----------------------------------------------------------------- -----

Here is the translation:

The constants $k_{t}$ and $k_{m}$ depend on air density, radius, shape, number and geometry of propellers, in addition to associated drag and lift coefficients (PROUTY, 2001).

Thus, mapping of rotor speeds (command variable to motor controllers) and efforts applied to drone, which will be inputs to controller to be detailed, is given by:

  ----------------------------------------------------------------- -----
  $$\overrightarrow{u} = \left\{ \begin{array}{r}                   (.)
  u_{1} \\
  u_{2} \\
  u_{3} \\
  u_{4} \\
  \end{array} \right\} = \left\{ \begin{array}{r}
  F_{z}^{b} \\
  \tau_{x}^{b} \\
  \tau_{y}^{b} \\
  \tau_{z}^{b} \\
  \end{array} \right\} = \begin{bmatrix}
  k_{t} & k_{t} & k_{t} & k_{t} \\
  0 & dk_{t} & 0 & - dk_{t} \\
  dk_{t} & 0 & - dk_{t} & 0 \\
  k_{m} & - k_{m} & k_{m} & - k_{m} \\
  \end{bmatrix}\left\{ \begin{array}{r}
  \Omega_{1}^{2} \\
  \Omega_{2}^{2} \\
  \Omega_{3}^{2} \\
  \Omega_{4}^{2} \\
  \end{array} \right\}$$

  ----------------------------------------------------------------- -----

It is emphasized that, given this direct mapping between rotor actuation and resulting force and moment on aircraft, control problem is taken up to definition of these efforts.

With this, dynamic model of quadcopter is obtained by applying Newton-Euler equations. Translational dynamics is written in inertial reference frame and obtained by equating rate of change of linear motion to sum of external forces:

  ---------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $$\frac{d}{dt}\left( M\dot{\overrightarrow{r}} \right) = \sum_{}^{}{\overrightarrow{F}}_{ext}$$                                                            (.)

  $$M\ \ddot{\overrightarrow{r}} = \mathbf{R}{\overrightarrow{F}}_{b} - Mg{\overrightarrow{e}}_{z} + {\overrightarrow{F}}_{d} + {\overrightarrow{D}}_{F}$$   (.)
  ---------------------------------------------------------------------------------------------------------------------------------------------------------- -----

In Eq. (2.13), ${\overrightarrow{F}}_{d}$ refers to translational drag force on aircraft, modeled as proportional to drone's speed (FREDDI; LANZON; LONGHI, 2011):

 -------------------------------------------------------------------------------------------------------------- ------
 $${\overrightarrow{F}}_{d} = {- \left\lbrack c_{x}\dot{x},{\ c}_{y}\dot{y},c_{z}\dot{z} \right\rbrack}^{T}$$ (2.)

 -------------------------------------------------------------------------------------------------------------- ------

Being $c_{x}$, $c_{y}$ and $c_{z}$ translational drag coefficients in each direction. ${\overrightarrow{D}}_{F}$ refers to unmodeled force disturbances. Isolating acceleration terms from Eq. (2.13), we have:

 -------------------------------------------------------------------------------------------------------------------------------- -----
 $$\left\{ \begin{aligned} (.)
 & \ddot{x} = \frac{1}{M}\left( \cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi \right)u_{1} - \frac{c_{x}}{M}\dot{x} + d_{x} \\
 & \ddot{y} = \frac{1}{M}\left( \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \right)u_{1} - \frac{c_{y}}{M}\dot{y} + d_{y} \\
 & \ddot{z} = - g + \frac{1}{M}\left( \cos\phi\cos\theta \right)u_{1} - \frac{c_{z}}/M)\dot{z} + d_{z} \\
 \end{aligned} \right.\ $$

 -------------------------------------------------------------------------------------------------------------------------------- -----

In Eq. (2.15), $d_{x}$, $d_{y}$ and $d_{z}$ refer to effects of disturbance ${\overrightarrow{D}}_{F}$ on each component of translational acceleration.

Rotational dynamics is taken in body reference frame and obtained by equating rate of change of angular momentum to sum of external moments:

 ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
 $$\frac{d}{dt}\left( \mathbf{I}\overrightarrow{\omega} \right) = \sum_{}^{}{\overrightarrow{\tau}}_{ext}^{b}$$ (.)

 $$\mathbf{I}\dot{\overrightarrow{\omega}} + \overrightarrow{\omega} \times \left( \mathbf{I}\overrightarrow{\omega} \right) = {\overrightarrow{\tau}}^{b} + {\overrightarrow{D}}_{\tau}$$ (.)
 ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

In Eq. (2.17), $\mathbf{I}$ is drone's inertia matrix, given by $\text{diag }\left( I_{x},I_{y},I_{z} \right)$, so that aircraft is considered symmetrical and with center of mass coincident with its geometric center. Rate of change of angular momentum includes, in addition to term of angular acceleration ($\dot{\overrightarrow{\omega}}$) and a portion referring to variation of direction of drone's angular momentum ($\overrightarrow{\omega} \times (\mathbf{I}\overrightarrow{\omega})$). External moments already include ${\overrightarrow{\tau}}^{b}$ from Eq. (2.8) and unmodeled moment disturbances (${\overrightarrow{D}}_{τ}$).

Isolating acceleration terms from Eq. (2.17), we have:

  --------------------------------------------------------------------------------------------------- -----
  $$\left\{ \begin{aligned}                                                                           (.)
  \dot{p} & = \frac{\left( I_{y} - I_{z} \right)}{I_{x}}qr + \frac{1}{I_{x}}u_{2} + D_{\tau}^{x} \\
  \dot{q} = \frac{\left( I_{z} - I_{x} \right)}{I_{y}}pr + \frac{1}{I_{y}}u_{3} + D_{\tau}^{y} \\
  \dot{r} & = \frac{\left( I_{x} - I_{y} \right)}{I_{z}}pq + \frac{1}{I_{z}}u_{4} + D_{\tau}^{z} \\
  \end{aligned} \right.\ $$

  --------------------------------------------------------------------------------------------------- -----

From control perspective, however, work is done with rate of change of Euler angles $\dot{\overrightarrow{\eta}}$, which requires use of transformation described in Eq. (2.2). This transformation, in turn, generates high complexity to equations, worth taking a simplification commonly made in literature: $\lbrack p,q,r\rbrack \approx \lbrack\dot{\phi},\dot{\theta},\dot{\psi}\rbrack$, which is exact for equilibrium point where $\phi = 0$ and $\theta = 0$, resulting in:

 ------------------------------------------------------------------------------------------------------------------------- -----
 $$\left\{ \begin{aligned} (.)
 \ddot{\phi} & = \frac{\left( I_{y} - I_{z} \right)}{I_{x}}\dot{\theta}\dot{\psi} + \frac{1}{I_{x}}u_{2} + d_{\phi} \\
 \ddot{\theta} & = \frac{\left( I_{z} - I_{x} \right)}{I_{y}}\dot{\phi}\dot{\psi} + \frac{1}{I_{y}}u_{3} + d_{\theta} \\
 \ddot{\psi} & = \frac{\left( I_{x} - I_{y} \right)}{I_{z}}\dot{\phi}\dot{\theta} + \frac{1}{I_{z}}u_{4} + d_{\psi} \\
 \end{aligned} \right.\ $$

 ------------------------------------------------------------------------------------------------------------------------- -----

In order to simplify disturbance notation in Eq. (2.19), making it more convenient for controller use, disturbance terms resulting in accelerations are summarized as $d_{\phi}$, $d_{θ}$ and $d_{ψ}$.

Therefore, dynamic model of drone without load can be summarized by translational dynamics described in system of equations (2.15) and by rotational dynamics, described in system of equations (2.19).

## Drone with Load Suspended by Cable

Developed dynamic model considers drone as a rigid body and load as a point mass. Load is connected to aircraft's center of mass through a cable with negligible mass and assumed to always be tensioned. Thus, cable's elastic effects and influence of load on drone's rotational dynamics are disregarded, considering only force disturbances. It is emphasized that these considerations are in accordance with most related works found in literature (GUERRERO-SÁNCHEZ et al., 2017; KLAUSEN; FOSSEN; JOHANSEN, 2017; SREENATH; MICHAEL; KUMAR, 2013).

Figure 2.2 illustrates system indicating additional elements to system composed only by drone.

![](media/image9.emf){width="2.72076990376203in" height="2.7102055993000875in"}

Figura 2.2 - Representação esquemática do drone com carga suspensa por cabo, indicando os referenciais, a posição da carga, além da força de tração do cabo e o peso da carga.

The position of the load is represented by $\vec{r}_L = \begin{bmatrix} x_L \\ y_L \\ z_L \end{bmatrix}$ and is related to the position of the drone as:

$$\vec{r}_L = \vec{r} + l\ \vec{p}$$

$l$ corresponds to the length of the cable and $\vec{p}$ consists of the unit vector pointing from the center of gravity of the quadcopter to the load, obtained by means of two consecutive rotations of the vector $-\vec{e}_z$: a rotation of angle $\phi_L$ around $x$ followed by another rotation of angle $\theta_L$ around the $y$ axis:

$$\vec{p} = R_x(\phi_L)R_y(\theta_L)\begin{bmatrix} 0 \\ 0 \\ -1 \end{bmatrix}$$

$$\vec{p} = \begin{bmatrix} -\sin\theta_L \\ \sin(\phi_L)\cos(\theta_L) \\ -\cos(\phi_L)\cos(\theta_L) \end{bmatrix}$$

Along this vector, the force of tension of the cable on the drone is applied ($\vec{T} = T\vec{p}$), assumed to be non-zero at all times. There is also the application of the weight force on the load and on the drone, in addition to the thrust force on the propellers and the drag force.

Thus, by applying the Newton-Euler equations to the drone and to the load, we have:

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------
  $$M\ddot{\overrightarrow{r}} = \mathbf{R}{\overrightarrow{F}}_{b} - Mg{\overrightarrow{e}}_{z} + \overrightarrow{T} + {\overrightarrow{F}}_{d} + {\overrightarrow{D}}_{F}$$   (2.)

  $$m{\ddot{\overrightarrow{r}}}_{L} = - \overrightarrow{T} - mg{\overrightarrow{e}}_{z} + {\overrightarrow{F}}_{d}^{L} + {\overrightarrow{D}}_{F}^{L}$$                        (2.)
  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------

Substituting $\vec{r}_L$ and its derivatives from Eq. (2.20) into Eq. (2.24) and $\vec{T}$ from Eq. (2.24) into Eq. (2.23)[^4], we obtain:

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $$(M + m)\ddot{\vec{r}} + ml\ddot{\vec{p}} + (M + m)g\vec{e}_z = \mathbf{R}\vec{F}_b + \vec{F}_d + \vec{F}_d^L + \vec{D}_F + \vec{D}_F^L$$   (.)

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

$\left\lbrack \vec{F}_d,\vec{F}_d^L \right\rbrack$ and $\left\lbrack \vec{D}_F,\vec{D}_F^L \right\rbrack$ refer to the drag force and force disturbances applied to the drone and the load, respectively. The drag force on the load is also modeled as proportional to the velocity, as was done for the drone (Eq. 2.14), but symmetric in the three directions:

  -------------------------------------------------------------------------- -----
  $$\vec{F}_d^L = - c_L\dot{\vec{r}_L}$$   (.)

  -------------------------------------------------------------------------- -----

Given the assumptions considered for the model, the addition of the load has an effect only on the translational dynamics of the drone, Eq. (2.15), so that the rotational dynamics, Eq. (2.19) remains unchanged.

Developing Eq. (2.24), we obtain a set of three equations in terms of not only the state variables of the drone $\left\lbrack \dot{x},\dot{y},\dot{z},\phi,\theta,\psi,\dot{\phi},\dot{\theta},\dot{\psi} \right\rbrack$, but also the state of the load, described in terms of $\left\lbrack \phi_L,\theta_L,\dot{\phi}_L,\dot{\theta}_L,\ddot{\phi}_L,\ddot{\theta}_L \right\rbrack$. However, in the Newton-Euler formulation, the behavior of these variables is not evident. Thus, the Euler-Lagrange formulation is applied to detail the obtained model.

For this, we define the Lagrangian associated with the translational dynamics, given by the difference between the translational kinetic energy and the potential energy of the system:

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $$L = \frac{M}{2}\left( {\dot{x}}^{2} + {\dot{y}}^{2} + {\dot{z}}^{2} \right) + \frac{m}{2}\left( {\dot{x}}_{L}^{2} + {\dot{y}}_{L}^{2} + {\dot{z}}_{L}^{2} \right) - g\left( Mz + mz_{L} \right)$$   (.)

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

Thus, the Lagrangian is developed by substituting $\vec{r}_L$ and its derivative, obtained by Eq. (2.20), into Eq. (2.27). With this, the dynamic relationships of the system are obtained by applying the Euler-Lagrange equation based on the generalized coordinates $\vec{q} = \left\lbrace x,y,z,\phi_L,\theta_L \right\rbrace$, as shown in Eq. (2.28), with $Q_i$ the generalized forces along each coordinate:

  ------------------------------------------------------------------------------------------------------------------------------------ -----
  $$\frac{\partial}{\partial t}\left( \frac{\partial L}{\partial{\dot{q}}_i} \right) - \frac{\partial L}{\partial q_i} = Q_i$$   (.)

  ------------------------------------------------------------------------------------------------------------------------------------ -----

Analyzing the efforts present in Equations (2.23) and (2.24), the propulsion and drag forces on the drone are already described along $x$, $y$ and $z$. However, the drag force on the load is described in terms of its Cartesian coordinates, and therefore must be converted to the generalized coordinates. For this, the potential function associated with the drag forces is defined as:

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $$P = - \frac{1}{2}\left\lbrace c_x{\dot{x}}^2 + c_y{\dot{y}}^2 + c_z{\dot{z}}^2 + c_L\left( {\dot{x}}_L^2 + {\dot{y}}_L^2 + {\dot{z}}_L^2 \right) \right\rbrace$$   (.)

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

Thus, the generalized effort associated with the translational drag force along each coordinate $i$ is given by:

  ----------------------------------------------------------------- -----
  $$T_i^P = \frac{\partial P}{\partial{\dot{q}}_i}$$          (.)

  ----------------------------------------------------------------- -----

For convenience, the unknown disturbance terms are transferred directly to each coordinate. Thus, the equations of the dynamic of the resulting system are given by:

  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------
  $$\left\{ \begin{aligned}                                                                                                                                                                                                                     (2.)
   & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial\dot{x}} \right) - \frac{\partial L}{\partial x} & = & \left( \cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi \right)u_{1} + \frac{\partial P}{\partial\dot{x}} + D_{F}^{x} \\
   & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial\dot{y}} \right) - \frac{\partial L}{\partial y} & = & \left( \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \right)u_{1} + \frac{\partial P}{\partial\dot{y}} + D_{F}^{y} \\
   & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial\dot{z}} \right) - \frac{\partial L}{\partial z} & = & \left( \cos\phi\cos\theta \right)u_{1} + \frac{\partial P}{\partial\dot{z}} + D_{F}^{z} \\
   & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial{\dot{\phi}}_{L}} \right) - \frac{\partial L}{\partial\phi_{L}} & = & \frac{\partial P}{\partial{\dot{\phi}}_{L}} + D_{\tau}^{\phi_{L}} \\
   & \frac{\partial}{\partial t}\left( \frac{\partial L}{\partial{\dot{\theta}}_{L}} \right) - \frac{\partial L}{\partial\theta_{L}} & = & \frac{\partial P}{\partial{\dot{\theta}}_{L}} + D_{\tau}^{\theta_{L}} \\
  \end{aligned} \right.\ $$

  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------

The system of equations (2.31) can be written in matrix form:

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $$\mathbf{M}\left( \overrightarrow{q} \right)\ddot{\overrightarrow{q}} + \mathbf{C}\left( \overrightarrow{q},\dot{\overrightarrow{q}} \right)\dot{\overrightarrow{q}} + \mathbf{G}\left( \overrightarrow{q} \right) = \mathbf{B}\left( \overrightarrow{q} \right)u_{1} + \mathbf{P}\left( \overrightarrow{q} \right)\dot{\overrightarrow{q}} + \mathbf{D}$$   (.)

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

where:

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------
  $$\mathbf{M}\left( \overrightarrow{q} \right) = \begin{bmatrix}                                                                                                                                                   (2.)
  (M + m) & 0 & 0 & 0 & - ml\ c\theta_{L} \\
  0 & (M + m) & 0 & ml\ c\theta_{L}\ c\theta_{L} & - ml\ s\phi_{L}\ c\theta_{L} \\
  0 & 0 & (M + m) & ml\ s\phi_{L}\ c\theta_{L} & ml\ c\phi_{L}\ s\theta_{L} \\
  0 & ml\ c\theta_{L}\ c\theta_{L} & ml\ s\phi_{L}\ c\theta_{L} & ml^{2}\ {c\theta_{L}}^{2} & 0 \\
   - ml\ c\theta_{L} & - ml\ s\phi_{L}\ c\theta_{L} & ml\ c\phi_{L}\ s\theta_{L} & 0 & ml^{2} \\
  \end{bmatrix}$$

  $$\mathbf{C}\left( \overrightarrow{q},\dot{\overrightarrow{q}} \right) = \left\lbrack \mathbf{Ο}_{\mathbf{5 \times 3}} \middle| \begin{matrix}                                                                    (2.)
  0 & ml\ s\theta_{L}\ {\dot{\theta}}_{L} \\
   - ml\left( s\phi_{L}c\theta_{L}\ {\dot{\phi}}_{L} + c\phi_{L}s\theta_{L}\ {\dot{\theta}}_{L} \right) & - ml\left( s\phi_{L}c\theta_{L}\ {\dot{\theta}}_{L} + c\phi_{L}s\theta_{L}\ {\dot{\phi}}_{L} \right) \\
  ml\left( c\phi_{L}c\theta_{L}\ {\dot{\phi}}_{L} - s\phi_{L}s\theta_{L}\ {\dot{\theta}}_{L} \right) & ml\left( c\phi_{L}c\theta_{L}\ {\dot{\theta}}_{L} - s\phi_{L}s\theta_{L}\ {\dot{\phi}}_{L} \right) \\
   - ml^{2}s\theta_{L}c\theta_{L}{\dot{\theta}}_{L} & - ml^{2}s\theta_{L}c\theta_{L}{\dot{\phi}}_{L} \\
  ml^{2}s\theta_{L}c\theta_{L}{\dot{\phi}}_{L} & 0 \\
  \end{matrix} \right\rbrack$$

  $$\mathbf{G}\left( \overrightarrow{q} \right) = \left\lbrack \begin{array}{r}                                                                                                                                     (2.)
  0 \\
  0 \\
  (M + m)g \\
  mgl\sin\phi_{L}\cos\theta_{L} \\
  mgl\cos\phi_{L}\sin\theta_{L} \\
  \end{array} \right\rbrack$$

  $$\mathbf{B}\left( \overrightarrow{q} \right) = \left\lbrack \begin{array}{r}                                                                                                                                     (2.)
  u_{x} \\
  u_{y} \\
  u_{z} \\
  0 \\
  0 \\
  \end{array} \right\rbrack = \left\lbrack \begin{array}{r}
  \cos\phi\sin\theta\cos\psi + \sin\phi\sin\psi \\
  \cos\phi\sin\theta\sin\psi - \sin\phi\cos\psi \\
  \cos\phi\cos\theta \\
  0 \\
  0 \\
  \end{array} \right\rbrack$$

  $$\mathbf{P}\left( \overrightarrow{q} \right) = \begin{bmatrix}                                                                                                                                                   (.)
   - \left( C_{x} + C_{L} \right) & 0 & 0 & 0 & C_{L}lc\theta_{L} \\
  0 & - \left( C_{y} + C_{L} \right) & 0 & - C_{L}lc\phi_{L}c\theta_{L} & C_{L}ls\phi_{L}s\theta_{L} \\
  0 & 0 & - \left( C_{x} + C_{L} \right) & - C_{L}ls\phi_{L}c\theta_{L} & - C_{L}lc\phi_{L}s\theta_{L} \\
  0 & - C_{L}lc\phi_{L}c\theta_{L} & - C_{L}ls\phi_{L}c\theta_{L} & - C_{L}l^{2}c\theta_{L}^{2} & 0 \\
  C_{L}lc\theta_{L} & C_{L}ls\phi_{L}s\theta_{L} & - C_{L}lc\phi_{L}s\theta_{L} & 0 & - C_{L}l^{2} \\
  \end{bmatrix}$$
  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------

It is observed that $\mathbf{M}\left( \overrightarrow{q} \right)$ is a positive definite matrix, that is, it is symmetric and the terms on the main diagonal are strictly positive except for when $\theta_{L} = \pm \frac{\pi}{2}$. With this, we take as a restriction $\left| \theta_{L} \right| < \frac{\pi}{2}$. Given this scenario, it is possible to isolate the acceleration term $\ddot{\overrightarrow{q}}$ from Eq. (2.32):

  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------
  $$\ddot{\overrightarrow{q}} = \mathbf{M}^{- 1}\left( \overrightarrow{q} \right)\left\lbrack - \mathbf{C}\left( \overrightarrow{q},\dot{\overrightarrow{q}} \right)\dot{\overrightarrow{q}} - \mathbf{G}\left( \overrightarrow{q} \right) + \mathbf{B}\left( \overrightarrow{q} \right)u + \mathbf{P}\left( \overrightarrow{q} \right)\dot{\overrightarrow{q}} + \mathbf{D} \right\rbrack$$   (2.)

  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------

For control purposes, we choose to include the drag term as a disturbance, so that the equations resulting from the development of Eq. (2.38) with this consideration are given by:

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $$\left\{ \begin{aligned}                                                                                                                                                                                  (.)
   & \ddot{x} & = & f_{x}\left( \theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L} \right) & + & b_{x}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right)\  & u_{1} & + d_{x} \\
   & \ddot{y} & = & f_{y}\left( \phi_{L},\theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L} \right) & + & b_{y}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right) & u_{1} & + d_{y} \\
   & \ddot{z} & = & f_{z}\left( \phi_{L},\theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L} \right) & + & b_{z}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right)\  & u_{1} & + d_{z} \\
   & {\ddot{\phi}}_{L} & = & f_{\phi_{L}}\left( \theta_{L},{\dot{\phi}}_{L},{\dot{\theta}}_{L} \right) & + & b_{\phi_{L}}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right)\  & u_{1} & + d_{\phi_{L}} \\
   & {\ddot{\theta}}_{L} & = & f_{\theta_{L}}\left( \theta_{L},{\dot{\phi}}_{L} \right) & + & b_{\theta_{L}}\left( \phi,\theta,\psi,\phi_{L},\theta_{L} \right)\  & u_{1} & + d_{\theta_{L}} \\
  \end{aligned} \right.\ $$

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

The last term of each equation in the system (2.39) refers to the effect of disturbances (including drag) on the accelerations. Here is the expansion of each term in Eq. (2.39):

  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $$f_{x} = - \frac{ml\sin\theta_{L}}{(M + m)}\left( \cos^{2}\theta_{L}{\dot{\phi}}_{L}^{2} + {\dot{\theta}}_{L}^{2} \right)$$                                                                                          (.)

  $$b_{x} = \frac{m}{M(M + m)}\left\lbrack s\theta_{L}c\theta_{L}\left( u_{y}s\phi_{L} - u_{z}c\phi_{L}\  \right) + u_{x}\left( \frac{M}{m} + {c\theta_{L}}^{2} \right) \right\rbrack$$                                 (.)

  $$f_{y} = \frac{ml\sin\phi_{L}\cos\theta_{L}}{(M + m)}\left( \cos^{2}\theta_{L}{\dot{\phi}}_{L}^{2} + {\dot{\theta}}_{L}^{2} \right)$$                                                                                (.)

  $$b_{y} = \frac{m}{M(M + m)}\left\lbrack s\phi_{L}c\theta_{L}\left( u_{x}s\theta_{L} + u_{z}c\phi_{L}c\theta_{L} \right) + u_{y}\left( \frac{M}{m} + 1 - {s\phi_{L}}^{2}{c\theta_{L}}^{2} \right) \right\rbrack$$     (.)

  $$f_{z} = - \frac{ml\cos\phi_{L}\cos\theta_{L}}{(M + m)}\left( \cos^{2}\theta_{L}{\dot{\phi}}_{L}^{2} + {\dot{\theta}}_{L}^{2} \right) - g$$                                                                          (.)

  $$b_{z} = \frac{m}{M(M + m)}\left\lbrack c\phi_{L}c\theta_{L}\left( - u_{x}s\theta_{L} + u_{y}s\phi_{L}c\theta_{L} \right) + u_{z}\left( \frac{M}{m} + 1 - {c\phi_{L}}^{2}{c\theta_{L}}^{2} \right) \right\rbrack$$   (.)

  $$f_{\phi_{L}} = 2{tg}\theta_{L}{\dot{\phi}}_{L}{\dot{\theta}}_{L}$$                                                                                                                                                  (.)

  $$b_{\phi_{L}} = - \frac{\left( u_{y}\cos\phi_{L} + u_{z}\sin\phi_{L} \right)}{Ml\cos\theta_{L}}$$                                                                                                                    (.)
  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

# CONTROL

This chapter presents the control solution developed for the quadrotor system with cable-suspended load described in Chapter 2. As introduced previously, it is desired to control the position of the aircraft and at the same time keep the load swing reduced. To do this, a sliding mode controller is developed that controls the position of the quadrotor considering the coupled dynamics of the load, but does not aim at its stabilization. The task of reducing the load swing is left to the trajectory generator described in the next chapter.

First, a qualitative analysis of the dynamic model and the control problem is presented, evaluating the system's actuation characteristics and making reference to other relevant control solutions. Then, the controller is detailed, presenting from the formulation and stability analysis to the verification of the system's behavior in simulation.

## Introduction

### Actuation Characteristics

Analyzing the dynamic equations of the quadrotor with load, it is verified that the system has eight degrees of freedom ($\overrightarrow{q} = \left\lbrack x,y,z,\phi,\theta,\psi,\phi_{L},\theta_{L} \right\rbrack^{T}$) for four independent control inputs ($\overrightarrow{u} = \left\lbrack u_{1},u_{2},u_{3},u_{4} \right\rbrack^{T}$). This difference characterizes the system as underactuated, which means that the control action is not able to act on all degrees of freedom independently. Compared to the system composed only of the quadrotor, the proposed challenge adds two un-actuated degrees of freedom to the problem.

It is verified that $u_{2}$, $u_{3}$ and $u_{4}$, the torque efforts on the quadrotor, explicitly appear in the equations of the variables that describe its orientation $\phi$, $\theta$ and $\psi$, as presented in Eq. (2.19). This means that, in isolation, it is possible to control the three variables through these three inputs. However, $\phi$ and $\theta$ also configure the orientation of the thrust force, which is responsible for causing the aircraft to move. Therefore, the input signals $u_{2}$ and $u_{3}$ indirectly affect the position $\lbrack x,y,z\rbrack$. This influence can also be verified by observing the presence of the angles in the equations of the aircraft's translational dynamics according to Eq. (2.15).

The thrust force $u_{1}$, on the other hand, is explicitly present in the equations of the variables that describe the position of the quadrotor $\lbrack x,y,z\rbrack$ and of the load $\left\lbrack \phi_{L},\theta_{L} \right\rbrack$, as presented in Eq. (2.39). However, $u_{1}$ points vertically at the equilibrium point, around which it is desired to keep the system's variables, exerting influence only on the acceleration $\ddot{z}$ in this condition. That is, in the conditions around the equilibrium, the thrust exerts control mainly along $z$.

These actuation characteristics motivated researchers in the field of multi-rotor aircraft control to develop cascade control solutions as illustrated in the figure below.

[Image of a cascade control system for a quadrotor with cable-suspended load]

In the cascade control system, the upper layer is responsible for controlling the position and orientation of the quadrotor, while the lower layer is responsible for controlling the load swing. The upper layer uses a sliding mode controller, while the lower layer uses a PID controller.

The upper layer sliding mode controller is designed to ensure that the quadrotor reaches its desired position and orientation. The lower layer PID controller is designed to reduce the load swing.

The cascade control system was implemented on a real quadrotor and was tested in a variety of conditions. The test results showed that the control system was able to successfully control the position and orientation of the quadrotor, and it was also able to reduce the load swing.

The cascade control system is a promising solution to the problem of controlling quadrotors with cable-suspended loads. The system is able to successfully control the position, orientation and load swing, and it has been implemented on a real quadrotor and tested in a variety of conditions.

![](media/image10.emf){width="5.5019969378827644in" height="1.9985203412073491in"}

Figure 3.1 -- Cascade control structure for multirotors. Adapted from MO; FARID (2018).

Basically, the solution presents a cascade position controller to an attitude (or orientation) controller. Given the desired positions, the position controller generates the signal $u_{1}$ and reference values for the roll and pitch angles ($\phi_{d}$ and $\theta_{d}$), which, together with the desired orientation $\psi_{d}$, feed the attitude controller that generates the signals $u_{2}$, $u_{3}$, and $u_{4}$. The signal $u_{4}$ is determined based on the desired yaw angle, defined externally, and the displacement in the $x$ and $y$ directions is achieved through the action of $u_{2}$ and $u_{3}$, which direct the propulsion force $u_{1}$ in the direction to reduce the displacement error, as illustrated in Figure 3.2.

Figure 3.2 - Illustration of the effect of $u_{2}$ and $u_{3}$ on the drone's horizontal displacement.{width="3.6449311023622046in" height="2.132752624671916in"}

As illustrated in Figure 3.2, the action of $u_{2}$ directs the propulsion in the direction to move the aircraft along ${\overrightarrow{e}}{y}^{b}$, while the action of $u{3}$ has an indirect influence on the displacement along ${\overrightarrow{e}}_{x}^{b}$.

Sliding Mode Control (SMC)
Basic Concepts
SMC is a nonlinear control technique robust to external disturbances and parameter uncertainties, whose implementation can be summarized as follows:

Define the so-called sliding variables, which are functions of the system variables carefully designed such that, when they become zero, the system exhibits stable behavior;

Design the input signals to drive the sliding variables to zero and maintain them in this condition.

For example, given a nonlinear system written in the form:

  ----------------------------------------------------------------- -----
  $$\ddot{x} = f\left( x,\dot{x},u \right)$$                        (.)

  ----------------------------------------------------------------- -----

sendo $x$ e $\dot{x}$ o estado do sistema e $u$ o sinal de entrada, pode-se definir, por exemplo, uma variável deslizante como uma combinação linear da variável de estado e a sua derivada:

  ----------------------------------------------------------------- ------
  $$s = \dot{x} + \lambda x,\ \ \lambda > 0$$                       (3.)

  ----------------------------------------------------------------- ------

Observa-se que, quando $s = 0$, tem-se:

  ----------------------------------------------------------------- -----
  $${\dot{x} = - \lambda x                                          (.)
  }{x = x(0)e^{- \lambda t}
  }{\dot{x} = - \lambda x(0)e^{- \lambda t}}$$

  ----------------------------------------------------------------- -----

Nesta situação, $x$ e $\dot{x}$ convergem para zero assintoticamente. Assim, quando o sinal de controle é definido de forma adequada, o retrato de fase do sistema para quando se define as variáveis deslizantes como feito no exemplo da Eq. (3.49) se assemelha ao mostrado na Figura 3.3.

![](media/image12.emf){width="3.589529746281715in" height="2.4246194225721784in"}

Figura 3.3 - Retrato de fase característico de um sistema comandado por um controlador por modos deslizantes para variável deslizante linear (Adaptado de GHAZALI et al., 2011).

Como mostra a Figura 3.3, o estado em que $s = 0$ corresponde à reta indicada no retrato de fase. Este estado é denominado superfície ou modo deslizante. Considerando $s \neq 0$ no estado inicial, o sistema controlado primeiramente é conduzido até a superfície deslizante, executando a chamada "fase de aproximação", e então segue deslizando ao longo da superfície até o ponto de equilíbrio, executando a chamada "fase de deslizamento".

Existem diversas soluções em torno deste conceito. De modo geral, as versões de CMD diferenciam-se pela forma com que determinam as variáveis deslizantes e pela estratégia que usam para realizar as fases de aproximação e deslizamento. Para conhecer mais sobre a técnica e as suas variações, recomenda-se as fontes (QIAN; YI, 2015; SHTESSEL et al., 2013; UTKIN; GULDNER; SHI, 2009).

#### CMD Aplicado a Drones com Carga Suspensa por Cabo

Foram encontrados dois trabalhos na literatura que aplicam controle por modos deslizantes a este sistema (KUI et al., 2017; ZHOU et al., 2016). Basicamente, eles utilizam o mesmo princípio de atuação comumente utilizado para drones como mostra a Figura 3.1 (Seção 3.1.1).

(KUI et al., 2017) assumem a existência de uma força aplicada à aeronave com componentes independentes ao longo de cada eixo do sistema de coordenadas inercial. A partir de análise geométrica deste vetor através da Eq. (2.36), é possível determinar qual deve ser a força de propulsão e os ângulos de rolagem e arfagem ideais para produzir esta entrada. Dessa forma, é possível tratar o sistema como totalmente atuado, de modo que cada componente da força virtual é projetada para controlar por modos deslizantes a posição da aeronave ao longo de cada eixo. A força de propulsão resultante da transformação geométrica é passada adiante, enquanto os ângulos de rolagem e arfagem calculadas são passados como referência para um controlador de atitude que também aplica controle por modos deslizantes clássico para cada eixo, resultando nos comandos de torque a serem enviados ao sistema de atuação da aeronave.

Observa-se que, nesta abordagem, a determinação dos ângulos de arfagem e rolagem fica totalmente em função da saída do controlador de posição. Esta estrutura possibilita utilizar valores de referência determinados externamente, como é feito neste trabalho com o gerador de trajetórias.

Também vale ressaltar que os artigos não deixam claro a interdependência entre as acelerações do drone e da carga na equação dinâmica. Considerando o modelo dinâmico em função das coordenadas generalizadas, Eq. (2.39), as "forças virtuais" mencionadas não se manifestam de forma exata de se traduzir em um estado de orientação desejado como acontece nas equações do drone sem carga.

Em contrapartida, o controlador desenvolvido neste trabalho leva em conta a característica de sub-atuação do sistema de forma explícita, sem usar o recurso da força virtual; permite a determinação externa de referências para os ângulos de rolagem e arfagem; e consideram o modelo dinâmico completo, considerando todas as interações mútuas entre *drone* e carga.

## Estratégia de Controle

Como descrito no Capítulo 2, as acelerações do sistema são descritas pelos sistemas de equações (2.39) e (2.19), que representam as dinâmicas de translação e rotação respectivamente. Para o intuito de realizar o controle, o sistema é subdividido em dois, um totalmente atuado, formado pelas variáveis $z$ e $\psi$, e outro sub-atuado, formado pelas variáveis $x$, $y$, $\phi$ e $\theta$:

  ------------------------------------------------------------------------------------- -----
  $$\left\{ \begin{aligned}                                                             (.)
  \ddot{z} & = f_{x} + b_{x}u_{1} + d_{z} \\
  \ddot{\psi} & = f_{\psi} + b_{\psi}u_{4} + d_{\psi} \\
  \end{aligned} \right.\ \ \ \ \ \ \ \ \ \ \ \text{(Subistema\ totalmente\ atuado)}$$

  $$\left\{ \begin{aligned}                                                             (.)
  \ddot{x} & = f_{x} + b_{x}u_{1} + d_{x} \\
  \ddot{y} & = f_{y} + b_{y}u_{1} + d_{y} \\
   \\
  \ddot{\phi} & = f_{\phi} + b_{\phi}u_{2} + d_{\psi} \\
  \ddot{\theta} & = f_{\theta} + b_{\theta}u_{3} + d_{\theta} \\
  \end{aligned} \right.\ \ \ \ \ \ \ \ \ \ \ (\text{Subsistema\ sub-atuado)}$$
  ------------------------------------------------------------------------------------- -----

Esta organização se baseia em trabalhos de controle por modos deslizantes de quadcópteros (XIONG; ZHENG, 2014, 2014; XU; ÖZGÜNER, 2008). O agrupamento das variáveis $\phi$ e $\theta$ com $x$ e $y$ é motivado pela lógica apresentada na Seção 3.1.1 de que $\phi$ e $\theta$ determinam a orientação de $u_{1}$ no plano $xy$ e portanto exercem influência na movimentação horizontal da aeronave. Assim, deseja-se determinar $u_{3}$ e $u_{4}$ de modo a controlar a posição $x$ e $y$ da aeronave ao mesmo tempo que estabilizar $\phi$ e $\ \theta$ no ponto de equilíbrio. Também vale ressaltar que esta estrutura é compatível com métodos genéricos de controle por modos deslizantes aplicados a sistemas sub-atuados tomadas como referência para este trabalho (ASHRAFIUON; ERWIN, 2004, 2004; SANKARANARAYANAN; MAHINDRAKAR, 2009).

O estado da carga, que está diretamente associado às variáveis $\phi_{L}$ e $\theta_{L}$, não é controlado diretamente. Sua estabilização é atingida por meio da geração de trajetórias adequadas, como será descrito na Seção 4.

Neste contexto, a solução desenvolvida é composta por dois grupos de controladores por modos deslizantes em cascata, como ilustra a Figura 3.4.

![](media/image13.emf){width="5.246268591426071in" height="1.966836176727909in"}

Figura 3.4 - Estrutura de controle do sistema drone com carga suspensa por cabo.

Como ilustrado na Figura 3.4, o primeiro CMD se encarrega de controlar o subsistema totalmente atuado com base nos valores desejados para altitude e ângulo de guinada até suas segundas derivadas, determinando $u_{1}$ e $u_{4}$. O segundo controlador comanda o subsistema sub-atuado com base nas referências de posição horizontal e ângulos de rolagem e arfagem até a segunda derivada, além de $u_{1}$ obtido anteriormente, gerando os sinais de controle restantes: $u_{2}$ e $u_{3}$.

Uma característica importante de se observar na arquitetura é a de que o controlador do sistema sub-atuado recebe referências externas do ângulo de rolagem e arfagem. Visto o acoplamento existente entre a atitude da aeronave e o direcionamento da força de propulsão, estes valores de entrada devem ser definidos em concordância com as referências de posição $x$ e $y$ ou mantidos nulos (condição de equilíbrio). Embora esta configuração imponha restrições na entrada dos sistema, ela permite a aplicação de geradores de trajetória que levam em conta a atitude da aeronave explicitamente (como é feito neste trabalho), diferentemente da estrutura em cascata apresentada na Seção 3.1.1, em que os ângulos $\phi_{d}$ e $\theta_{d}$ já são definidos internamente pelo controlador.

### CMD do Subsistema Totalmente Atuado ($z,\psi$)

Para realizar o controle de altitude e guinada da aeronave, aplica-se a variação clássica de controlador por modos deslizantes aplicado a sistemas não lineares, reproduzindo o que já foi feito por outros autores (XIONG; ZHENG, 2014; ZHENG; XIONG; LUO, 2014).

#### Dedução do Controlador

Primeiramente, define-se as variáveis deslizantes:

  --------------------------------------------------------------------------------------------------------- -----
  $$s_{1} = \left( {\dot{z}}_{d} - \dot{z} \right) + \lambda_{z}\left( z_{d} - z \right)$$                  (.)

  $$s_{2} = \left( {\dot{\psi}}_{d} - \dot{\psi} \right) + \lambda_{\psi}\left( \psi_{d} - \psi \right)$$   (.)
  --------------------------------------------------------------------------------------------------------- -----

O objetivo do CMD é conduzir estas variáveis até zero para que, uma vez nesta condição, as variáveis do subsistema se estabilizem de modo que $z \rightarrow z_{d}$ e $\psi \rightarrow \psi_{d}$. De fato, quando $s_{1} = 0$, tem-se:

  ---------------------------------------------------------------------------------------------------------- -----
  $${\left( {\dot{z}}_{d} - \dot{z} \right) = - \lambda_{z}\left( z_{d} - z \right)                          (.)
  }{\left( z_{d} - z \right) = \left( z_{d} - z \right)(0)e^{- \lambda_{z}t}
  }{\left( {\dot{z}}_{d} - \dot{z} \right) = - \lambda_{z}\left( z_{d} - z \right)(0)e^{- \lambda_{z}t}}$$

  ---------------------------------------------------------------------------------------------------------- -----

Dessa forma, $\left( z_{d} - z \right) \rightarrow 0$ e $\left( {\dot{z}}_{d} - \dot{z} \right) \rightarrow 0$ assintoticamente. O mesmo acontece para $s_{2} = 0$, em que $\left( \psi_{d} - \psi \right) \rightarrow 0$ e $\left( {\dot{\psi}}_{d} - \dot{\psi} \right) \rightarrow 0$ assintoticamente.

O próximo passo consiste em definir as entradas do sistema que realizem a regularização das variáveis deslizantes. Para isso, primeiramente, extrai-se as derivadas de $s_{1}$ e $s_{2}$:

  --------------------------------------------------------------------------------------------------------------------------------- ------
  $${\dot{s}}_{1} = \left( {\ddot{z}}_{d} - \ddot{z} \right) + \lambda_{z}\left( z_{d} - z \right)$$                                (.)

  $${\dot{s}}_{2} = \left( {\ddot{\psi}}_{d} - \ddot{\psi} \right) + \lambda_{\psi}\left( {\dot{\psi}}_{d} - \dot{\psi} \right)$$   (3.)
  --------------------------------------------------------------------------------------------------------------------------------- ------

Observa-se que ${\dot{s}}_{1}$ e ${\dot{s}}_{2}$ contêm as acelerações $\ddot{z}$ e $\ddot{\psi}$, estas definidas em função das entradas $u_{1}$ e $u_{4}$ segundo o sistema de equações (3.51). Assim, é possível conduzir ${\dot{s}}_{1}$ e ${\dot{s}}_{2}$ de forma a estabilizar $s_{1}$ e $s_{2}$ como desejado. Com isso, deseja-se que:

  ------------------------------------------------------------------------------------------------------------ ---------
  $${\dot{s}}_{1} = - \kappa_{1}s_{1} - \eta_{1}{sign}{\left( s_{1} \right),\ \ \kappa_{1},\eta_{1} > 0\ }$$   (.)

  $${\dot{s}}_{2} = - \kappa_{2}s_{2} - \eta_{2}{sign}\left( s_{2} \right),\ \ \kappa_{2},\eta_{2} > 0$$       (3.)
  ------------------------------------------------------------------------------------------------------------ ---------

Onde:

  ----------------------------------------------------------------- -----
  $${sign}(x) = \left\{ \begin{aligned}                             (.)
  1,\ \ \  & se\ x \geq 0 \\
   - 1,\ \ \  & se\ x < 0 \\
  \end{aligned} \right.\ $$

  ----------------------------------------------------------------- -----

Substituindo (3.56) e (3.57) em (3.58) e (3.59) dadas as acelerações escritas em função das entradas conforme a Eq. (3.51) sem os distúrbios, encontra-se:

  -------------------------------------------------------------------------------------------------------------------- -----
  $$u_{1} = \frac{{\ddot{z}}_{d} - f_{z} + \kappa_{1}s_{1} + \eta_{1}{sign}\left( s_{1} \right)}{b_{z}}$$              (.)

  $$u_{4} = \frac{{\ddot{\psi}}_{d} - f_{\psi} + \kappa_{2}s_{2} + \eta_{2}{sign}\left( s_{2} \right)\ }{b_{\psi}}$$   (.)
  -------------------------------------------------------------------------------------------------------------------- -----

Um ponto a ser ressaltado é o de que o sinal de controle resultante é descontínuo, visto a presença da função ${sign}(s)$. Esta característica provoca uma entrada ruidosa que leva ao fenômeno chamado *chattering*. A Figura 3.5 apresenta o retrato de fase característico de um sistema controlado por modos deslizantes apresentando este efeito.

![](media/image14.emf){width="2.952830271216098in" height="2.09375in"}

Figura 3.5 -Ilustração do efeito de *chattering* durante a fase de deslizamento (Adaptado de HOSSAIN et al., 2017).

Como apresentado na Figura 3.5, o termo descontínuo na entrada faz com que o sistema faça pequenos saltos em torno da superfície deslizante, caracterizando o fenômeno de *chattering*. Isso ocorre durante a fase de deslizamento, pois é quando a variável deslizante oscila em torno de zero, fazendo com que o sinal de controle chaveie entre $+ \eta$ e $- \eta$ devido ao termo ${sign}(s)$. Com isso, o sistema pode responder com oscilações indesejadas, podendo até leva-lo à instabilidade.

Esta característica é um dos pontos mais desvantajosos do controlador por modos deslizantes, porém existem formas de atenuá-lo. A forma mais simples é aproximar a função ${sign}(s)$ para uma função contínua aproximada, como a *sigmoide* e a tangente hiperbólica. Fazendo isso, porém, o controlador deixa de ser idealmente robusto, embora se consiga produzir bons resultados na prática (SHTESSEL et al., 2013). Neste trabalho, toma-se a aproximação:

  ----------------------------------------------------------------------------------------------------- ---------
  $${sign}\left( s_{1} \right) \approx \tanh\left( \epsilon_{1}s_{1} \right),\ \ \epsilon_{1} \gg 0$$   (.)

  $${sign}\left( s_{2} \right) \approx \tanh\left( \epsilon_{2}s_{2} \right),\ \ \epsilon_{2} \gg 0$$   (.)
  ----------------------------------------------------------------------------------------------------- ---------

#### Análise de Estabilidade

A motivação por trás da definição imposta para ${\dot{s}}_{1}$ e ${\dot{s}}_{2}$ se dá com base na teoria de estabilidade de *Lyapunov*, que estabelece o critério de que, dado um sistema escrito na forma $\dot{x} = f(x)$ com $x = 0$ como ponto de equilíbrio, uma função $V(x):\mathbb{R}^{n}\mathbb{\rightarrow R}$ é chamada função de *Lyapunov* candidata e o sistema é estável no sentido de *Lyapunov* se:

a)  $V(x) = 0$ se e somente se $x = 0$;

b)  $V(x) > 0$ para todo $x \neq 0$;

c)  $\dot{V}(x) \leq 0$ $\rightarrow$ o sistema é localmente estável;

d)  $\dot{V}(x) < 0$ para todo $x \neq 0 \rightarrow$ o sistema é assintoticamente estável;

Uma função de *Lyapunov* pode ser interpretada como uma função de energia do sistema que é sempre dissipada ao longo do tempo até a nulidade, quando o sistema atinge o ponto de equilíbrio (QIAN; YI, 2015). Com isso, define-se as seguintes funções de *Lyapunov* candidatas:

  ----------------------------------------------------------------- -----
  $$V_{1}\left( s_{1} \right) = \frac{1}{2}s_{1}^{2}$$              (.)

  $$V_{2}\left( s_{2} \right) = \frac{1}{2}s_{2}^{2}$$              (.)
  ----------------------------------------------------------------- -----

Nota-se que as condições (a) $V(0) = 0$ e (b) $V(x) > 0\ (x \neq 0$) são satisfeitas, visto que $V_{1}$ e $V_{2}$ são funções quadráticas. Derivando-se $V_{1}$, tem-se:

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $${{\dot{V}}_{1} = s_{1}{\dot{\mathbf{s}}}_{\mathbf{1}}                                                                                                                                                                                                                               (.)
  }{{\dot{V}}_{1} = s_{1}\left\lbrack \left( {\ddot{z}}_{d} - \ddot{\mathbf{z}} \right) + \lambda_{z}\left( {\dot{z}}_{d} - \dot{z} \right) \right\rbrack
  }{{\dot{V}}_{1} = s_{1}\left\lbrack \left( {\ddot{z}}_{d} - f_{z} - b_{z}\mathbf{u}_{\mathbf{1}} - d_{z} \right) + \lambda_{z}\left( {\dot{z}}_{d} - \dot{z} \right) \right\rbrack = s_{1}\left\lbrack - \kappa_{1}s_{1} - \eta_{1}{sign}\left( s_{1} \right) - d_{z} \right\rbrack
  }{{\dot{V}}_{1} = - \kappa_{1}s_{1}^{2} - \eta_{1}\left| s_{1} \right| - \mathbf{d}_{\mathbf{z}}\mathbf{s}_{\mathbf{1}} \leq - \kappa_{1}s_{1}^{2} - \eta_{1}\left| s_{1} \right| + \left| d_{z} \right|\left| s_{1} \right|}$$

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

Tomando-se $D_{z} = \max\left( \left| d_{z} \right| \right)$:

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $${{\dot{V}}_{1} \leq - \kappa_{1}s_{1}^{2} - \eta_{1}\left| s_{1} \right| + \left| \mathbf{d}_{\mathbf{z}} \right|\left| \mathbf{s}_{\mathbf{1}} \right| \leq - \kappa_{1}s_{1}^{2} - \eta_{1}\left| s_{1} \right| + D_{z}\left| s_{1} \right|   (.)
  }{{\dot{V}}_{1} \leq - \kappa_{1}s_{1}^{2} + \left( D_{z} - \eta_{1} \right)\left| s_{1} \right|}$$

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

Observa-se que ${\dot{V}}_{1} < 0$ para $\kappa_{1} \geq 0$ e $\eta_{1} > D_{z}$. Similarmente, ${\dot{V}}_{2} < 0$ para $\kappa_{2} \geq 0$ e $\eta_{2} \geq D_{\psi}$, sendo $D_{\psi} = \max\left( \left| d_{\psi} \right| \right)$. Portanto, sob estas condições, o subsistema é assintoticamente estável no sentido de *Lyapunov* e robusto contra distúrbios que não ultrapassam os limites definidos.

Uma observação relevante é a de que o termo associado às constantes $\kappa_{1}$ e $\kappa_{2}$ não são estritamente necessários para garantir a convergência do sistema. A Figura 3.6 apresenta um exemplo de comportamento de uma variável deslizante, ilustrando a contribuição de cada termo.

![](media/image15.emf){width="3.8018077427821524in" height="2.850748031496063in"}

Figura 3.6 - Exemplo de convergência de uma variável deslizante $s$ com decaimento exponencial, constante e combinado.

A Figura 3.6 mostra que $s$ apresenta decaimento linear para $\dot{s} = - \eta|s|$ e exponencial para $\dot{s} = - \kappa s$. A primeira parcela é responsável por garantir convergência em tempo finito e superar distúrbios externos (como demonstrado anteriormente), enquanto o principal papel da segunda é acelerar a convergência quando distante de zero.

### CMD do Subsistema Sub-atuado

Existem diversas variações de controle por modos deslizantes para controlar sistemas sub-atuado na literatura (ASHRAFIUON; ERWIN, 2004, 2008; SANKARANARAYANAN; MAHINDRAKAR, 2009; WANG et al., 2004; WANG; LIU; YI, 2007; XU; ÖZGÜNER, 2008). Basicamente, a estratégia para estender o uso da técnica para esse tipo de sistema consiste em definir variáveis deslizantes que combinam componentes sob influência direta dos sinais de controle com variáveis sem atuação direta e então determinar a entrada de modo a garantir a convergência destas superfícies e das variáveis do sistema durante a fase de deslizamento.

Especificamente, a técnica utilizada neste trabalho se baseia em (ZHENG; XIONG; LUO, 2014) e (XIONG; ZHENG, 2014), que realizam o controle de um quadcóptero sem carga. Ressalta-se que, em comparação a estes trabalhos, a solução desenvolvida, além de adicionar o efeito da carga suspensa ao modelo, inova ao propor definições alternativas para as variáveis deslizantes e os parâmetros de controle.

#### Dedução do Controlador

A intuição por trás do controlador é semelhante ao realizado no controle de quadcópteros, como ilustrado anteriormente na Figura 3.2: $u_{2}$ é direcionado a reduzir o erro ao longo de ${\overrightarrow{e}}_{y}^{b}$, enquanto $u_{3}$ atua no sentido de reduzir o erro ao longo de ${\overrightarrow{e}}_{x}^{b}$. Assim, calcula-se o erro de posição da aeronave projetado sobre o plano $xy$ como:

  ----------------------------------------------------------------- ------
  $$\left\{ \begin{array}{r}                                        (3.)
  {\widetilde{x}}_{b} \\
  {\widetilde{y}}_{b} \\
  \end{array} \right\} = \begin{bmatrix}
  \cos\psi & \sin\psi \\
   - \sin\psi & \cos\psi \\
  \end{bmatrix}\left\{ \begin{array}{r}
  x_{d} - x \\
  y_{d} - y \\
  \end{array} \right\}$$

  ----------------------------------------------------------------- ------

Na Eq. (3.69), assume-se $\psi$ como invariante no tempo, ou seja, $\dot{\psi} = \ddot{\psi} = 0$. Assim, as derivadas da Eq. (3.69) são dadas por:

  ----------------------------------------------------------------- ------
  $$\left\{ \begin{array}{r}                                        (.)
  {\dot{\widetilde{x}}}_{b} \\
  {\dot{\widetilde{y}}}_{b} \\
  \end{array} \right\} = \begin{bmatrix}
  \cos\psi & \sin\psi \\
   - \sin\psi & \cos\psi \\
  \end{bmatrix}\left\{ \begin{array}{r}
  {\dot{x}}_{d} - \dot{x} \\
  {\dot{y}}_{d} - \dot{y} \\
  \end{array} \right\}$$

  $$\left\{ \begin{array}{r}                                        (3.)
  {\ddot{\widetilde{x}}}_{b} \\
  {\ddot{\widetilde{y}}}_{b} \\
  \end{array} \right\} = \begin{bmatrix}
  \cos\psi & \sin\psi \\
   - \sin\psi & \cos\psi \\
  \end{bmatrix}\left\{ \begin{array}{r}
  {\ddot{x}}_{d} - \ddot{x} \\
  {\ddot{y}}_{d} - \ddot{y} \\
  \end{array} \right\}$$
  ----------------------------------------------------------------- ------

Esta consideração se demonstra razoável, visto que as condições de controle de $\psi$ são favoráveis para atingir convergência em curto prazo (XIONG; ZHENG, 2014).

Com isso, define-se as variáveis deslizantes e suas derivadas como:

  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------
  $$s_{3} = \lambda_{1}{\dot{\widetilde{x}}}_{b} + \lambda_{2}{\widetilde{x}}_{b} + \lambda_{3}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \lambda_{4}\left( \theta_{d} - \theta \right)$$                                  (3.)

  $$s_{4} = \lambda_{5}{\dot{\widetilde{y}}}_{b} + \lambda_{6}{\widetilde{y}}_{b} + \lambda_{7}\left( {\dot{\phi}}_{d} - \dot{\phi} \right) + \lambda_{8}\left( \phi_{d} - \phi \right)$$                                          (.)

  $${\dot{s}}_{3} = \lambda_{1}{\ddot{\widetilde{x}}}_{b} + \lambda_{2}{\dot{\widetilde{x}}}_{b} + \lambda_{3}\left( {\ddot{\theta}}_{d} - \ddot{\theta} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right)$$   (3.)

  $${\dot{s}}_{4} = \lambda_{5}{\ddot{\widetilde{y}}}_{b} + \lambda_{6}{\dot{\widetilde{y}}}_{b} + \lambda_{7}\left( {\ddot{\phi}}_{d} - \ddot{\phi} \right) + \lambda_{8}\left( {\dot{\phi}}_{d} - \dot{\phi} \right)$$           (3.)
  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------

Observa-se que as variáveis deslizantes são definidas como a combinação linear dos erros das variáveis do sistema. Diferentemente das variáveis deslizantes definidas para o subsistema totalmente atuado, a condição de estabilidade para quando $s_{3} = 0$ e $s_{4} = 0$ não é trivial. Neste momento, assume-se que a estabilidade será garantida e na Seção 3.2.2.3 é feita uma análise de estabilidade para a condição de deslizamento.

Observa-se que as equações que descrevem as derivadas das variáveis deslizantes contêm as acelerações $\ddot{\phi}$ e $\ddot{\theta}$, que são definidas em função das entradas $u_{2}$ e $u_{3}$ segundo o sistema de equações (3.52). Assim, espera-se ser possível conduzir ${\dot{s}}_{3}$ e ${\dot{s}}_{4}$ de forma a estabilizar $s_{3}$ e $s_{4}$ como desejado. Nota-se também os termos ${\ddot{\widetilde{x}}}_{b}$ e ${\ddot{\widetilde{y}}}_{b}$ possuem os termos $\ddot{x}$ e $\ddot{y}$, que são definidos em função de $u_{1}$, ao qual se atribui o valor já determinado pelo controlador de altitude para esta entrada, comportando-se como se fosse uma constante neste contexto (XIONG; ZHENG, 2014).

Assim, similarmente ao que foi feito para o subsistema totalmente atuado, deseja-se que:

  ---------------------------------------------------------------------------- ------
  $${\dot{s}}_{3} = - \kappa_{3}s_{3} - \eta_{3}{sign}\left( s_{3} \right)$$   (3.)

  $${\dot{s}}_{4} = - \kappa_{4}s_{4} - \eta_{4}{sign}\left( s_{4} \right)$$   (3.)
  ---------------------------------------------------------------------------- ------

Igualando as equações (3.74) e (3.75) a (3.76) e (3.77), substituindo $\ddot{\psi}$ e $\ddot{\theta}$ dados pelo modelo (3.52) e isolando $u_{2}$ e $u_{3}$, obtém-se:

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $$u_{3} = \frac{\lambda_{1}{\ddot{\widetilde{x}}}_{b} + \lambda_{2}{\dot{\widetilde{x}}}_{b} + \lambda_{3}\left( {\ddot{\theta}}_{d} - f_{\theta} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \kappa_{3}s_{3} + \eta_{3}{sign}\left( s_{3} \right)}{\lambda_{3}b_{\theta}}$$   (.)

  $$u_{2} = \frac{\lambda_{5}{\ddot{\widetilde{y}}}_{b} + \lambda_{6}{\dot{\widetilde{y}}}_{b} + \lambda_{7}\left( {\ddot{\phi}}_{d} - f_{\phi} \right) + \lambda_{8}\left( {\dot{\phi}}_{d} - \dot{\phi} \right) + \kappa_{4}s_{4} + \eta_{4}{sign}\left( s_{4} \right)}{\lambda_{7}b_{\phi}}$$             (.)
  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

A fim de reduzir o efeito de *chattering*, similarmente ao que foi feito no CMD do subsistema totalmente atuado, toma-se as aproximações:

  ----------------------------------------------------------------------------------------------------- ---------
  $${sign}\left( s_{3} \right) \approx \tanh\left( \epsilon_{3}s_{3} \right),\ \ \epsilon_{3} \gg 0$$   (.)

  $${sign}\left( s_{4} \right) \approx \tanh\left( \epsilon_{4}s_{4} \right),\ \ \epsilon_{4} \gg 0$$   (.)
  ----------------------------------------------------------------------------------------------------- ---------

#### Análise de Estabilidade das Variáveis Deslizantes

Como feito no CMD do subsistema totalmente atuado, a estabilidade das variáveis deslizantes é determinada pela teoria de estabilidade de *Lyapunov*. Assim, define-se as seguintes funções de *Lyapunov* candidatas:

  ----------------------------------------------------------------- -----
  $$V_{3}\left( s_{3} \right) = \frac{1}{2}s_{3}^{2}$$              (.)

  $$V_{4}\left( s_{4} \right) = \frac{1}{2}s_{4}^{2}$$              (.)
  ----------------------------------------------------------------- -----

Nota-se que as condições (a) $V(0) = 0$ e (b) $V(x) > 0\ (x \neq 0$) são satisfeitas, visto que $V_{3}$ e $V_{4}$ consistem em funções quadráticas. Derivando-se $V_{3}$, obtém-se:

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ---------
  $${{\dot{V}}_{3} = s_{3}{\dot{\mathbf{s}}}_{\mathbf{3}} = s_{3}\left\lbrack \lambda_{1}{\ddot{\widetilde{\mathbf{x}}}}_{\mathbf{b}} + \lambda_{2}{\dot{\widetilde{x}}}_{b} + \lambda_{3}\left( {\ddot{\theta}}_{d} - \ddot{\mathbf{\theta}} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) \right\rbrack                                                                                                                       (.)
  }{= s_{1}\left\lbrack \lambda_{1}\left( \cos\psi\left( {\ddot{x}}_{d} - f_{x} - b_{x}u_{1}\  - d_{x} \right) + \sin\psi\left( {\ddot{y}}_{d} - f_{y} - b_{y}u_{1}\  - d_{y} \right) \right)\  + \lambda_{2}{\dot{\widetilde{x}}}_{b}\ \ \  + \lambda_{3}\left( {\ddot{\theta}}_{d} - f_{\theta} - b_{\theta}\mathbf{u}_{\mathbf{3}} - d_{\theta} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) \right\rbrack
  }{{\dot{V}}_{3} = - \kappa_{3}s_{3}^{2} - \eta_{3}\left| s_{3} \right| - \left\lbrack \left( \lambda_{1} + \lambda_{3} \right)\left( \cos\psi d_{x} + \sin\psi d_{y} \right) + d_{\theta} \right\rbrack\mathbf{s}_{\mathbf{3}} \leq - \kappa_{3}s_{3}^{2} - \eta_{3}\left| s_{3} \right| + \left( \left| \lambda_{1} + \lambda_{3} \right|\left| \cos\psi d_{x} + \sin\psi d_{y} \right| + \left| d_{\theta} \right| \right)\left| s_{3} \right|}$$

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ---------

Tomando-se $D_{{\widetilde{x}}_{b}} = \max\left( \left| \lambda_{1} + \lambda_{3} \right|\left| \cos\psi d_{x} + \sin\psi d_{y} \right| + \left| d_{\theta} \right| \right)$:

  -------------------------------------------------------------------------------------------------------------------------- -----
  $${{\dot{V}}_{3} \leq - \kappa_{3}s_{3}^{2} - \eta_{3}\left| s_{3} \right| + D_{{\widetilde{x}}_{b}}\left| s_{3} \right|   (.)
  }{{\dot{V}}_{3} \leq - \kappa_{3}s_{3}^{2} + \left( D_{{\widetilde{x}}_{b}} - \eta_{3} \right)\left| s_{3} \right|}$$

  -------------------------------------------------------------------------------------------------------------------------- -----

Observa-se que ${\dot{V}}_{3} < 0$ para $\kappa_{3} \geq 0$ e $\eta_{3} > D_{{\widetilde{x}}_{b}}$. Similarmente, ${\dot{V}}_{4} < 0$ para $\kappa_{4} \geq 0$ e $\eta_{4} > D_{{\widetilde{y}}_{b}}$, sendo $D_{{\widetilde{y}}_{b}} = \max\left( \left| \lambda_{5} + \lambda_{7} \right|\left| - \sin\psi d_{x} + \cos\psi d_{y} \right| + \left| d_{\phi} \right| \right)$. Portanto, sob estas condições, o subsistema é assintoticamente estável no sentido de Lyapunov (para a variáveis $s_{3}$ e $s_{4}$) e robusto contra distúrbios que não ultrapassem os limites definidos.

#### Análise de Estabilidade do Sistema nas Superfícies Deslizantes

A forma mais trivial de provar a estabilidade do sistema durante a fase de deslizamento seria constatando que o sistema toma a forma $\dot{x} = - \mathbf{K}x$, onde $\mathbf{K}$ possui apenas valores positivos na diagonal principal, de forma que o sistema seja assintoticamente estável na variável $x$. Porém, a característica de subatuação e a forma de definição das variáveis deslizantes não favorece este cenário. Outra forma adequada seria definir uma função de *Lyapunov* $V(x)$ candidata na condição de deslizamento e constatar que $V(x) < 0$ (SANKARANARAYANAN; MAHINDRAKAR, 2009). Porém, esta abordagem não se demonstra trivial. Por fim, recorre-se à análise de estabilidade local em torno do ponto equilíbrio com base nos trabalhos de (ASHRAFIUON; ERWIN, 2008; ZHENG; XIONG; LUO, 2014).

A ideia central consiste em determinar os coeficientes $\lambda_{1}$ a $\lambda_{8}$ a partir da condição de estabilidade de *Routh-Hurwitz* aplicado às equações das superfícies deslizantes linearizadas em torno do ponto de equilíbrio.

##### Estabilidade no Deslizamento em $s_{3}$ {#estabilidade-no-deslizamento-em-s_3 .unnumbered}

Primeiramente, rearranja-se as equações (3.74) e (3.72) para as condições de deslizamento, em que ${\dot{s}}_{3} = 0$ e $s_{3} = 0$:

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------
  $${\ddot{\theta}}_{d} - \ddot{\theta} = - \frac{\lambda_{1}}{\lambda_{3}}{\ddot{\widetilde{x}}}_{b} - \frac{\lambda_{2}}{\lambda_{3}}{\dot{\widetilde{x}}}_{b} - \frac{\lambda_{4}}{\lambda_{3}}\left( {\dot{\theta}}_{d} - \dot{\theta} \right)$$   (3.)

  $${\dot{\widetilde{x}}}_{b} = - \frac{\lambda_{2}}{\lambda_{1}}{\widetilde{x}}_{b} - \frac{\lambda_{3}}{\lambda_{1}}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) - \frac{\lambda_{4}}{\lambda_{1}}\left( \theta_{d} - \theta \right)$$           (3.)
  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------

Substituindo a Eq. (3.87) na Eq. (3.86), tem-se:

  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $${\ddot{\theta}}_{d} - \ddot{\theta} = - \frac{\lambda_{1}}{\lambda_{3}}{\ddot{\widetilde{x}}}_{b} + \frac{\lambda_{2}^{2}}{\lambda_{1}\lambda_{3}}{\widetilde{x}}_{b} + \left( \frac{\lambda_{2}}{\lambda_{1}} - \frac{\lambda_{4}}{\lambda_{3}} \right)\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \frac{\lambda_{2}\lambda_{4}}{\lambda_{1}\lambda_{3}}\left( \theta_{d} - \theta \right)$$   (.)

  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

Faz-se então uma redefinição de variável, $y_{1} = \theta_{d} - \theta$, $y_{2} = {\dot{\theta}}_{d} - \dot{\theta}$ e $y_{3} = {\widetilde{x}}_{b}$, obtendo-se o sistema:

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------
  $$\left\{ \begin{aligned}                                                                                                                                                                                                                                                                                               (3.)
  {\dot{y}}_{1} & = y_{2} \\
  {\dot{y}}_{2} & = - \frac{\lambda_{1}}{\lambda_{3}}{\ddot{\widetilde{x}}}_{b}\left( y_{1},y_{2} \right) + \frac{\lambda_{2}\lambda_{4}}{\lambda_{1}\lambda_{3}}y_{1} + \left( \frac{\lambda_{2}}{\lambda_{1}} - \frac{\lambda_{4}}{\lambda_{3}} \right)y_{2} + \frac{\lambda_{2}^{2}}{\lambda_{1}\lambda_{3}}y_{3} \\
  {\dot{y}}_{3} & = - \frac{\lambda_{2}}{\lambda_{1}}y_{1} - \frac{\lambda_{3}}{\lambda_{1}}y_{2} - \frac{\lambda_{4}}{\lambda_{1}}y_{3} \\
  \end{aligned} \right.\ $$

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------

Na Eq. (3.89), ${\ddot{\widetilde{x}}}_{b}\left( y_{1},y_{2} \right)$ é dado pela Eq. (3.71) de modo que $\theta = \theta_{d} - y_{1}$ e $\dot{\theta} = {\dot{\theta}}_{d} - y_{2}$. Observa-se que, $y_{1} \rightarrow 0$, $y_{2} \rightarrow 0$ e $y_{3} \rightarrow 0$ quando as variáveis estão próximas dos seus pontos de equilíbrio, isto é, $\theta \rightarrow \theta_{d}$, $\dot{\theta} \rightarrow {\dot{\theta}}_{d}$ e ${\widetilde{x}}_{b} \rightarrow 0$. Definindo o vetor $\overrightarrow{y} = \left\{ y_{1},y_{2},y_{3} \right\}^{T}$, o ponto de equilíbrio ${\overrightarrow{y}}_{e} = \left\{ 0,0,0 \right\}$ e  $\dot{\overrightarrow{y}} = f\left( \overrightarrow{y} \right)$ (sistema 3.89), a linearização de $f\left( \overrightarrow{y} \right)$ em torno do ponto de equilíbrio é dada por:

  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------
  $$f'\left( \overrightarrow{y} \right) = \left. \ \mathbf{J}(f) \right|_{\overrightarrow{y} = \left\{ 0,0,0 \right\}}\ \overrightarrow{y} + f\left( {\overrightarrow{y}}_{e} \right)$$   (3.)

  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------

Na Eq. (3.90), $\left. \ \mathbf{J}(f) \right|_{\overrightarrow{y} = {\overrightarrow{y}}_{e}}$, ou simplesmente $\mathbf{J}$**,** é a Jacobiana da função $f(y)$ avaliada no ponto de equilíbrio $y_{e}$, definido como:

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ ------
  $$\mathbf{J} = \begin{bmatrix}                                                                                                                                                                                                       (3.)
  \frac{\partial f_{1}}{\partial y_{1}} & \frac{\partial f_{1}}{\partial y_{2}} & \frac{\partial f_{1}}{\partial y_{3}} \\
  \frac{\partial f_{2}}{\partial y_{1}} & \frac{\partial f_{2}}{\partial y_{2}} & \frac{\partial f_{2}}{\partial y_{3}} \\
  \frac{\partial f_{3}}{\partial y_{1}} & \frac{\partial f_{3}}{\partial y_{2}} & \frac{\partial f_{3}}{\partial y_{3}} \\
  \end{bmatrix}_{y = y_{e}} = \begin{bmatrix}
  0 & 1 & 0 \\
   - \frac{\lambda_{1}}{\lambda_{3}}F + \frac{\lambda_{2}\lambda_{4}}{\lambda_{1}\lambda_{3}} & \left( \frac{\lambda_{2}}{\lambda_{1}} - \frac{\lambda_{4}}{\lambda_{3}} \right) & \frac{\lambda_{2}^{2}}{\lambda_{1}\lambda_{3}} \\
   - \frac{\lambda_{4}}{\lambda_{1}} & - \frac{\lambda_{3}}{\lambda_{1}} & - \frac{\lambda_{2}}{\lambda_{1}} \\
  \end{bmatrix}$$

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ ------

Dado que:

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ ------
  $$F = \frac{\partial{\ddot{\widetilde{x}}}_{b}}{\partial y_{1}} = \frac{u_{1}c(\phi)m}{M(M + m)}\left\lbrack c\theta_{d}\ A\left( \psi,\phi_{L},\theta_{L} \right) - s\theta_{d}B\left( \psi,\phi_{L},\theta_{L} \right) \right\rbrack$$   (3.)

  $$A\left( \psi,\phi_{L},\theta_{L} \right) = \frac{M}{m} + \left\lbrack \left( c\psi c\theta_{L} + s\psi s\phi_{L}s\theta_{L} \right)^{2} + s\psi^{2}c\phi_{L}^{2} \right\rbrack$$                                                         (.)

  $$B\left( \psi,\phi_{L},\theta_{L} \right) = c\phi_{L}c\theta_{L}\left( s\psi s\phi_{L}c\theta_{L} - c\psi s\theta_{L} \right)$$                                                                                                           (.)
  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ ------

Assim, o sistema linear descrito pela Eq. (3.90) é estável se os autovalores da matriz do Jacobiana dada pela Eq. (3.91) forem todos menores que zero, de forma que $\overrightarrow{y}$ e  $\dot{\overrightarrow{y}}$ apresentem convergência assintótica até o ponto de equilíbrio. Espera-se que esta condição possa ser alcançada ajustando-se os coeficientes $\lambda_{1}$, $\lambda_{2}$, $\lambda_{3}$ e $\lambda_{4}$. Com isso, calcula-se o polinômio característico de $\mathbf{J}$, fazendo:

  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $$\det\left( p\mathbb{I} - \mathbf{J} \right) = 0$$                                                                                                                        (.)

  $$p^{3} + \left( \frac{\lambda_{4}}{\lambda_{3}} \right)p^{2} + \left( \frac{\lambda_{1}}{\lambda_{3}}F \right)p + \left( \frac{\lambda_{2}}{\lambda_{3}}F \right) = 0$$   (.)
  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

Os autovalores são dados pelas raízes do polinômio característico. Porém, ao invés de calcular as raízes diretamente, aplica-se o critério de estabilidade de *Routh-Hurwitz*, que infere a estabilidade do sistema apenas avaliando os coeficientes do polinômio característico. Tendo o polinômio característico de terceiro grau escrito na forma $p^{3} + a_{2}p^{2} + a_{1}p + a_{0} = 0$, é condição necessária e suficiente para que o sistema linear invariante no tempo associado seja estável: $a_{2} > 0$, $a_{0} > 0$ e $a_{2}a_{1} > a_{0}$ (NISE, 2011). Portanto, para o polinômio característico (3.96):

  ---------------------------------------------------------------------------------------------- ------
  $$1.\ \ \frac{\lambda_{4}}{\lambda_{3}} > 0$$                                                  (3.)

  $$2.\ \ \frac{\lambda_{2}}{\lambda_{3}}F > 0$$                                                 (3.)

  $$3.\ \ \frac{\lambda_{1}\lambda_{4}}{\lambda_{3}^{2}}F > \frac{\lambda_{2}}{\lambda_{3}}F$$   (3.)
  ---------------------------------------------------------------------------------------------- ------

Observa-se que as duas últimas condições têm dependência do comportamento de $F$. Portanto, vale o estudo desta função para definir os parâmetros de controle. Assumindo que $u_{1} > 0$ (propulsão sempre positiva) e $- \frac{\pi\ }{2} < \left\lbrack \psi,\theta\text{,}\phi_{L},\theta_{L} \right\rbrack < \frac{\pi\ }{2}$, é possível fazer com que $F$ seja maior do que zero condicionando o valor dado a $\theta_{d}$, que é de domínio do gerador de trajetórias.

Primeiramente, nota-se que o termo em evidência da Eq. (3.92) ($u_{1}\cos\phi$) é sempre positivo para as condições de operação impostas. Em segundo, verifica-se que $\theta_{d}$ pondera as contribuições de $A$ e $B$ na equação de forma que $A$ é maximizado e $B$ neutralizado para $\theta_{d} = 0$ e o oposto ocorre para $\theta_{d} = \pm \frac{\pi}{2}$.

Outro comportamento observado é o de que $A\left( \psi,\phi_{L},\theta_{L} \right)$ é sempre positivo e o seu valor mínimo possível é $\frac{M}{m}$. Portanto, $\cos\theta_{d}A$ é maior do que $0$ nas condições de operação. Na contramão, o termo $B\left( \psi,\phi_{L},\theta_{L} \right)$ pode assumir valores positivos e negativos, porém limitados a $\pm \frac{1}{2}$ e, portanto, $\left| \sin\theta_{d}B \right|$ é menor do que $\frac{1}{2}$ nas condições de operação.

Este comportamento indica que pode existir um valor máximo para $\theta_{d}$ que $F$ se tornaria positivo independentemente dos valores de $A$ e $B$. De fato, o caso em que $A$ assume seu valor mínimo e $B$ assume seu valor máximo (em módulo) retrata o cenário em que $F$ assume o menor valor possível. Assim, o valor de $\theta_{d}$ que anula $F$ nestas condições extremas ($\theta_{d}^{*}$) é obtido fazendo-se $F = 0$:

  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------
  $$\cos{\left( \theta_{d}^{*} \right)\min\left( |A| \right)} - \sin\left( \theta_{d}^{*} \right)\max\left( |B| \right) = 0 \rightarrow \tan\left( \theta_{d}^{*} \right) = \pm \frac{\min\left( |A| \right)}{\max\left( |B| \right)}$$   (.)

  $$\theta_{d}^{*} = \pm {atan}\left( \frac{2M}{m} \right)$$                                                                                                                                                                              (3.)
  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------

Portanto, pode-se afirmar que:

  ---------------------------------------------------------------------------------- -----
  $$F > 0\ \ se\ \ \left| \theta_{d} \right| < {atan}\left( \frac{2M}{m} \right)$$   (.)

  ---------------------------------------------------------------------------------- -----

Na realidade, a definição deste limite é conservadora, visto que não há um estado $\left\lbrack \psi,\phi_{L},\theta_{L} \right\rbrack$ possível que faça com que $A$ e $B$ assumam seus valores de mínimo e máximo (em módulo) ao mesmo tempo. O limite real ocorre, dentre outros estados, quando $\psi = 0$, $\phi_{L} = 0$ e $\theta_{L} = \pm \frac{\pi}{4}$, em que $A\left( 0,0, \pm \frac{\pi}{4} \right) = \frac{M}{m} + \frac{1}{2}$ e $B\left( 0,0, \pm \frac{\pi}{4} \right) = \frac{1}{2}$, conduzindo a $\theta_{d}^{*} = \pm {atan}\left( \frac{2M}{m} + 1 \right)$ e uma condição de estabilidade mais flexível em que:

  -------------------------------------------------------------------------------------------- -----
  $$F \geq 0\ \ se\ \ \left| \theta_{d} \right| \leq {atan}\left( \frac{2M}{m} + 1 \right)$$   (.)

  -------------------------------------------------------------------------------------------- -----

Assim, assumindo $F > 0$, as relações de estabilidade dos coeficientes do controlador (Equações 3.97, 3.98 e 3.99) podem ser resumidas a:

  --------------------------------------------------------------------------- -----
  $$\frac{\lambda_{4}}{\lambda_{3}} > \frac{\lambda_{2}}{\lambda_{1}} > 0$$   (.)

  --------------------------------------------------------------------------- -----

Constata-se que a relação entre os coeficientes associados a $\phi$ e $\dot{\phi}$ deve ser superior à relação dos coeficientes associados ${\widetilde{y}}_{b}$ e ${\dot{\widetilde{y}}}_{b}$ e que nenhum coeficiente pode assumir valor nulo nem possuir sinal diferente dos demais.

##### Estabilidade no Deslizamento em $s_{4}$ {#estabilidade-no-deslizamento-em-s_4 .unnumbered}

Basicamente, aplica-se o mesmo procedimento feito para $s_{3}$, em que ${\widetilde{y}}_{b}$ é similar a ${\widetilde{x}}_{b}$ e $\phi$ é similar a $\theta$. O jacobiano (Eq. 3.91), o polinômio característico (Eq. 3.96) e as condições de estabilidade (Equações 3.97, 3.98 e 3.99) são idênticas substituindo $\lambda_{1},\ \lambda_{2},\ \lambda_{3}$ e $\lambda_{4}$ por $\lambda_{5},\lambda_{6},\lambda_{7}$ e $\lambda_{8}$ respectivamente, e o termo $F$ pelo termo $G$, dado por:

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ --------
  $$G = \frac{\partial{\ddot{\widetilde{y}}}_{b}}{\partial y_{1}} = \frac{u_{1}\ m}{M(M + m)}\left\lbrack - c\phi_{d}\ C\left( \psi,\phi_{L},\theta_{L} \right) + s\phi_{d}\ D\left( \psi,\phi_{L},\theta_{L} \right) \right\rbrack$$        (.)

  $$C = \frac{M}{m} + \left\lbrack \left( s\psi c\theta_{L} + c\psi s\phi_{L}s\theta_{L} \right)^{2} + c\psi^{2}c\phi_{L}^{2} \right\rbrack$$                                                                                                (.)

  $$D = c\theta c\phi_{L}c\theta_{L}\left( c\psi s\phi_{L}c\theta_{L} + s\psi s\theta_{L} \right) + s\theta\left\lbrack s\phi_{L}s\theta_{L}c\theta_{L}\left( s\psi^{2} - c\psi^{2} \right) - c\phi_{L}^{2}s\theta_{L}^{2} \right\rbrack$$   (.)
  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ --------

De forma semelhante, o termo em evidência é sempre positivo para as condições de operação impostas e, no termo entre colchetes, $\phi_{d}$ pondera $C$ e $D$ de forma que o módulo de $C$ é maximizado e $D$ é neutralizado para $\phi_{d} = 0$ e o oposto ocorre para $\phi_{d} = \pm \frac{\pi}{2}$. Também é possível verificar que $C \geq \frac{M}{m}$ e $|D| \leq \frac{1}{2}$. Porém, diferentemente do que ocorre para $F$, o termo estritamente positivo $C$ está multiplicado a $- \cos\phi_{d}$, que é sempre negativo para as condições de operação.

Desse modo, deseja-se fazer com que $G$ seja sempre menor do que zero condicionando o valor de $\phi_{d}$. Suspeita-se de que possa existir um valor máximo para $\phi_{d}$ que faça com que $G$ seja negativo independentemente dos valores de $C$ e $D$. De fato, o caso em que $C$ assume seu valor mínimo e $D$ assume seu valor máximo (em módulo) retrata o cenário em que $G$ assume o maior valor possível. Assim, o valor de $\phi_{d}$ que anula $G$ nestas condições extremas ($\phi_{d}^{*}$) é obtido fazendo-se $G = 0$:

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------
  $$- \cos{\left( \phi_{d}^{*} \right)\min\left( |C| \right)} + \sin\left( \phi_{d}^{*} \right)\max\left( |D| \right) = 0 \rightarrow \tan\left( \theta_{d}^{*} \right) = \pm \frac{\min\left( |C| \right)}{\max\left( |D| \right)}$$   (.)

  $$\phi_{d}^{*} = \pm {atan}\left( \frac{2M}{m} \right)$$                                                                                                                                                                              (3.)
  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------

Podendo-se afirmar que:

  -------------------------------------------------------------------------------------- -----
  $$G < 0\ \ \ \ se\ \ \ \left| \phi_{d} \right| < {atan}\left( \frac{2M}{m} \right)$$   (.)

  -------------------------------------------------------------------------------------- -----

Vale observar que a definição dos limites para $\phi_{d}$ e $\theta_{d}$ foi feita sobre o modelo linearizado e com a aproximação $\lbrack p,q,r\rbrack \approx \lbrack\dot{\phi},\dot{\theta},\dot{\psi}\rbrack,$ portanto é válida somente em regiões bem próximas do ponto de equilíbrio. Este conservadorismo enfraquece a utilidade prática dos valores obtidos para $\phi_{d}^{*}$ e $\theta_{d}^{*}$ em termos numéricos, porém ainda é possível concluir que a estabilidade do sistema fica cada vez mais comprometida quanto maior for a massa da carga em relação à massa da aeronave.

Assim, assumindo $G < 0$, as relações de estabilidade dos coeficientes do controlador (Equações 3.97, 3.98 e 3.99) resumem-se a:

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $$\frac{\lambda_{8}}{\lambda_{7}} > 0\ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \  \rightarrow \ \ \ \ \ \ \ \ {sign}\left( \lambda_{8} \right) = {sign}\left( \lambda_{7} \right)$$                                                                                                                                      (.)

  $$\frac{\lambda_{5}\lambda_{8}}{\lambda_{7}^{2}} < \frac{\lambda_{6}}{\lambda_{7}} < 0\ \ \ \  \rightarrow \ \ \ \ \ \ \ {sign}\left( \lambda_{5} \right) = {sign}\left( \lambda_{6} \right) \neq {sign}\left( \lambda_{7} \right),\ \ \frac{\lambda_{8}}{\lambda_{7}} > \frac{\lambda_{6}}{\lambda_{5}} > 0$$   (.)
  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

Em suma, constata-se que a relação entre os coeficientes associados a $\phi$ e $\dot{\phi}$ deve ser superior à relação dos coeficientes associados ${\widetilde{y}}_{b}$ e ${\dot{\widetilde{y}}}_{b}$. Adicionalmente, requer-se que os coeficientes atrelados $\phi$ e $\dot{\phi}$ tenham sinais opostos aos coeficientes atrelados a ${\widetilde{y}}_{b}$ e ${\dot{\widetilde{y}}}_{b}$.

### Resumo

+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| Variáveis deslizantes:                                                                                                                                                                                                                                                                                                       |
|                                                                                                                                                                                                                                                                                                                              |
|   -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
|   $$s_{1} = \left( {\dot{z}}_{d} - \dot{z} \right) + \lambda_{z}\left( z_{d} - z \right),\ \ \lambda_{z} > 0$$                                                                                                                                                                                                               |
|                                                                                                                                                                                                                                                                                                                              |
|   $$s_{2} = \left( {\dot{\psi}}_{d} - \dot{\psi} \right) + \lambda_{\psi}\left( \psi_{d} - \psi \right),\ \ \lambda_{\psi} > 0$$                                                                                                                                                                                             |
|                                                                                                                                                                                                                                                                                                                              |
|   $$s_{3} = \lambda_{1}{\dot{\widetilde{x}}}_{b} + \lambda_{2}{\widetilde{x}}_{b} + \lambda_{3}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \lambda_{4}\left( \theta_{d} - \theta \right),\ \ \frac{\lambda_{4}}{\lambda_{3}} > \frac{\lambda_{2}}{\lambda_{1}} > 0$$                                                  |
|                                                                                                                                                                                                                                                                                                                              |
|   $$s_{4} = \lambda_{5}{\dot{\widetilde{y}}}_{b} + \lambda_{6}{\widetilde{y}}_{b} + \lambda_{7}\left( {\dot{\phi}}_{d} - \dot{\phi} \right) + \lambda_{8}\left( \phi_{d} - \phi \right),\ \ \frac{\lambda_{8}}{\lambda_{7}} > 0,\ \ \frac{\lambda_{5}\lambda_{8}}{\lambda_{7}^{2}} < \frac{\lambda_{6}}{\lambda_{7}} < 0$$   |
|   -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
|                                                                                                                                                                                                                                                                                                                              |
| Variável auxiliar:                                                                                                                                                                                                                                                                                                           |
|                                                                                                                                                                                                                                                                                                                              |
| > $$\left\{ \begin{array}{r}                                                                                                                                                                                                                                                                                                 |
| > {\widetilde{x}}_{b}^{(n)} \\                                                                                                                                                                                                                                                                                               |
| > {\widetilde{y}}_{b}^{(n)} \\                                                                                                                                                                                                                                                                                               |
| > \end{array} \right\} = \begin{bmatrix}                                                                                                                                                                                                                                                                                     |
| > \cos\psi & \sin\psi \\                                                                                                                                                                                                                                                                                                     |
| >  - \sin\psi & \cos\psi \\                                                                                                                                                                                                                                                                                                  |
| > \end{bmatrix}\left\{ \begin{array}{r}                                                                                                                                                                                                                                                                                      |
| > x_{d}^{(n)} - x^{(n)} \\                                                                                                                                                                                                                                                                                                   |
| > y_{d}^{(n)} - y^{(n)} \\                                                                                                                                                                                                                                                                                                   |
| > \end{array} \right\},\ \ \dot{\psi} = \ddot{\psi} = 0,\ \ \ \ \ n = 0,\ 1,\ 2$$                                                                                                                                                                                                                                            |
|                                                                                                                                                                                                                                                                                                                              |
| Entradas:                                                                                                                                                                                                                                                                                                                    |
|                                                                                                                                                                                                                                                                                                                              |
| +-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+              |
| | $$u_{1} = \frac{{\ddot{z}}_{d} - f_{z} + \kappa_{1}s_{1} + \eta_{1}{sign}\left( s_{1} \right)}{b_{z}},\ \ \kappa_{1} > 0,\ \ \eta_{1} > \max\left( \left| d_{z} \right| \right),\ \ b_{z} \neq 0$$                                                                                                          |              |
| +-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+              |
| | $$u_{4} = \frac{{\ddot{\psi}}_{d} - f_{\psi} + \kappa_{2}s_{2} + \eta_{2}{sign}\left( s_{2} \right)\ }{b_{\psi}},\ \ \kappa_{2} > 0,\ \ \eta_{2} > \max\left( \left| d_{\psi} \right| \right),\ \ b_{\psi} \neq 0$$                                                                                         |              |
| +-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+              |
| | $$u_{3} = \frac{\lambda_{1}{\ddot{\widetilde{x}}}_{b} + \lambda_{2}{\dot{\widetilde{x}}}_{b} + \lambda_{3}\left( {\ddot{\theta}}_{d} - f_{\theta} \right) + \lambda_{4}\left( {\dot{\theta}}_{d} - \dot{\theta} \right) + \kappa_{3}s_{3} + \eta_{3}{sign}\left( s_{3} \right)}{\lambda_{3}b_{\theta}},\ $$ |              |
| |                                                                                                                                                                                                                                                                                                             |              |
| | $$\ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \kappa_{3} > 0,\ \ \eta_{3} > \max\left( \left| \lambda_{1} + \lambda_{3} \right|\left| \cos\psi d_{x} + \sin\psi d_{y} \right| + \left| d_{\theta} \right| \right),\ \ b_{\theta} \neq 0$$                                                             |              |
| +-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+              |
| | $$u_{2} = \frac{\lambda_{5}{\ddot{\widetilde{y}}}_{b} + \lambda_{6}{\dot{\widetilde{y}}}_{b} + \lambda_{7}\left( {\ddot{\phi}}_{d} - f_{\phi} \right) + \lambda_{8}\left( {\dot{\phi}}_{d} - \dot{\phi} \right) + \kappa_{4}s_{4} + \eta_{4}{sign}\left( s_{4} \right)}{\lambda_{7}b_{\phi}},$$             |              |
| |                                                                                                                                                                                                                                                                                                             |              |
| | $$\ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \kappa_{4} > 0,\ \ \eta_{4} > \max\left( \left| \lambda_{5} + \lambda_{7} \right|\left| - \sin\psi d_{x} + \cos\psi d_{y} \right| + \left| d_{\phi} \right| \right),\ \ b_{\phi} \neq 0$$                                                                 |              |
| +-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+              |
+==============================================================================================================================================================================================================================================================================================================================+
+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+

## Simulação

Para avaliar o funcionamento do controlador, realiza-se uma simulação do sistema utilizando o software MATLAB, em que se integra a equação dinâmica descrita no Capítulo 2 com os sinais de entrada calculados pelo controlador. Os parâmetros físicos da simulação, quando não especificados explicitamente, são adotados como apresenta a Tabela 3.1:

Tabela 3.1 - Parâmetros físicos de simulação

  ---------------------------------------------------------------------------------------
  **Parâmetro**       **Valor**                   **Parâmetro**       **Valor**
  ------------------- --------------------------- ------------------- -------------------
  $$M$$               $$2,4\ kg$$                 $$g$$               $$9,81\ m/s^{2}$$

  $$m$$               $$1,0\ kg$$                 $$c_{x},\ c_{y}$$   $$0,2\ kg/s$$

  $$l$$               $$1,0\ m$$                  $$c_{z}$$           $$0,5\ kg/s$$

  $$I_{x},\ I_{y}$$   $$0,055\ kg \cdot m^{2}$$   $$c_{L}$$           $$0,1\ kg/s$$

  $$I_{z}$$           $$0,1\ kg \cdot m^{2}$$     \-                  \-
  ---------------------------------------------------------------------------------------

Os valores apresentados na Tabela 3.1 são aproximadamente as especificações do drone comercial *DJI Matrice 100*, que possui uma capacidade de carga de até aproximadamente 50% da sua massa (JEAONG et al., 2018). Os parâmetros de controle, quando não especificados explicitamente, são definidos como mostra a Tabela 3.2:

Tabela 3.2 - Parâmetros de controle da simulação.

  -------------------------------------------------------------------------
  **Parâmetro**      **Valor**        **Parâmetro**        **Valor**
  ------------------ ---------------- -------------------- ----------------
  $$\lambda_{z}$$    $$2$$            $$\lambda_{\psi}$$   $$2$$

  $$\kappa_{z}$$     $$1$$            $$\kappa_{\psi}$$    $$1$$

  $$\eta_{z}$$       $$1$$            $$\eta_{\psi}$$      $$1$$

  $$\epsilon_{1}$$   $$50$$           $$\epsilon_{2}$$     $$50$$

  $$\lambda_{1}$$    $$5$$            $$\lambda_{5}$$      $$- 5$$

  $$\lambda_{2}$$    $$2$$            $$\lambda_{6}$$      $$- 2$$

  $$\lambda_{3}$$    $$0,1$$          $$\lambda_{7}$$      $$0,1$$

  $$\lambda_{4}$$    $$1$$            $$\lambda_{8}$$      $$1$$

  $$\kappa_{1}$$     $$1$$            $$\kappa_{2}$$       $$1$$

  $$\eta_{1}$$       $$2$$            $$\eta_{2}$$         $$2$$

  $$\epsilon_{3}$$   $$50$$           $$\epsilon_{4}$$     $$50$$
  -------------------------------------------------------------------------

Observa-se que os parâmetros selecionados respeitam os limites especificados. Em especial, observa-se que $\frac{\lambda_{2}}{\lambda_{1}} = \frac{\lambda_{6}}{\lambda_{5}} = \frac{2}{5}$ é menor do que $\frac{\lambda_{4}}{\lambda_{3}} = \frac{\lambda_{8}}{\lambda_{7}} = 10$ e que $\lambda_{5}$ e $\lambda_{6}$ são opostos a $\lambda_{1}$ e $\lambda_{2}$. Também se verifica que o limite de estabilidade para os ângulos de referência de atitude ($\phi_{d}^{*}$ e $\theta_{d}^{*}$) segundo as equações (3.109) e (3.101) é de aproximadamente $1.4\ rad$ ($\approx 80{^\circ}$), indicando que a relação da massa da carga e da aeronave oferece margem expressiva para a estabilidade do subsistema sub-atuado ao menos nas proximidades do ponto de equilíbrio. Os parâmetros do controlador $\kappa_{z}$, $\kappa_{\psi}$, $\kappa_{1}$, $\kappa_{2}$, $\eta_{z}$, $\eta_{\psi}$, $\eta_{1}$ e $\eta_{2}$ foram dimensionados para combater o distúrbio de arrasto do modelo e perturbações internas provenientes das aproximações feitas nas equações do controlador.

Com isso, avalia-se o comportamento do sistema controlado para entrada degrau e para um ponto inicial levemente deslocado do ponto de equilíbrio para avaliar as condições de estabilidade deduzidas na seção anterior.

### Resposta do sistema para entrada degrau unitário

Primeiramente se aplica entrada degrau unitário para a posição e ângulo de guinada da aeronave com velocidades e acelerações nulas. Os ângulos de referência de rolagem e arfagem são mantidos em zero. Ressalta-se que este tipo de entrada não é, pois promove uma mudança abrupta da distância do sistema para o ponto de equilíbrio, que é uma condição não prevista para o controlador. Apesar disso, este teste permite avaliar a robustez do sistema contra esta condição adversa e observar a ação do controlador no comportamento no sistema, em especial sobre as variáveis deslizantes.

Adicionalmente, a fim de destacar o efeito de *chattering* previsto, considera-se a aplicação de ruído branco com desvio padrão de $0,005\ m$ para variáveis de posição do sistema ($x,y,z$), $0,01\ m/s$ para as variáveis de velocidade ($\dot{x},\dot{y},\dot{z}$), $0,005{^\circ}$ para os ângulos ($\phi,\theta,\ \psi,\phi_{L},\theta_{L}$) e $0,01{^\circ}/s$ para as velocidades angulares ($\dot{\phi},\dot{\theta},\ \dot{\psi},{\dot{\phi}}_{L},{\dot{\theta}}_{L}$).

As Figuras 3.7, 3.8 e 3.9 apresentam o comportamento dinâmico da posição, da orientação da aeronave e da orientação do cabo para este cenário, respectivamente.

![](media/image16.emf){width="4.68503937007874in" height="3.7651235783027124in"}

Figura 3.7 -- Posição e velocidade da aeronave para entrada degrau unitário.

![](media/image17.emf){width="4.68503937007874in" height="3.810391513560805in"}

Figura 3.8 - Orientação e velocidade angular da aeronave para entrada nula para $\phi$ e $\theta$ e degrau unitário para $\psi$.

![](media/image18.emf){width="4.3484536307961505in" height="2.740621172353456in"}

Figura 3.9 - Orientação do cabo para entrada degrau unitário.

Primeiramente, observa-se que as variáveis de posição (

Figura 3.7) e ângulo de guinada ($\psi$, Figura 3.8) apresentam convergência exponencial para o estado desejado. Este comportamento era esperado para as variáveis $z$ e $\psi$, visto a condição de atuação plena sobre estes dois graus de liberdade e o comportamento de convergência assintótica projetada para das superfícies deslizantes associadas $s_{1}$ e $s_{2}$. Esperava-se a convergência das variáveis $x$ e $y$ de forma acoplada a $\phi$ e $\theta$, visto que se cumpriu as restrições de estabilidade dos parâmetros de controle. Porém, o comportamento assintótico observado para $x$ e $y$ (

Figura 3.7) combinado ao perfil oscilatório de $\phi$ e $\theta$ (Figura 3.8) não foi previsto ao projetar as variáveis deslizantes. Este comportamento ocorre, dentre outros fatores, em função dos valores escolhidos para os parâmetros de controle, à característica de acoplamento de entrada e ao fato do sinal de referência de atitude ser sempre zero. Para que ocorra deslocamento horizontal, o controlador permitir certa variação de $\phi$ e $\theta$ para direcionar a força de propulsão, porém, contrário a este objetivo, sempre tenta puxar os estados de orientação de volta para o equilíbrio à medida que se afasta de zero.

Na Figura 3.9, observa-se que a movimentação da carga apresenta oscilação regular, mas, apesar do controlador não atuar explicitamente para conter esta oscilação, ela apresenta uma leve tendência de convergência. Dada a observação da convergência das variáveis de estado do drone, este comportamento é esperado visto o acoplamento dinâmico existente entre o estado da carga e o estado do drone, além do efeito dissipativo que o arrasto do ar.

Na Figura 3.8, observa-se um ruído de alta frequência nos ângulos de rolagem ($\phi$) e arfagem ($\theta$) e nas suas derivadas. Este fenômeno é consequência do efeito de *chattering*, observado no comportamento das variáveis deslizantes, apresentado na Figura 3.10.

![](media/image19.emf){width="5.707547025371828in" height="2.597192694663167in"}

Figura 3.10 -- Comportamento das variáveis deslizantes para entrada degrau.

Observa-se na Figura 3.10 que, no momento de aplicação da entrada degrau, em que se provoca um desvio na posição do estado do sistema em relação ao ponto de equilíbrio, as variáveis deslizantes saltam para valores diferentes de zeros. Porém, imediatamente começam a decrescer, realizando a fase de aproximação, que apresentam decaimento exponencial e linear esperados (Seção 3.2.1.2), até atingir o zero e entrar na fase de deslizamento. Porém, visto o ruído branco de alta frequência inserido nas variáveis do sistema, as variáveis não se mantêm exatamente em zero, ativando os termos (quase) descontínuos dos sinais de entrada que atuam no sistema, refletindo o comportamento oscilatório nas variáveis do sistema. A Figura 3.11 apresenta os sinais de controle gerados.

![](media/image20.emf){width="4.4995297462817145in" height="5.245138888888889in"}

Figura 3.11 - Sinais de controle do CMD para entrada degrau unitário.

Nota-se na Figura 3.11 que os sinais $u_{1}$ e $u_{4}$ apresentam um salto negativo posterior ao momento em que se provoca o desvio de referência unitário. Este momento coincide com o momento de estabilização das variáveis deslizantes. Entende-se que este efeito tem relação com a transição abrupta da variável deslizante para zero provocado pelos termos descontínuos do sinal de entrada.

O efeito da transição para a fase de deslizamento também pode ser observado nas variáveis $\phi$ e $\theta$ (Figura 3.8). Observa-se que durante a fase de deslizamento (entre aproximadamente 2s e 4s), os ângulos apresentam comportamento distinto do observado em seguida, quando passa a oscilar de forma regular. O comportamento observado a partir deste momento é majoritariamente definido pelas constantes $\lambda_{1}$ a $\lambda_{8}$, que, para os valores definidos, levaram à estabilização rápida das posições $x$ e $y$ e a um comportamento oscilatório amortecido da atitude do drone.

### Avaliação da condição de estabilidade na superfície deslizante

Para ilustrar a validade das condições de estabilidade para os parâmetros de controle do subsistema sub-atuado, propõe-se comparar o comportamento do sistema a partir de um ponto no entorno da condição de equilíbrio para três configurações de parâmetro: uma estável, outra instável e outra na margem de estabilidade.

Para isso, toma-se como estado inicial $\phi(0) = \theta(0) = 10{^\circ}$ e zero para todas as outras variáveis do sistema. Fixando os parâmetros de controle $\lambda_{2}$, $\lambda_{3}$, $\lambda_{4}$, $\lambda_{6}$ e $\ \lambda_{8}$ em 2 e $\lambda_{6} = - 2$, faz-se $\lambda_{5} = - \lambda_{1}$ e varia-se $\lambda_{1}$ entre 1, 2 e 4, de modo a gerar os cenários em que $\left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack > \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack$ (instável), $\left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack = \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack$ (marginalmente estável) e $\left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack < \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack$ (estável), respectivamente. As Figuras 3.12, 3.13 e 3.14 apresentam o comportamento das variáveis mais influenciadas $x$,$y$, $\phi$,$\theta$, $\phi_{L}$ e $\theta_{L}$, diretamente atreladas à dinâmica ao longo do plano $xy$.

![](media/image21.emf){width="5.905511811023622in" height="2.430452755905512in"}

Figura 3.12 - Comportamento do sistema no plano $xy$ para $\lambda_{1} = 1$ $\left( \left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack > \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack \right)$.

![](media/image22.emf){width="5.905511811023622in" height="2.4304549431321085in"}

Figura 3.13 - Comportamento do sistema no plano $xy$ para $\lambda_{1} = 2$ $\left( \left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack < \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack \right)$.

![](media/image23.emf){width="5.905511811023622in" height="2.41501968503937in"}

Figura 3.14 -- Comportamento do sistema no plano $xy$ para $\lambda_{1} = 4$ $\left( \left\lbrack \frac{\lambda_{2}}{\lambda_{1}},\frac{\lambda_{6}}{\lambda_{5}} \right\rbrack < \left\lbrack \frac{\lambda_{4}}{\lambda_{3}},\frac{\lambda_{8}}{\lambda_{7}} \right\rbrack \right)$.

Como esperado, no primeiro cenário (Figura 3.12) as variáveis divergem indefinidamente. Já na segunda configuração (Figura 3.13), as variáveis oscilam de forma uniforme, sem demonstrar tendência clara de convergência nem divergência. Por fim, ao ajustar as variáveis dentro das condições de estabilidade obtidas (Figura 3.14), as variáveis apresentaram comportamento claro de convergência.

IV.

# GERAÇÃO DE TRAJETÓRIAS

Embora o controlador desenvolvido leve em conta a dinâmica do drone acoplada à carga, ele se preocupa apenas em controlar a posição da aeronave, ignorando o comportamento da carga. Porém, a depender da trajetória de referência dada à aeronave, a carga pode oscilar a altas amplitudes e velocidades, de forma a degradar o desempenho do movimento como um todo.

Para diminuir este efeito, decide-se atuar na geração de trajetórias de modo a induzir a aeronave a se movimentar de forma favorável à estabilização da carga. Especificamente, explora-se a combinação de duas técnicas distintas: a primeira consiste em determinar trajetórias de referência para o drone a partir de trajetórias de posição para a carga e para o ângulo de guinada da aeronave com base na propriedade de planicidade diferencial do sistema. A segunda técnica refere-se ao i*nput shaping*, que filtra o sinal de entrada com base no conhecimento da dinâmica de vibração do sistema para gerar saídas com vibração atenuada.

## Geração de Trajetória com Base na Planicidade Diferencial do Sistema

Primeiramente, apresenta-se a técnica de geração de trajetória desenvolvida por (SREENATH; MICHAEL; KUMAR, 2013) e (MELLINGER, 2012; MELLINGER; KUMAR, 2011). Eles demonstram que o sistema se trata de um sistema diferencialmente plano de modo a ser possível determinar a posição da aeronave dadas trajetórias desejadas para a carga e para o ângulo de guinada da aeronave.

Primeiramente, apresenta-se a definição de planicidade diferencial do sistema e como ela é utilizada para gerar as trajetórias do sistema, detalhando-se a definição de cada variável do sistema. Ressalta-se que esta seção reproduz a ideia central de geração de trajetória dos trabalhos de referência[^5], diferenciando-se por adotar um sistema de coordenadas alternativo, apresentar mais detalhes nas etapas de desenvolvimento das equações e por adicionar a força de arrasto linear ao modelo.

### Planicidade Diferencial do Sistema

Dado um sistema com estado $x \in \mathbb{R}^{n}$ e entrada $u \in \mathbb{R}^{m}$, ele é dito diferencialmente plano se existe um conjunto finito de variáveis $y \in \mathbb{R}^{m}$, denominadas saídas planas, que são descritas em função do estado, da entrada e suas derivadas até uma ordem finita $p$:

  ----------------------------------------------------------------- -----
  $$y = y\left( x,u,\dot{u},\ldots,u^{(p)} \right)$$                (.)

  ----------------------------------------------------------------- -----

de modo que o estado e as entradas do sistema podem ser escritos como funções contínuas destas saídas e suas derivadas até uma ordem finita $q$:

  ----------------------------------------------------------------- -----
  $$x = x\left( y,\dot{y},\ldots,y^{(q)} \right)$$                  (.)

  $$u = u\left( y,\dot{y},\ldots,y^{(q)} \right)$$                  (.)
  ----------------------------------------------------------------- -----

Esta propriedade potencializa o planejamento de trajetória, pois permite que os estados desejados sejam determinados a partir de trajetórias definidas no domínio das saídas planas (FLIESS et al., 1993).

Neste contexto, é possível mostrar que o estado e as entradas do sistema drone com carga suspensa por cabo podem ser escritas em função da posição da carga e ângulo de guinada da aeronave e suas derivadas até determinada ordem. Em outras palavras, $\left\lbrack {\overrightarrow{r}}_{L},\psi \right\rbrack$ é um conjunto de saídas planas para o sistema.

### Determinação das Variáveis do Sistema

#### Determinação da Posição da Aeronave $\overrightarrow{\mathbf{r}}$ e suas Derivadas

Derivando-se a Eq. (2.20) *n* vezes e isolando o termo correspondente à posição do quadcóptero, tem-se:

  ----------------------------------------------------------------------------------------------- ------
  $${\overrightarrow{r}}^{(n)} = {\overrightarrow{r}}_{L}^{(n)} - l{\overrightarrow{p}}^{(n)}$$   (4.)

  ----------------------------------------------------------------------------------------------- ------

Portanto, para determinar a enésima derivada de $\overrightarrow{r}$, basta ter conhecimento da enésima derivada de ${\overrightarrow{r}}_{L}$ e $\overrightarrow{p}$. O vetor $\overrightarrow{p}$ pode ser determinado em função das saídas planas a partir da equação dinâmica da carga (Eq. 2.24). Isolando-se o termo referente à tensão no cabo, tem-se:

  -------------------------------------------------------------------------------------------------------------------------------- ------
  $$\overrightarrow{T} = - m{\ddot{\overrightarrow{r}}}_{L} - mg{\overrightarrow{e}}_{z} - C_{L}{\dot{\overrightarrow{r}}}_{L}$$   (4.)

  -------------------------------------------------------------------------------------------------------------------------------- ------

Observa-se que $\overrightarrow{T}$ pode ser calculado dado o conhecimento de ${\dot{\overrightarrow{r}}}_{L}$ e ${\ddot{\overrightarrow{r}}}_{L}$. Através da definição de $\overrightarrow{p}$, tem-se:

  ----------------------------------------------------------------------------------------- ------
  $$\overrightarrow{p} = \frac{\overrightarrow{T}}{\left\| \overrightarrow{T} \right\|}$$   (4.)

  $$\left\| \overrightarrow{T} \right\| = \overrightarrow{T} \cdot \overrightarrow{p}$$     (4.)
  ----------------------------------------------------------------------------------------- ------

Vale observar que a derivada da Eq. (4.119) não depende de $\dot{\overrightarrow{p}}$:

  --------------------------------------------------------------------------------------------------- -----
  $$\dot{\left\| \overrightarrow{T} \right\|} = \dot{\overrightarrow{T}} \cdot \overrightarrow{p}$$   (.)

  --------------------------------------------------------------------------------------------------- -----

Assim, ${\overrightarrow{p}}^{(n)}$ pode ser obtido derivando-se a Eq. (4.118) $n$ vezes, sendo ${\overrightarrow{T}}^{(n)}$ descrito em função de ${\overrightarrow{r}}_{L}^{(n + 2)}$ (Eq. 4.113) e $\left\| \overrightarrow{T} \right\|^{(n)}$ (dado $n > 1$) em função de ${\overrightarrow{p}}^{(n - 1)}$ (Eq. 4.116). Portanto, ${\overrightarrow{p}}^{(n)}$ depende de ${\overrightarrow{r}}_{L}^{(n + 2)}$ e ${\overrightarrow{r}}^{(n)}$ depende de ${\overrightarrow{p}}^{(n)}$ (Eq. 4.116), logo ${\overrightarrow{r}}^{(n)}$ depende de ${\overrightarrow{r}}_{L}^{(n + 2)}$.

#### Determinação da Orientação $\overrightarrow{\mathbf{\eta}}$ e da força de propulsão $\mathbf{F}_{\mathbf{b}}^{\mathbf{z}}$

A equação da dinâmica da aeronave (Eq. 2.23) pode ser rearranjada da seguinte forma:

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------
  $$M\ddot{\overrightarrow{r}} - T\overrightarrow{p} + Mg{\overrightarrow{e}}_{z} + C\dot{\overrightarrow{r}} = F_{z}^{b}{\overrightarrow{\mathbf{e}}}_{\mathbf{z}}^{\mathbf{b}}$$   (4.)

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- ------

Verifica-se que:

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ -----
  $$\mathbf{R}{\overrightarrow{e}}_{z} = {\overrightarrow{e}}_{z}^{b} = \frac{\overrightarrow{t}}{\left\| \overrightarrow{t} \right\|},\ \ \ \ onde\ \ \ \overrightarrow{t} = \left\{ \begin{array}{r}   (.)
  \ddot{x} + c_{x}\dot{x} - \frac{T_{x}}{M} \\
  \ddot{y} + c_{y}\dot{y} - \frac{T_{y}}{M} \\
  \ddot{z} + g + c_{z}\dot{z} - \frac{T_{z}}{M} \\
  \end{array} \right\}$$

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ -----

Assim, define-se um sistema de coordenadas auxiliar $\Sigma_{c} = \left\lbrack {\overrightarrow{e}}_{x}^{c},{\overrightarrow{e}}_{y}^{c},{\overrightarrow{e}}_{z}^{c} \right\rbrack$, que corresponde ao sistemas de coordenadas inercial rotacionado de $\psi$ em torno de ${\overrightarrow{e}}_{z}$, como mostra a Figura 4.1.

![](media/image24.emf){width="5.305349956255468in" height="2.2830194663167105in"}

Figura 4.1 - Ilustração do sistema de coordenadas auxiliar $\Sigma_{c}$.

Analisando a Figura 4.1, é possível verifica que:

  ----------------------------------------------------------------------------------------- -----
  $${\overrightarrow{e}}_{y}^{c} = \left\lbrack - \sin\psi,\cos\psi,0 \right\rbrack^{T}$$   (.)

  ----------------------------------------------------------------------------------------- -----

A partir de ${\overrightarrow{e}}_{y}^{c}$ é possível determinar os outros vetores unitários que compõem o sistema de coordenadas do corpo, como:

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $${\overrightarrow{e}}_{x}^{b} = \frac{{\overrightarrow{e}}_{y}^{c} \times {\overrightarrow{e}}_{z}^{b}}{\left\| {\overrightarrow{e}}_{y}^{c} \times {\overrightarrow{e}}_{z}^{b} \right\|}$$   (.)

  $${\overrightarrow{e}}_{y}^{b} = {\overrightarrow{e}}_{z}^{b} \times {\overrightarrow{e}}_{x}^{b}$$                                                                                             (.)
  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

Assim, tem-se a matriz de rotação dada por:

  ------------------------------------------------------------------------------------------------------------------------------------ -----
  $$\mathbf{R} = \left\lbrack {\overrightarrow{e}}_{x}^{b},{\overrightarrow{e}}_{y}^{b},{\overrightarrow{e}}_{z}^{b} \right\rbrack$$   (.)

  ------------------------------------------------------------------------------------------------------------------------------------ -----

A partir da matriz de rotação é possível determinar os ângulos de *Euler* (SLABAUGH, 1999). Enfim, define-se $F_{z}^{b}$ substituindo $\mathbf{R}{\overrightarrow{e}}_{z}$ na Eq. (4.121).

#### Determinação da Velocidade Angular $\overrightarrow{\mathbf{\omega}}$

Derivando-se a equação de movimento da aeronave (Eq. 4.121), tem-se que:

  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $$M\ \dddot{\overrightarrow{r}} + C\ddot{\overrightarrow{x}} - \dot{\overrightarrow{T}} = {\dot{F}}_{z}^{b}{\overrightarrow{e}}_{z}^{b} + F_{z}^{b}\left( \overrightarrow{\omega} \times {\overrightarrow{e}}_{z}^{b} \right)$$   (.)

  --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

Projetando esta expressão ao longo de ${\overrightarrow{e}}_{z}^{b}$, tem-se que:

  ------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $$\dot{F_{z}^{b}} = \left( M\dddot{\overrightarrow{r}} + C\ddot{\overrightarrow{x}} - \dot{\overrightarrow{T}} \right) \cdot {\overrightarrow{e}}_{z}^{b}$$   (.)

  ------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

Na direção perpendicular a ${\overrightarrow{e}}_{z}^{b}$ e $\overrightarrow{\omega}$, ao longo da qual se tem:

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $${\overrightarrow{h}}_{\omega} = \overrightarrow{\omega} \times {\overrightarrow{e}}_{z}^{b} = \frac{1}{F_{z}^{b}}\left\{ M\dddot{\overrightarrow{r}} + C\ddot{\overrightarrow{r}} - \dot{\overrightarrow{T}} - \dot{\mathbf{F}_{\mathbf{z}}^{\mathbf{b}}}{\overrightarrow{e}}_{z}^{b} \right\}$$   (.)

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

Observa-se que ${\overrightarrow{h}}_{\omega}$ é a projeção de $\overrightarrow{\omega}$ no plano $x_{b}y_{b}$ rotacionada em 90°, de forma que é possível determinar as componentes da velocidade angular neste plano como$:$

  ---------------------------------------------------------------------------- -----
  $$p = - {\overrightarrow{h}}_{\omega} \cdot {\overrightarrow{e}}_{y}^{b}$$   (.)

  $$q = {\overrightarrow{h}}_{\omega} \cdot {\overrightarrow{e}}_{x}^{b}$$     (.)
  ---------------------------------------------------------------------------- -----

Por fim, conhecidos $p$, $q$ e $\dot{\psi}$, a terceira componente do vetor $\overrightarrow{\omega}$ é obtida da terceira componente da Eq. (2.2):

  ----------------------------------------------------------------- -----
  $$r = \frac{\cos\theta\dot{\psi} - \sin\phi q}{\cos\phi}\ $$      (.)

  ----------------------------------------------------------------- -----

#### Determinação da Aceleração Angular $\dot{\overrightarrow{\mathbf{\omega}}}$ e Momento de Entrada ${\overrightarrow{\mathbf{\tau}}}_{\mathbf{b}}$

Para determinar $\dot{\overrightarrow{\omega}}$, aplica-se um procedimento semelhante ao que se fez para encontrar $\overrightarrow{\omega}$. Primeiramente, deriva-se a equação dinâmica mais uma vez:

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $M\ \ddddot{\overrightarrow{r}} + C\dddot{\overrightarrow{r}} - \ddot{\overrightarrow{T}} = {\ddot{F}}_{z}^{b}{\overrightarrow{e}}_{z}^{b} + 2\left( \overrightarrow{\omega} \times {\dot{F}}_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right) + \dot{\overrightarrow{\omega}} \times F_{z}^{b}{\overrightarrow{e}}_{z}^{b} + \overrightarrow{\omega} \times \left( \overrightarrow{\omega} \times F_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right)$   (.)

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

Projetando esta expressão ao longo de ${\overrightarrow{e}}_{z}^{b}$, tem-se que:

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----
  $${\ddot{F}}_{z}^{b} = \left\lbrack \left( M\ \ddddot{\overrightarrow{r}} + C\dddot{\overrightarrow{r}}\  - \ddot{\overrightarrow{T}} \right) - \overrightarrow{\omega} \times \left( \overrightarrow{\omega} \times F_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right) \right\rbrack \cdot {\overrightarrow{e}}_{z}^{b}$$   (.)

  ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -----

Na direção perpendicular a ${\overrightarrow{e}}_{z}^{b}$ e $\dot{\overrightarrow{\omega}}$, ao longo da qual se tem:

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ -----
  $${\overrightarrow{h}}_{\alpha} = \dot{\overrightarrow{\omega}} \times {\overrightarrow{e}}_{z}^{b} = \frac{1}{F_{z}^{b}}\left\{ M\ \ddddot{\overrightarrow{r}} + C\dddot{\overrightarrow{r}}\  - {\ddot{\mathbf{F}}}_{\mathbf{z}}^{\mathbf{b}}{\overrightarrow{e}}_{z}^{b} - 2\left( \overrightarrow{\omega} \times {\dot{F}}_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right) - \overrightarrow{\omega} \times \left( \overrightarrow{\omega} \times F_{z}^{b}{\overrightarrow{e}}_{z}^{b} \right) \right\}$$   (.)

  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ -----

Observa-se que ${\overrightarrow{h}}_{\alpha}$ corresponde à projeção de $\dot{\overrightarrow{\omega}}$ no plano $x_{b}y_{b}$ rotacionada de 90°, de forma que é possível constatar que:

  ---------------------------------------------------------------------------------- -----
  $$\dot{p} = - {\overrightarrow{h}}_{\alpha} \cdot {\overrightarrow{e}}_{y}^{b}$$   (.)

  $$\dot{q} = {\overrightarrow{h}}_{\alpha} \cdot {\overrightarrow{e}}_{x}^{b}$$     (.)
  ---------------------------------------------------------------------------------- -----

Por fim, conhecidos $\dot{p}$, $\dot{q}$ e $\ddot{\psi}$, a terceira componente do vetor $\dot{\overrightarrow{\omega}}$ é determinada a partir de terceira componente da derivada da Eq. (2.2):

  --------------------------------------------------------------------------------------------------------------------------------------- ------
  $$\dot{r} = \frac{\cos\theta\ddot{\psi} - \sin\phi\dot{q} - \dot{\theta}\left( \dot{\phi} + \sin\theta\dot{\psi} \right)}{\cos\phi}$$   (.)

  --------------------------------------------------------------------------------------------------------------------------------------- ------

Ressalta-se que, para chegar à definição de $\dot{\overrightarrow{\omega}}$ e ${\overrightarrow{\tau}}_{b}$ foi necessário ter conhecimento de $\ \ddddot{\overrightarrow{r}}$ que, por sua vez, requer o conhecimento até ${\overrightarrow{r}}_{L}^{(6)}$ (vide seção 4.1.2.1). Ou seja, para determinar todas as variáveis do sistema, as acelerações angulares e torques de entrada, é necessário o conhecimento até a sexta derivada da posição da carga.

### Definição de Trajetórias para a Carga

Conhecida a planicidade diferencial do sistema, com o intuito de reduzir o balanço da carga, visa-se projetar uma trajetória estável para a carga para então obter o estado da aeronave e passá-lo como referência para o controlador. Se o controlador for capaz de seguir a referência com sucesso, espera-se que a carga cumpra sua trajetória programada.

Desse modo, projeta-se trajetórias polinomiais por partes de 13ª ordem que passam por pontos arbitrários em que as derivadas até a sexta ordem são contínuas (restrições deduzidas na seção anterior), sendo nulas nos pontos de interesse (APÊNDICE II -- INTERPOLAÇÃO POLINOMIAL POR PARTES). A Figura 4.2 apresenta a posição, velocidade e aceleração de um exemplo de trajetória com estas restrições para os pontos de passagem $q_{d} = \lbrack 3,0,2\rbrack$ e tempos $t_{d} = \lbrack 0,5,10\rbrack\ s$.

![](media/image25.emf){width="3.65625in" height="3.0632174103237095in"}

Figura 4.2 - Exemplo de trajetória polinomial ponto a ponto de 13° grau com derivadas nulas até a sexta ordem para os pontos de passagem $q_{d} = \lbrack 3,0,2\rbrack$ para os tempos $t_{d} = \lbrack 0,5,10\rbrack\ s$.

Observa-se que este tipo de interpolação gera curvas suaves com paradas bem marcantes nos pontos de interesse. Definindo trajetórias dessa forma no espaço tridimensional, tem-se retas entre os pontos.

A propriedade de planicidade diferencial permite encontrar o estado do *drone* necessário para que a carga tenha o comportamento desejado, porém não há garantia de que as trajetórias resultantes sejam compatíveis com as restrições do modelo, como a de tensão do cabo sempre positiva e de ângulos de guinada e arfagem inferiores a 90°, nem que sejam factíveis de serem executadas pelo controlador.

De fato, a estrutura de geração de trajetória polinomial definida não releva características do sistema em sua formulação, podendo produzir saídas dissonantes com a dinâmica do sistema.

Uma forma natural de solucionar este problema seria tratar a definição do estado da carga como um problema de otimização que considera restrições convenientes para o estado do *drone*. Porém, este processo não se demonstra trivial (CRUZ; FIERRO, 2017; SREENATH; LEE; KUMAR, 2013).

Alternativamente, propõe-se aplicar a técnica de *input shaping*, que se destaca pela simplicidade e efetividade em reduzir a oscilação na saída do sistema por meio do embutimento de oscilações no sinal de entrada que levam a frequência natural do sistema em consideração.

## Input Shaping

### Fundamentação Teórica

A técnica de *input shaping*, foi inspirada no trabalho de operadores de guindastes experientes que conseguem manobrar cargas sem provocar balanços excessivos simplesmente pressionando o botão de acionamento repetidas vezes em momentos específicos (SINGH; SINGHOSE, 2002). O princípio deste método consiste em introduzir um ou mais sinais impulsivos na entrada do sistema para gerar uma vibração na saída que é posteriormente compensada ao introduzir outros elementos impulsivos que provocariam uma vibração contrária (QIAN; YI, 2015). A Figura 4.3 ilustra a ideia para dois sinais impulsivos.

![](media/image26.emf){width="2.9895833333333335in" height="2.3519444444444444in"}

Figura 4.3 - Ilustração de resposta de um sistema sob a ação de dois impulsos devidamente selecionados utilizando técnica de *input shaping* (Adaptado de SINGH; SINGHOSE, 2002).

Na Figura 4.3, a curva em azul é a resposta do sistema quando apenas o impulso *A~1~* é aplicado. A curva em vermelho consiste na resposta do sistema quando apenas o impulso *A~2~* é aplicado. Ambos os impulsos geram uma oscilação de mesma frequência e taxa de decaimento, comportamento este esperado para sistemas de segunda ordem com amortecimento. *A~2~* está dimensionado exatamente no momento de inversão do movimento causado por $A_{1}$ produzindo uma resposta oposta. Esta configuração, por sua vez, gera uma trajetória sem oscilação como ilustrado pela linha em preto.

Para obter a sequência de dois impulsos como ilustrado na Figura 4.3, parte-se da descrição da vibração residual de um sistema de frequência natural $\omega_{n}$ e fator de amortecimento $\zeta$, dada por:

  ------------------------------------------------------------------------------------------------------------------- ----------
  $$V\left( \omega_{n},\zeta \right) = e^{- \zeta\omega\_ nt_{N}}\sqrt{C(\omega,\zeta)^{2} + S(\omega,\zeta)^{2}}$$   (.)

  ------------------------------------------------------------------------------------------------------------------- ----------

Em que:

  -------------------------------------------------------------------------------------------------------------------------- -----
  $$C\left( \omega_{n},\zeta \right) = \sum_{i = 1}^{I}{A_{i}e^{\zeta\omega_{n}t_{i}}\cos\left( \omega_{d}t_{i} \right)}$$   (.)

  $$S(\omega,\zeta) = \sum_{i = 1}^{N}{A_{i}e^{\zeta\omega_{n}t_{i}}\sin\left( \omega_{d}t_{i} \right)}$$                    (.)
  -------------------------------------------------------------------------------------------------------------------------- -----

$A_{i}$ e $t_{i}$ são as amplitudes e os tempos em que ocorrem os impulsos e $\omega_{d} = \omega_{n}\sqrt{1 - \zeta^{2}}$ é a frequência natural amortecida. Fazendo-se a porcentagem de vibração residual igual a zero, impondo a restrição $\sum A_{i} = 1$ e tomando a posição do primeiro impulso como $t_{1} = 0$, é possível determinar as amplitudes dos impulsos $(A_{1}$ e $A_{2}$) e o momento em que o segundo impulso ocorre : $t_{2}$ (BISGAARD; COUR-HARBO; BENDTSEN, 2008).

O filtro com dois impulsos apresentado é chamado de *zero vibration shaper* (ou *ZV shaper*). Porém, neste trabalho se opta por aplicar um modelador de três impulsos chamado *ZVD shaper* (*Zero Vibration and Derivative shaper*) em que se adiciona a restrição de derivada nula à vibração residual:

  ------------------------------------------------------------------------------------------------------------------------ ----------
  $$\frac{d}{d\omega_{n}}V\left( \omega_{n},\zeta \right) = 0,\ \ \frac{d}{d\zeta}V\left( \omega_{n},\zeta \right) = 0$$   (.)

  ------------------------------------------------------------------------------------------------------------------------ ----------

Assim, as amplitudes e os tempos dos impulsos são dados por:

  --------------------------------------------------------------------------------------------------------------------- -----
  $$t_{1} = 0,\ \ t_{2} = \frac{T_{d}}{2},\ \ t_{3} = \frac{T_{d}}{3}$$                                                 (.)

  $$A_{1} = \frac{1}{1 + 2K + K^{2}},\ \ A_{2} = \frac{2K}{1 + 2K + K^{2}},\ \ A_{3} = \frac{K^{2}}{1 + 2K + K^{2}}$$   (.)
  --------------------------------------------------------------------------------------------------------------------- -----

Sendo que $T_{d}$ é o período amortecido e $K$ é dado por:

  -------------------------------------------------------------------- ----------
  $$K = \exp\left( - \frac{\zeta\pi}{\sqrt{1 - \zeta^{2}}} \right)$$   (.)

  -------------------------------------------------------------------- ----------

Em comparação ao ZV *shaper*, o ZVD apresenta robustez significativamente maior quanto a incertezas quanto a frequência natural do sistema. Por outro lado, introduz um atraso maior na saída do sistema, que é a tendência à medida que se considerada modeladores de ordem maior (SINGH; SINGHOSE, 2002).

### Input shaping aplicado ao problema

Levando a aplicação desta técnica para o problema proposto, primeiramente deve-se determinar a frequência natural do sistema. Para isso, toma-se a frequência natural do balanço da carga, obtida das equações dinâmicas de $\phi_{L}$ e $\theta_{L}$ linearizadas em torno do ponto de equilíbrio, em que se obtém:

  ----------------------------------------------------------------- -----
  $$\omega_{n} = \sqrt{\frac{(M + m)g}{Ml}\ }$$                     (.)

  ----------------------------------------------------------------- -----

Visto que o controlador por modos deslizantes compensa o arrasto como uma parcela de distúrbio, toma-se o coeficiente de amortecimento $\zeta$ como nulo. Assim, considerando os parâmetros físicos padrões definidos na Seção 3.3, obtém-se os valores apresentados na Tabela 4.1:

Tabela 4.1 -- Parâmetros de *input shaping* para o sistema de exemplo.

  ---------------------------------------------------------------------------
  **Parâmetro**     **Valor**           **Parâmetro**     **Valor**
  ----------------- ------------------- ----------------- -------------------
  $$\omega_{n}$$    $$3,7279\ rad/s$$   $$\omega_{d}$$    $$3,7279\ rad/s$$

  $$\zeta$$         $$0$$               $$T_{d}$$         $$1,6856\ s$$

  $$A_{1}$$         $$0,25$$            $$t_{1}$$         $$0$$

  $$A_{2}$$         $$0,5$$             $$t_{2}$$         $$0,8428\ s$$

  $$A_{3}$$         $$0,25$$            $$t_{3}$$         $$1,6856\ s$$
  ---------------------------------------------------------------------------

Assim, para transferir este comportamento para sinais de entrada arbitrários, basta fazer a convolução da sequência de impulsos projetada no sinal de entrada do sistema. Para o problema em questão, primeiramente propõe-se definir pontos e ângulos de guinada por onde se deseja que o drone passe em determinados instantes de tempo, fazer uma interpolação polinomial restringindo velocidades e acelerações nulas nestes pontos, e então aplicar *input shaping* nas curvas resultantes da interpolação e passá-las como referência para o controlador. Neste caso, as referências para a orientação do drone são tomadas como nulas. Esta estratégia já foi utilizada com sucesso em outros trabalhos de controle de VANT's com carga suspensa por cabo como (BISGAARD; COUR-HARBO; BENDTSEN, 2008; KLAUSEN; FOSSEN; JOHANSEN, 2017)

Adicionalmente, também se propõe combinar esta técnica com a baseada na planicidade diferencial do sistema, como será apresentado na próxima seção.

## Trajetórias Baseadas na Planicidade Diferencial do Sistema com *Input Shaping*

A Figura 4.4 ilustra a nova estratégia de geração de trajetória proposta.

![](media/image27.emf){width="6.530064523184602in" height="1.1369575678040245in"}

Figura 4.4 - Estrutura da solução final de geração de trajetória.

Como ilustrado na Figura 4.4, a estratégia final de geração de trajetória recebe os pontos de parada desejados para a posição da carga e para o ângulo de guinada do drone como entrada e então realiza uma interpolação polinomial restringindo que todas as derivadas até a sexta ordem para a posição da carga e até a segunda ordem para o ângulo de guinada sejam nulas nestes pontos. Assim, aplica-se *input shaping* nas curvas resultantes da interpolação, obtendo as saídas planas que são utilizadas para calcular o estado desejado para o drone, que é passado como referência para o controlador (Figura 3.4, Seção 3.2).

Para exemplificar o efeito do *input shaping* na saída do gerador de trajetórias, define-se uma trajetória polinomial para a carga e para o ângulo de guinada do drone que parte do ponto ${{\overrightarrow{r}}_{L}}_{i} = \lbrack 0,0, - l\rbrack$ e $\psi_{i} = 0$ até ${{\overrightarrow{r}}_{L}}_{f} = \lbrack 2,2,1\rbrack$ $\psi_{f} = \frac{\pi}{3}$ ($60{^\circ}$) em 4 segundos, à qual se aplica *input shaping* com os parâmetros apresentados na Tabela 4.1, na Seção 4.2.2. Primeiramente, a Figura 4.5 apresenta a trajetória definida para cada componente da posição da carga, assim como da posição resultante do drone para os casos em que se aplica e quando não se aplica *input shaping* na entrada.

![](media/image28.emf){width="5.939129483814523in" height="4.827600612423447in"}

Figura 4.5 - Comparação entre uma trajetória polinomial ponto a ponto definida para posição da carga e a obtida para o drone segundo o modelo diferencialmente plano para as configurações em que se aplica e em que não se aplica *input shaping* na trajetória de entrada.

Na Figura 4.5, ao comparar a trajetória definida para a carga com a obtida para o drone, nota-se que esta apresenta uma leve frenagem aproximadamente na metade do tempo de subida. Em relação ao efeito do *input shaping*, observa-se que o filtro provoca uma certa inclinação das curvas, antecipando a aceleração e desaceleração do movimento no início e no final da trajetória. Também é possível verificar, principalmente no perfil da altura $z$, que o *input shaping* promove uma leve atenuação no efeito transiente. Estes efeitos também podem ser observados no espaço tridimensional, como mostra a Figura 4.6.

![](media/image29.emf){width="3.9791666666666665in" height="3.4308727034120734in"}

Figura 4.6 -- Visualização espacial da trajetória da carga e do drone para as configurações em que se aplica e não se aplica *input shaping* na trajetória de entrada.

A Figura 4.6 mostra que, sem aplicar *input shaping*, o drone executa uma manobra de amplitude maior, apresentando uma oscilação nítida na metade do trajeto aproximadamente. Quando se aplica o filtro, esta curva se aproxima mais a uma reta, porém é possível observar pequenas oscilações ao longo de todo o trajeto.

Por fim, vale observar o comportamento das outras variáveis do sistema. As Figuras 4.7, 4.8 e 4.9 apresentam o perfil das variáveis que descrevem a orientação do drone, os esforços de entrada e os ângulos de orientação do cabo, respectivamente.

De modo geral, nota-se que o *input shaping* promove uma redução significativa na amplitude das variáveis do sistema, com exceção do ângulo de guinada da aeronave, que segue um comportamento semelhante ao observado para a posição do drone, visto que é uma saída plana pré-definida como uma curva polinomial. Por exemplo, ao observar o primeiro gráfico da Figura 4.8, verifica-se que a força de propulsão $u_{1}$ atinge valores de até aproximadamente 45 N quando não se aplica *input shaping*, passando a assumir valores inferiores a 40 N ao aplicar o filtro. Por outro lado, verificou-se o surgimento de oscilações adicionais, porém de pouca intensidade, no comportamento dos ângulos de orientação do cabo, como mostra a Figura 4.9.

![](media/image30.emf){width="3.168744531933508in" height="2.8240004374453194in"}

Figura 4.7 -- Variáveis de estado que definem a orientação do drone obtidas por meio do modelo diferencialmente plano para quando se aplica e quando não se aplica *input shaping* em trajetória polinomial ponto a ponto definida para a posição carga.

![](media/image31.emf){width="3.4320002187226595in" height="3.81412510936133in"}

Figura 4.8 - Sinais de controle obtidos por meio do modelo diferencialmente plano para quando se aplica e quando não se aplica *input shaping* em trajetória polinomial ponto a ponto definida para a posição carga.

![](media/image32.emf){width="3.556314523184602in" height="2.6666666666666665in"}

Figura 4.9 - Variáveis que descrevem a orientação do cabo para quando se aplica e não se aplica *input shaping* em trajetória polinomial ponto a ponto definida para a posição carga.

V.

# CONTROLADOR COM GERADOR DE TRAJETÓRIA

Este capítulo apresenta uma análise do desempenho do controlador combinado com as técnicas de geração de trajetórias abordadas no cumprimento do objetivo proposto.

## Estrutura da Análise

Para avaliar o desempenho da solução final de controle, propõe-se uma análise comparativa entre três configurações diferentes:

I)  Trajetórias polinomiais para o drone;

II) Trajetórias polinomiais para o drone *input shaping*;

III) Trajetórias polinomiais para carga *input shaping* modelo diferencialmente plano.

Na configuração I, define-se trajetórias polinomiais ponto a ponto como referência de posição e guinada da aeronave (até a segunda derivada) para o controlador, mantendo as referências de ângulos de rolagem e arfagem nulas. Na configuração II, aplica-se *input shaping* às trajetórias definidas para o *drone* antes de enviar ao controlador. Por último, define-se as mesmas trajetórias definidas nas configurações I e II para a carga, porém com a coordenada $z$ deslocada para baixo o correspondente ao comprimento do cabo e as derivadas contínuas até a sexta ordem. Aplica-se *input shaping* a estas trajetórias e as utilizam como saídas planas do modelo diferencialmente plano, definindo as referências de trajetória da aeronave para alimentar o controlador

A Tabela 5.1 apresenta a lista de pontos de trajeto a serem passados como entrada dos interpoladores polinomiais para cada configuração, enquanto a Figura 5.1 mostra uma representação gráfica do caminho definido para a carga e para o drone.

Tabela 5.1 -- Pontos de referência do interpolador polinomial para as diferentes configurações de geração de trajetórias em comparação.

  ----------------------------------------------------------------------------------------------------------------------------------
  **Configuração**      **Variável**          **Valor**
  --------------------- --------------------- --------------------------------------------------------------------------------------
  I, II e III           $$x,x_{L}$$           $$\lbrack 0,4,4,1,0\rbrack\ m$$

  I, II e III           $$y,\ y_{L}$$         $$\lbrack 0,6,6,9,0\rbrack\ m$$

  I e II                $$z$$                 $$\lbrack 0,5,5,2,0\rbrack\ m$$

  III                   $$z_{L}$$             $$\lbrack - 1,\ 4,\ 4,\ 1, - 1\rbrack\ m$$

  I, II e III           $$\psi$$              $$\left\lbrack 0,\frac{\pi}{3},\frac{\pi}{3}, - \frac{\pi}{4},0 \right\rbrack\ rad$$

  I e II                $$\phi,\theta$$       $$\lbrack 0,0,0,0,0\rbrack\ rad$$
  ----------------------------------------------------------------------------------------------------------------------------------

![](media/image33.emf){width="5.0365791776028in" height="4.1337084426946635in"}

Figura 5.1 -- Trajeto de referência de teste para o drone (configurações I e II) e para a carga (configuração III)

Assume-se que estes pontos ocorrem em tempos igualmente espaçados com um tempo de espera adicional de 3 segundos no início e um tempo adicional para acomodação de 10 segundos no final. Assim, realiza-se a simulação do sistema para cada configuração, variando-se o tempo total da manobra (tirando o tempo de espera e o de acomodação) com o intuito de verificar o desempenho do sistema para diferentes níveis de agressividade de manobra. Especificamente, considera a execução da manobra em $T = \lbrack 18,\ 15,\ 12,\ 10\rbrack\ s$. São adotados os mesmos parâmetros de simulação especificados na Seção 3.3. Porém, a fim de testar a robustez do controlador contra incerteza de parâmetros, aplica-se variações fixas nos parâmetros físicos segundo apresenta a Tabela 5.2.

Tabela 5.2 -- Perturbação fixa imposta sobre os parâmetros do modelo.

  -------------------------------------------------------------------------------------
  **Parâmetro**     **Perturbação**     **Parâmetro**                 **Perturbação**
  ----------------- ------------------- ----------------------------- -----------------
  $$M$$             $$- 15\%$$          $$I_{x},I_{y},I_{z}$$         $$+ 30\%$$

  $$m$$             $$+ 20\%$$          $$c_{x},c_{y},c_{z},c_{L}$$   $$+ 10\%$$

  $$l$$             $$- 10\%$$          $$-$$                         $$-$$
  -------------------------------------------------------------------------------------

Para comparar os resultados, define-se um conjunto de métricas calculadas sobre as variáveis de saída da simulação amostradas em intervalos 0,01 s. A Tabela 5.3 lista as métricas utilizadas para fazer a comparação.

Tabela 5.3 -- Lista de métricas de comparação do comportamento do sistema controlado sob diferentes configurações de geração de trajetória.

  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
  **Símbolo**                   **Métrica**
  ----------------------------- ---------------------------------------------------------------------------------------------------------------------------------------
  $$r_{RMS}$$                   Valor eficaz do erro de posição do drone.

  $$\beta_{RMS}$$               Valor eficaz do ângulo entre o eixo vertical inercial (${\overrightarrow{e}}_{z}$) e o não inercial (${\overrightarrow{e}}_{z}^{b}$).

  $$\alpha_{RMS}$$              Valor eficaz do ângulo do cabo em relação à vertical durante o tempo de acomodação.

  $${\overline{f}}_{\omega}$$   Frequência média do módulo da frequência angular $\overrightarrow{\omega}$
  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------

A primeira medida ($r_{RMS}$) visa quantificar o erro médio do controle de posição da aeronave. O valor $\beta_{RMS}$ busca acessar o grau de inclinação médio do *drone* ao longo de percurso. Quando $\beta_{RMS}$ é maior, significa que o *drone* apresentou ângulos de rolagem e guinada maiores e por mais tempo comparativamente. A medida $\alpha_{RMS}$ visa quantificar o grau de oscilação da carga após a execução do trajeto planejado, valendo idealmente zero. Por fim, ${\overline{f}}_{\omega}$ capta o grau de oscilação da atitude do *drone* em termos de frequência.

## Análise dos Resultados

A Tabela 5.4 apresenta o resultado dos cálculos dos parâmetros de análise para cada cenário de simulação.

Tabela 5.4 -- Quadro comparativo de desempenho com base em métricas entre as configurações de geração de trajetória em análise para diferentes tempos de execução da trajetória de referência.

<table>
<colgroup>
<col style="width: 11%" />
<col style="width: 9%" />
<col style="width: 11%" />
<col style="width: 11%" />
<col style="width: 9%" />
<col style="width: 1%" />
<col style="width: 10%" />
<col style="width: 11%" />
<col style="width: 11%" />
<col style="width: 10%" />
</colgroup>
<thead>
<tr class="header">
<th><span class="math inline"><strong>T</strong></span></th>
<th colspan="4"><strong>18 s</strong></th>
<th colspan="5"><strong>15 s</strong></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td><strong>Métrica</strong></td>
<td><p><span class="math display"><strong>r</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[m]</p></td>
<td><p><span class="math display"><strong>β</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[°]</p></td>
<td><p><span class="math display"><strong>α</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[°]</p></td>
<td colspan="2"><p><span class="math display">$${\overline{\mathbf{f}}}_{\mathbf{\omega}}$$</span></p>
<p>[hz]</p></td>
<td><p><span class="math display"><strong>r</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[m]</p></td>
<td><p><span class="math display"><strong>β</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[°]</p></td>
<td><p><span class="math display"><strong>α</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[°]</p></td>
<td><p><span class="math display">$${\overline{\mathbf{f}}}_{\mathbf{\omega}}$$</span></p>
<p>[hz]</p></td>
</tr>
<tr class="even">
<td>I</td>
<td>0,0347</td>
<td>6,8638</td>
<td>14,8947</td>
<td colspan="2">0,1621</td>
<td>0,0330</td>
<td>8,5778</td>
<td>3,5353</td>
<td>0,1694</td>
</tr>
<tr class="odd">
<td>II</td>
<td>0,0230</td>
<td>4,8082</td>
<td>1,6924</td>
<td colspan="2">0,0788</td>
<td>0,0273</td>
<td>6,2406</td>
<td>1,1159</td>
<td>0,1083</td>
</tr>
<tr class="even">
<td>III</td>
<td>0,0119</td>
<td>5,7702</td>
<td>1,0048</td>
<td colspan="2">0,0704</td>
<td>0,0147</td>
<td>6,7651</td>
<td>1,6136</td>
<td>0,0858</td>
</tr>
<tr class="odd">
<td><span class="math display"><strong>T</strong></span></td>
<td colspan="4"><strong>12 s</strong></td>
<td colspan="5"><strong>10 s</strong></td>
</tr>
<tr class="even">
<td><strong>Métrica</strong></td>
<td><p><span class="math display"><strong>r</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[m]</p></td>
<td><p><span class="math display"><strong>β</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[°]</p></td>
<td><p><span class="math display"><strong>α</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[°]</p></td>
<td colspan="2"><p><span class="math display">$${\overline{\mathbf{f}}}_{\mathbf{\omega}}$$</span></p>
<p>[hz]</p></td>
<td><p><span class="math display"><strong>r</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[m]</p></td>
<td><p><span class="math display"><strong>β</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[°]</p></td>
<td><p><span class="math display"><strong>α</strong><sub><strong>R</strong><strong>M</strong><strong>S</strong></sub></span></p>
<p>[°]</p></td>
<td><p><span class="math display">$${\overline{\mathbf{f}}}_{\mathbf{\omega}}$$</span></p>
<p>[hz]</p></td>
</tr>
<tr class="odd">
<td>I</td>
<td>0,1029</td>
<td>17,7995</td>
<td>67,2285</td>
<td colspan="2">0,4421</td>
<td>0,1073</td>
<td>19,1869</td>
<td>46,0548</td>
<td>0,5544</td>
</tr>
<tr class="even">
<td>II</td>
<td>0,0352</td>
<td>8,5118</td>
<td>8,8411</td>
<td colspan="2">0,1657</td>
<td>0,0521</td>
<td>11,7518</td>
<td>26,8252</td>
<td>0,3355</td>
</tr>
<tr class="odd">
<td>III</td>
<td>0,0189</td>
<td>8,0322</td>
<td>3,5985</td>
<td colspan="2">0,1449</td>
<td>0,0465</td>
<td>10,3182</td>
<td>5,3366</td>
<td>0,5213</td>
</tr>
</tbody>
</table>

Analisando os resultados da Tabela 5.4 globalmente, observa-se que a configuração I, a qual não há nenhuma estratégia explícita de redução do balanço da carga, é a que apresenta o maior erro de posição ($\alpha_{RMS}$) e intensidade de oscilação de carga em todas as simulações. Também se observa que a configuração III, que leva em consideração o modelo diferencialmente plano, apresenta o menor erro de posição e grau de da oscilação da carga em todas as simulações. Apesar das diferenças numéricas do erro de posição ($r_{RMS}$), constata-se que ele é baixo para todos os cenários (no máximo 10 cm), demonstrando a eficiência do controlador no cumprimento do seu objetivo.

Para auxiliar na continuação da análise, as Figuras 5.2, 5.3 e 5.4 apresentam o trajeto percorrido pelo *drone* e pela carga para os resultados das simulações com trajeto de 15 e de 10 segundos para as três configurações.

![](media/image34.png){width="2.2472222222222222in" height="0.6972222222222222in"}![](media/image36.emf){width="3.248031496062992in" height="2.9022058180227472in"}![](media/image37.emf){width="3.248031496062992in" height="2.767594050743657in"}

Figura 5.2 - Caminho percorrido pelo drone e pela carga para a simulação de 15 e 10 segundos com a configuração I.

![](media/image34.png){width="2.2472222222222222in" height="0.6972222222222222in"}![](media/image38.emf){width="3.248031496062992in" height="2.9022058180227472in"}![](media/image39.emf){width="3.248031496062992in" height="2.874300087489064in"}

Figura 5.3 - Caminho percorrido pelo drone e pela carga para a simulação de 15 e 10 segundos com a configuração II.

![](media/image34.png){width="2.2472222222222222in" height="0.6972222222222222in"}![](media/image40.emf){width="3.248031496062992in" height="2.916362642169729in"}![](media/image41.emf){width="3.248031496062992in" height="2.9450951443569555in"}

Figura 5.4 - Caminho percorrido pelo drone e pela carga para a simulação de 15 e 10 segundos com a configuração III.

Ao analisar a Figura 5.2, em que não se aplica uma estratégia especializada de geração de trajetória (configuração I), reforça-se a conclusão de que, embora o controlador seja capaz de manter a posição do drone estável, a carga balança de forma intensa, especialmente para a manobra mais agressiva de 10 segundos.

Embora o controlador com *input shaping* (Figura 5.3) não tenha apresentado o menor $\alpha_{RMS}$ em nenhuma das simulações, ele apresentou desempenho prático convincente neste quesito. Ao comparar com a configuração em que não se aplica *input shaping* (Figura 5.2), verifica-se que a técnica foi eficaz em promover atenuação do balanço da carga, apresentando desempenho semelhante à configuração III para as manobras de 18, 15 e 12 segundos (Tabela 5.4). Porém, apresentou desempenho consideravelmente inferior a este na simulação de 10 segundos, apresentando $\alpha_{RMS}$ cinco vezes superior, resultado este refletido no balanço final observado na Figura 5.4. Ressalta-se que o desvio vertical da posição carga em relação à referência observado entre os pontos de passagem é inerente do movimento da carga e que o abaulamento do trajeto do drone no segundo ponto de passagem é consequência da aplicação do *input shaping*.

Quando se aplica *input shaping* na trajetória da carga (Figura 5.4), não é possível perceber a presença de oscilações nos pontos extremos mesmo na simulação de 10 segundos. Nota-se um abaulamento no trajeto da carga ao passar pelo segundo ponto da trajetória na simulação de 10 segundos, mas ele ocorre devido à aplicação do *input shaping*. Também é possível observar o desvio de trajeto que o *drone* faz em relação às retas que liga os pontos de passagem, especialmente para a manobra mais agressiva.

O comportamento de atenuação da carga fica mais evidente ao analisar graficamente o estado da orientação do cabo. A Figura 5.5 apresenta o comportamento das variáveis que descrevem a orientação do cabo para a simulação de 12 segundos de trajetória com as configurações I, II e III.

![](media/image42.emf){width="2.057303149606299in" height="3.1950853018372705in"}![](media/image43.emf){width="2.168115704286964in" height="3.0000820209973753in"}![](media/image44.emf){width="2.1338582677165356in" height="3.0486909448818897in"}

Figura 5.5 - Estado da orientação do cabo para a simulação de 12 segundos de trajetória com as configurações I (sem *input shaping*), II (com *input shaping*) e III (com *input shaping* e modelo diferencialmente plano)

Observa-se que, depois da linha vertical pontilhada, a partir de onde se mede $\alpha_{RMS}$, o caso em que não se aplica nenhuma técnica de geração de trajetória apresenta oscilações superiores a $90{^\circ}$ para $\phi_{L}$ e quase atingindo o ponto de singularidade de $\theta_{L}$. Quando se aplica *input shaping* (configuração II) e o modelo diferencialmente plano (configuração III), o balanço é significativamente suprimido.

Para auxiliar na análise do desempenho de controle de atitude do drone, as Figuras 5.6 e 5.7 apresentam o comportamento das variáveis que descrevem a orientação do drone para as simulações de 15 e 10 segundos com as configurações II e III, respectivamente.

![](media/image45.emf){width="2.704724409448819in" height="4.810877077865267in"} ![](media/image46.emf){width="2.678472222222222in" height="4.803928258967629in"}

Figura 5.6 - Comportamento de atitude da aeronave para as simulações de 15 e 10 segundos com a configuração II.

![](media/image47.emf){width="2.4606299212598426in" height="4.410337926509186in"}![](media/image48.emf){width="2.4606299212598426in" height="4.424361329833771in"}

Figura 5.7 -- Comportamento de atitude da aeronave para as simulação de 15 e 10 segundos com a configuração III.

Comparando as Figuras 5.6 e 5.7, verifica-se que, para a simulação de 15 segundos, embora o controlador apenas com *input shaping* (Figura 5.6) receba uma referência nula para os ângulos de rolagem e arfagem, estas variáveis apresentam um comportamento semelhante ao segundo, quando se tem trajetórias bem definidas com base no modelo diferencialmente plano.

Na simulação de 10 segundos, a segunda solução apresentou um comportamento mais agressivo e de maior amplitude, que também reflete na Tabela 5.4 por meio das métricas $\beta_{RMS}$ e ${\overline{f}}_{\omega}$. Embora a simulação tenha convergido para esta situação, é provável que este perfil de atitude não seja infactível em aplicações reais. Neste cenário, a primeira solução (configuração II) atua com uma amplitude menor, porém apresenta um ruído de alta frequência que se torna mais nítido ao observar as velocidades angulares, como mostram as Figuras 5.8 e 5.9.

![](media/image45.emf){width="2.2440944881889764in" height="3.918964348206474in"}![](media/image46.emf){width="2.2440944881889764in" height="3.9441951006124234in"}

Figura 5.8 -- Velocidade angular do drone para as simulações de 15s e 10s com a configuração II.

![](media/image47.emf){width="2.2440944881889764in" height="3.918419728783902in"}![](media/image48.emf){width="2.2440944881889764in" height="3.9244477252843395in"}

Figura 5.9 - Velocidade angular do drone para as simulações de 15s e 10s com a configuração III.

As componentes de alta frequência observadas na Figura 5.8 justificam os valores superiores de ${\overline{f}}_{\omega}$ obtidos para as simulações de 18, 15 e 12 segundos (Tabela 5.4). Este efeito pode estar associado ao fato de que é passado referência nula à atitude da aeronave, pois assim o erro de atitude se distancia de zero com mais frequência, constantemente ativando os termos descontínuos dos sinais de entrada.

Por fim, vale comparar o sinal de entrada obtido para as configurações II e III. As Figuras 5.10 e 5.11 apresentam os esforços produzidos pelo controlador para as simulações de 15 e 10 segundos de trajetória, sendo que para a configuração III (Figura 5.11), também se apresenta os esforços ideais obtidos por meio do modelo diferencialmente plano.

![](media/image49.emf){width="3.0746369203849517in" height="4.537312992125984in"} ![](media/image50.emf){width="3.082620297462817in" height="4.544662073490814in"}

Figura 5.10 -- Esforços de controle para as simulações de 15s e 10s com a configuração II.

![](media/image51.emf){width="3.174835958005249in" height="5.292451881014873in"}![](media/image52.emf){width="3.1740879265091864in" height="5.29120406824147in"}

Figura 5.11 - Esforços de controle para as simulações de 15s e 10s com a configuração III.

Primeiramente, observa-se na Figura 5.10 que a força de propulsão $u_{1}$ apresenta oscilações consideráveis depois que atinge o último ponto da trajetória. Este comportamento pode indicar que os parâmetros de controle definidos não garantiram robustez contra as perturbações aplicadas aos parâmetros físicos do problema, especialmente às massas do *drone* e da carga, que são diretamente atreladas à força de propulsão. A influência da variação dos parâmetros físicos também pode ser observada na Figura 5.12, em que a força de propulsão prevista pelo modelo diferencialmente plano é levemente inferior ao realizado, visto que o valor de massa total do conjunto considerada pelo controlador é menor do que a real, segundo as configurações de incerteza listadas na Tabela 5.2 da Seção 5.1. A compensação desta incerteza é atingida pela robustez do sistema.

Também é possível observar que o torque produzido pela solução III (Figura 5.10) apresenta algumas osciçãoes em alta frequência em torno de valores baixos para os dois tempos de manobra, mas que chegam a atingir picos superiores aos observados no comportamento da configuração IV (Figura 5.11) para a trajetória de 15 segundos. Este comportamento influncia o comportamento da velocidade angular (Figura 5.8) e na métrica ${\overline{f}}_{\omega}$ da Tabela 5.4.

Analisando a Figura 5.11, verifica-se que os sinais obtidos pelo modelo diferencialmente plano e as calculadas pelo controlador são bem próximas, especialmente para a manobra menos agressiva. Porém, observa-se que o torque de guinada executado na simulação de 10 segundos não atinge os picos previstos pelo modelo. Este desvio é coerente com os pequenos desvios observados no perfil de posição e velocidade angulares de guinada mostradas nas Figuras 5.7 e 5.9. Este comportamento pode estar associado a uma insuficiência dos parâmetros de controle definidos, especialmente $\kappa_{\psi}$ e $\eta_{\psi}$, que estão diretamente ligados à robustez contra distúrbios externos e incertezas internas ao longo deste grau de liberdade.

Em suma, verificou-se que o controlador desempenha bem o papel de controlar a posição do *drone* mesmo em condições de balanço intenso da carga, mas não contém as oscilações para trajetórias definidas arbitrariamente. As soluções de geração de trajetória designadas a contornar este problema apresentaram bom desepenho no papel de atenuar a oscilação da carga, com destaque para a configuração em que se agrega a propriedade de planicidade diferencial do sistema. Porém, esta solução tendeu a degradar o trajeto de referência da aeronave a partir de certo grau de velocidade e aceleração de manobra (não determinado neste trabalho, mas que pode ser explorado futuramente). A alternativa de aplicar apenas *input shaping*, embora tenha desempenhado pior em relação à supressão do balanço da carga, gerado ruídos nas manobras mais lentas e oscilação no sinal de propulsão na manobra mais rápida, executou trajetórias mais suaves no último cenário, sendo mais factíveis de serem realizadas.

É importante ressaltar que não é possível concluir que a solução com o modelo diferencialmente plano sempre produzirá resultados melhores do que quando se aplica apenas *input shaping*. Propõe-se avaliar o desempenho da solução para outras trajetórias e configurações de inerteza, além de comparar o desempenho da solução desenvolvida com outras técnicas similares.

VI.

# CONCLUSÕES

Primeiramente, o presente trabalho desenvolve um modelo dinâmico do sistema drone com carga suspensa por cabo, explicitando as equações de aceleração da aeronave e dos ângulos que descrevem a orientação do cabo de modo que possa ser utilizado pelo controlador desenvolvido.

Em seguida, apresenta-se a derivação da solução de controle por modos deslizantes (CMD) desenvolvida com foco no desafio de enfrentar a sub-atuação do sistema com proveito do conhecimento do modelo dinâmico. Define-se um CMD clássico para controlar a altitude e ângulo de guinada da aeronave e outro que leva em conta a característica de sub-atuação da dinâmica de deslocamento horizontal da aeronave de forma explícita. Demonstra-se por meio do conceito de estabilidade de *Lyapunov* que os sub-sistemas são estáveis sob o efeito de distúrbios cujos limiares sejam conhecidos. Também foi demonstrado que o sistema é localmente estável no deslocamento horizontal por meio do critério de estabilidade de *Routh-Hurwitz* aplicado ao sistema em fase de deslizamento e linearizado em torno do ponto de equilíbrio, deduzindo-se restrições para as constantes de controle e valores referência de atitude. Com isso, analisou-se o comportamento de convergência do controlador para alguns cenários característicos, como nos casos em que as condições deduzidas para os parâmetros de controle não são satisfeitas.

Para reduzir o balanço da carga e, consequentemente, o seu efeito na dinâmica da aeronave, investe-se em técnicas de geração de trajetória com base na propriedade de planicidade diferencial do sistema e *input shaping*. Constatou-se que, ao definir trajetórias polinomiais ponto a ponto com derivadas nulas nos pontos de passagem da carga, o modelo diferencialmente plano gera trajetórias de referência válidas para a aeronave, mas que tendem a oscilar de forma proibitiva para o controlador à medida que se aumenta as velocidades e as acelerações demandadas. Assim, como alternativa a soluções de otimização mais sofisticadas, propôs-se a aplicação de *input shaping* no trajeto definido para a carga, embutindo o conhecimento da dinâmica de balanço na entrada. Observou-se que esta estratégia levou a uma suavização das trajetórias resultantes do *drone*, viabilizando a realização de manobras com velocidades maiores.

Enfim, avaliou-se o desempenho do controlador na execução de uma trajetória ponto a ponto definida por interpolação polinomial, aplicando-se diferentes configurações de tratamento destas trajetórias e variando-se o tempo de manobra. Na primeira configuração, a curva de entrada é passada diretamente como referência para o drone, depois se aplica *input shaping* nesta curva e a repassa como referência da aeronave e, por último, aplica-se uma estratégia combinada, em que se aplica *input shaping* na trajetória definida para a carga para depois aplicar o modelo diferencialmente plano e gerar referências para a aeronave.

Constatou-se que o controlador é capaz de estabilizar a posição da aeronave em todos os casos avaliados, cumprindo o seu objetivo primário. Porém, a carga apresenta oscilações intensas quando não se trata o sinal de referência base. As soluções que aplicam *input shaping* promovem uma atenuação significativa no balanço da carga. Dentre estas soluções, a que leva em consideração o modelo diferencialmente plano desempenhou-se melhor, especialmente para as manobras mais lentas, apresentando melhor fator de atenuação, menor erro de posição e taxa de variação de atitude da aeronave. Porém, seu desempenho cai a partir de determinado limiar de velocidade de manobra, quando o sinal de referência resultante para o *drone* assume perfis demasiadamente agressivos para o controlador. A solução em que se aplica *input shaping* diretamente na trajetória da aeronave apresenta melhor resultado neste cenário. Na realidade, esta configuração é a que apresentou maior constância entre os diferentes tempos de manobra com desempenho satisfatório em relação à atenuação da carga, embora tenha apresentado componentes de alta frequência no estado de atitude da aeronave em trajetos de tempo maior.

Por fim, é importante levantar alguns pontos que ainda podem ser explorados futuramente para consolidar a solução proposta. Como já apresentado ao longo do texto, primeiramente, busca-se levantar as condições em que a combinação de *input shaping* com o modelo diferencialmente plano passa a gerar trajetórias proibitivas, avaliando-se o desempenho da solução para outras trajetórias e na presença de diferentes configurações de distúrbios externos e incerteza de parâmetros. Também é válido comparar o desempenho do controlador e gerador de trajetórias com outras técnicas similares da literatura.

Dentre outras inúmeras oportunidades possíveis de avanço da pesquisa, sugere-se abranger mais detalhes no modelo dinâmico. Pode-se abdicar das aproximações feitas para o controlador sobre as velocidades angulares, considerar as condições em que o cabo não está tensionado (CROUSAZ et al., 2015; SREENATH; MICHAEL; KUMAR, 2013) e levar em conta as perturbações de momento na aeronave quando o cabo não é preso exatamente no seu centro de massa (CRUZ; FIERRO, 2017; PALUNKO; FIERRO; CRUZ, 2012; REGO; RAFFO, 2019; SANTOS et al., 2017). Além disso, sugere-se atualizar o controlador no sentido de reduzir o efeito de *chattering* de forma mais efetiva aplicando-se CMD de segunda ordem (SHTESSEL et al., 2013)e de considerar o erro de orientação da carga a fim de promover uma atenuação ativa do balanço e não depender totalmente de soluções de malha aberta. Enfim, propõe-se explorar o problema de estimação de estados, especialmente da posição da carga, normalmente realizado com o auxílio de sensores adicionais como câmeras (TANG; WÜEST; KUMAR, 2018; ZÜRN et al., 2016) e *encoders* (KLAUSEN; FOSSEN; JOHANSEN, 2017), para então partir para testes com o sistema real.

# REFERÊNCIAS BIBLIOGRÁFICAS

ALEXIS, K. et al. Robust Model Predictive Flight Control of Unmanned Rotorcrafts. **Journal of Intelligent & Robotic Systems**, v. 81, n. 3--4, p. 443--469, 1 mar. 2016. <https://doi.org/10.1007/s10846-015-0238-7>.

ASHRAFIUON, H.; ERWIN, R. S. **Sliding control approach to underactuated multibody systems**. Proceedings of the 2004 American Control Conference. **Anais**\... In: PROCEEDINGS OF THE 2004 AMERICAN CONTROL CONFERENCE. jun. 2004. <https://doi.org/10.23919/ACC.2004.1386750>.

ASHRAFIUON, H.; ERWIN, R. S. Sliding mode control of underactuated multibody systems and its application to shape change control. **International Journal of Control**, v. 81, n. 12, p. 1849--1858, 1 dez. 2008. <https://doi.org/10.1080/00207170801910409>.

BISGAARD, M. **Modeling, Estimation, and Control of Helicopter Slung Load System**. \[s.l.\] Department of Control Engineering, Aalborg University, 2008. <https://doi.org/10.2514/6.2007-6762>.

BISGAARD, M.; COUR-HARBO, A. LA; BENDTSEN, J. D. **Input Shaping for Helicopter Slung Load Swing Reduction**. AIAA Guidance, Navigation and Control Conference and Exhibit. **Anais**\... In: AIAA GUIDANCE, NAVIGATION AND CONTROL CONFERENCE AND EXHIBIT. Honolulu, Hawaii: American Institute of Aeronautics and Astronautics, 18 ago. 2008. <https://doi.org/10.2514/6.2008-6964>.

BISGAARD, M.; LA COUR-HARBO, A.; DIMON BENDTSEN, J. Adaptive control system for autonomous helicopter slung load operations. **Control Engineering Practice**, Special Issue on Aerial Robotics. v. 18, n. 7, p. 800--811, 1 jul. 2010.

<https://doi.org/10.1016/j.conengprac.2010.01.017>.

BRESCIANI, T. **Modelling, Identification and Control of a Quadrotor Helicopter**. MSc Theses, 2008. Disponível em: <http://lup.lub.lu.se/student-papers/record/8847641>.

CROUSAZ, C. D.; FARSHIDIAN, F.; BUCHLI, J. **Aggressive optimal control for agile flight with a slung load**. in IROS 2014 Workshop on Machine Learning in Planning and Control of Robot Motion. **Anais**\...2014. Disponível em:

<https://www.cs.unm.edu/amprg/mlpc14Workshop/submissions/mlpc2014_submission_7.pdf>. Acesso em: 15 set. 2019.

CROUSAZ, C. DE et al. **Unified motion control for dynamic quadrotor maneuvers demonstrated on slung load and rotor failure tasks**. 2015 IEEE International Conference on Robotics and Automation (ICRA). **Anais**\... In: 2015 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION (ICRA). maio 2015.

<https://doi.org/10.1109/ICRA.2015.7139493>.

CRUZ, P. J.; FIERRO, R. Cable-suspended load lifting by a quadrotor UAV: hybrid model, trajectory generation, and control. **Autonomous Robots**, v. 41, n. 8, p. 1629--1643, 1 dez. 2017. <https://doi.org/10.1007/s10514-017-9632-2>.

DE ALMEIDA, M. M.; RAFFO, G. V. Nonlinear Control of a TiltRotor UAV for Load Transportation\*\*The authors would like to thank the Brazilian research agencies CAPES, CNPq and FAPEMIG for their financial contribution for the accomplishment of this work. **IFAC-PapersOnLine**, 11th IFAC Symposium on Robot Control SYROCO 2015. v. 48, n. 19, p. 232--237, 1 jan. 2015. <https://doi.org/10.1016/j.ifacol.2015.12.039>.

DING, X. et al. A review of aerial manipulation of small-scale rotorcraft unmanned robotic systems. **Chinese Journal of Aeronautics**, 22 jun. 2018.

<https://doi.org/10.1016/j.cja.2018.05.012>

FAUST, A. et al. **Learning swing-free trajectories for UAVs with a suspended load**. 2013 IEEE International Conference on Robotics and Automation. **Anais**\... In: 2013 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION. maio 2013. <https://doi.org/10.1109/ICRA.2013.6631277>.

FAUST, A. et al. Automated aerial suspended cargo delivery through reinforcement learning. **Artificial Intelligence**, Special Issue on AI and Robotics. v. 247, p. 381--398, 1 jun. 2017. <https://doi.org/10.1016/j.artint.2014.11.009>.

FLIESS, M. et al. ON DIFFERENTIALLY FLAT NONLINEAR SYSTEMS. In: FLIESS, M. (Ed.). . **Nonlinear Control Systems Design 1992**. IFAC Symposia Series. Oxford: Pergamon, 1993. p. 159--163. <https://doi.org/10.1016/B978-0-08-041901-5.50031-2>.

FREDDI, A.; LANZON, A.; LONGHI, S. A Feedback Linearization Approach to Fault Tolerance in Quadrotor Vehicles. **IFAC Proceedings Volumes**, 18th IFAC World Congress. v. 44, n. 1, p. 5413--5418, 1 jan. 2011. <https://doi.org/10.3182/20110828-6-IT-1002.02016>.

GHAZALI, R. et al. Performance Comparison between Sliding Mode Control with PID Sliding Surface and PID Controller for an Electro-hydraulic Positioning System. **International Journal on Advanced Science, Engineering and Information Technology**, v. 1, n. 4, p. 447-452--452, 2011. <https://doi.org/10.18517/ijaseit.1.4.91>.

GOODARZI, F. A.; LEE, D.; LEE, T. **Geometric stabilization of a quadrotor UAV with a payload connected by flexible cable**. 2014 American Control Conference. **Anais**\... In: 2014 AMERICAN CONTROL CONFERENCE. jun. 2014.

<https://doi.org/10.1109/ACC.2014.6859419>.

GUERRERO, M. E. et al. **IDA-PBC methodology for a quadrotor UAV transporting a cable-suspended payload**. 2015 International Conference on Unmanned Aircraft Systems (ICUAS). **Anais**\... In: 2015 INTERNATIONAL CONFERENCE ON UNMANNED AIRCRAFT SYSTEMS (ICUAS). Denver, CO, USA: IEEE, jun. 2015a.

<https://doi.org/10.1109/ICUAS.2015.7152325>.

GUERRERO, M. E. et al. **Passivity based control for a quadrotor UAV transporting a cable-suspended payload with minimum swing**. 2015 54th IEEE Conference on Decision and Control (CDC). **Anais**\... In: 2015 54TH IEEE CONFERENCE ON DECISION AND CONTROL (CDC). dez. 2015b. <https://doi.org/10.1109/CDC.2015.7403277>.

GUERRERO-SÁNCHEZ, M. E. et al. Swing-attenuation for a quadrotor transporting a cable-suspended payload. **ISA Transactions**, v. 68, p. 433--449, 1 maio 2017.

<https://doi.org/10.1016/j.isatra.2017.01.027>.

HOSSAIN, E. et al. Sliding Mode Controller and Lyapunov Redesign Controller to Improve Microgrid Stability: A Comparative Analysis with CPL Power Variation. **Energies**, v. 10, n. 12, p. 1959, dez. 2017. <https://doi.org/10.3390/en10121959>.

JEAONG, H. et al. **Simulation and Flight Experiment of a Quadrotor Using Disturbance Observer Based Control**. 2018. Disponível em:

<http://www.imavs.org/papers/2018/IMAV_2018_paper_15.pdf>. Acesso em: 15 set. 2019.

KLAUSEN, K.; FOSSEN, T. I.; JOHANSEN, T. A. **Nonlinear control of a multirotor UAV with suspended load**. 2015 International Conference on Unmanned Aircraft Systems (ICUAS). **Anais**\... In: 2015 INTERNATIONAL CONFERENCE ON UNMANNED AIRCRAFT SYSTEMS (ICUAS). Denver, CO, USA: IEEE, jun. 2015. <https://doi.org/10.1109/ICUAS.2015.7152289>.

KLAUSEN, K.; FOSSEN, T. I.; JOHANSEN, T. A. Nonlinear Control with Swing Damping of a Multirotor UAV with Suspended Load. **Journal of Intelligent & Robotic Systems**, v. 88, n. 2--4, p. 379--394, 1 dez. 2017. <https://doi.org/10.1007/s10846-017-0509-6>.

KOTARU, P.; WU, G.; SREENATH, K. **Dynamics and control of a quadrotor with a payload suspended through an elastic cable**. 2017 American Control Conference (ACC). **Anais**\... In: 2017 AMERICAN CONTROL CONFERENCE (ACC). maio 2017.

<https://doi.org/10.23919/ACC.2017.7963553>.

KUI, Y. et al. **Sliding mode control for a quadrotor slung load system**. 2017 36th Chinese Control Conference (CCC). **Anais**\... In: 2017 36TH CHINESE CONTROL CONFERENCE (CCC). jul. 2017. <https://doi.org/10.23919/ChiCC.2017.8027934>.

LEE, T.; LEOK, M.; MCCLAMROCH, N. H. **Geometric tracking control of a quadrotor UAV on SE(3)**. 49th IEEE Conference on Decision and Control (CDC). **Anais**\... In: 49TH IEEE CONFERENCE ON DECISION AND CONTROL (CDC). dez. 2010. <https://doi.org/10.1109/CDC.2010.5717652>.

MELLINGER, D. **Trajectory generation and control for quadrotors**. Publicly Accessible Penn Dissertations. University of Pennsylvania, 2012. Disponível em:

<https://repository.upenn.edu/edissertations/547/>. Acesso em: 15 set. 2019.

MELLINGER, D.; KUMAR, V. **Minimum snap trajectory generation and control for quadrotors**. 2011 IEEE International Conference on Robotics and Automation. **Anais**\... In: 2011 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION. Shanghai, China: IEEE, maio 2011. <https://doi.org/10.1109/ICRA.2011.5980409>.

NISE, N. S. **Control Systems Engineering**. 6th ed ed. Hoboken, NJ: John Wiley & Sons, Incorporated, 2011.

NOTTER, S. et al. Modelling, Simulation and Flight Test of a Model Predictive Controlled Multirotor with Heavy Slung Load. **IFAC-PapersOnLine**, 20th IFAC Symposium on Automatic Control in AerospaceACA 2016. v. 49, n. 17, p. 182--187, 1 jan. 2016.

<https://doi.org/10.1016/j.ifacol.2016.09.032>

ORE, J.-P. et al. Autonomous Aerial Water Sampling. **Journal of Field Robotics**, v. 32, n. 8, p. 1095--1113, 2015. <https://doi.org/10.1002/rob.21591>.

O'REILLY, O. M. **Intermediate Dynamics for Engineers: A Unified Treatment of Newton-Euler and Lagrangian Mechanics**. Cambridge ; New York, NY: Cambridge University Press, 2008. <https://doi.org/10.1017/CBO9780511791352>.

PALUNKO, I.; FIERRO, R.; CRUZ, P. **Trajectory generation for swing-free maneuvers of a quadrotor with suspended payload: A dynamic programming approach**. 2012 IEEE International Conference on Robotics and Automation. **Anais**\... In: 2012 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION. maio 2012. <https://doi.org/10.1109/ICRA.2012.6225213>.

PIZETTA, I. H. B.; BRANDÃO, A. S.; SARCINELLI-FILHO, M. **Modelling and control of a PVTOL quadrotor carrying a suspended load**. 2015 International Conference on Unmanned Aircraft Systems (ICUAS). **Anais**\... In: 2015 INTERNATIONAL CONFERENCE ON UNMANNED AIRCRAFT SYSTEMS (ICUAS). jun. 2015.

<https://doi.org/10.1109/ICUAS.2015.7152321>.

PROUTY, R. W. **Helicopter Performance, Stability, and Control**. 2002 edition ed. Malabar, Fla.: Krieger Pub Co, 2001.

QIAN, D.; YI, J. **Hierarchical Sliding Mode Control for Under-actuated Cranes: Design, Analysis and Simulation**. Berlin Heidelberg: Springer-Verlag, 2015.

<https://doi.org/10.1007/978-3-662-48417-3>.

RAFFO, G. V.; ALMEIDA, M. M. DE. **Nonlinear robust control of a quadrotor UAV for load transportation with swing improvement**. 2016 American Control Conference (ACC). **Anais**\... In: 2016 AMERICAN CONTROL CONFERENCE (ACC). jul. 2016.

<https://doi.org/10.1109/ACC.2016.7525403>.

RAMLI, L. et al. Control strategies for crane systems: A comprehensive review. **Mechanical Systems and Signal Processing**, v. 95, p. 1--23, 1 out. 2017.

<https://doi.org/10.1016/j.ymssp.2017.03.015>.

REGO, B. S.; RAFFO, G. V. **Suspended load path tracking control based on zonotopic state estimation using a tilt-rotor UAV**. 2016 IEEE 19th International Conference on Intelligent Transportation Systems (ITSC). **Anais**\... In: 2016 IEEE 19TH INTERNATIONAL CONFERENCE ON INTELLIGENT TRANSPORTATION SYSTEMS (ITSC). nov. 2016. <https://doi.org/10.1109/ITSC.2016.7795747>.

REGO, B. S.; RAFFO, G. V. Suspended load path tracking control using a tilt-rotor UAV based on zonotopic state estimation. **Journal of the Franklin Institute**, v. 356, n. 4, p. 1695--1729, 1 mar. 2019. <https://doi.org/10.1016/j.jfranklin.2018.08.028>.

SADR, S.; MOOSAVIAN, S. A. A.; ZARAFSHAN, P. Dynamics Modeling and Control of a Quadrotor with Swing Load. **Journal of Robotics**, 2014. <https://doi.org/10.1155/2014/265897>.

SANKARANARAYANAN, V.; MAHINDRAKAR, A. D. Control of a Class of Underactuated Mechanical Systems Using Sliding Modes. **IEEE Transactions on Robotics**, v. 25, n. 2, p. 459--467, abr. 2009. <https://doi.org/10.1109/TRO.2008.2012338>.

SANTOS, M. A. et al. Suspended Load Path Tracking Control Strategy Using a Tilt-Rotor UAV. **Journal of Advanced Transportation**, 2017. <https://doi.org/10.1155/2017/9095324>.

SHTESSEL, Y. et al. **Sliding Mode Control and Observation**. 2013 edition ed. New York: Birkhäuser, 2013. <https://doi.org/10.1007/978-0-8176-4893-0>.

SINGH, T.; SINGHOSE, W. **Input shaping/time delay control of maneuvering flexible structures**. Proceedings of the 2002 American Control Conference (IEEE Cat. No.CH37301). **Anais**\... In: PROCEEDINGS OF THE 2002 AMERICAN CONTROL CONFERENCE (IEEE CAT. NO.CH37301). maio 2002. <https://doi.org/10.1109/ACC.2002.1023813>.

SLABAUGH, G. G. Computing Euler angles from a rotation matrix. **Retrieved on August**, v. 6, n. 2000, p. 39--63, 1999.

SPONG, M. W.; HUTCHINSON, S.; VIDYASAGAR, M. **Robot Modeling and Control**. 1 edition ed. Hoboken, NJ: Wiley, 2005.

SREENATH, K.; LEE, T.; KUMAR, V. **Geometric control and differential flatness of a quadrotor UAV with a cable-suspended load**. 52nd IEEE Conference on Decision and Control. **Anais**\... In: 52ND IEEE CONFERENCE ON DECISION AND CONTROL. Florence, Italy: IEEE, dez. 2013. <https://doi.org/10.1109/CDC.2013.6760219>.

SREENATH, K.; MICHAEL, N.; KUMAR, V. **Trajectory generation and control of a quadrotor with a cable-suspended load - A differentially-flat hybrid system**. 2013 IEEE International Conference on Robotics and Automation. **Anais**\... In: 2013 IEEE INTERNATIONAL CONFERENCE ON ROBOTICS AND AUTOMATION. maio 2013. <https://doi.org/10.1109/ICRA.2013.6631275>.

TANG, S.; WÜEST, V.; KUMAR, V. Aggressive Flight With Suspended Payloads Using Vision-Based Control. **IEEE Robotics and Automation Letters**, v. 3, n. 2, p. 1152--1159, abr. 2018. <https://doi.org/10.1109/LRA.2018.2793305>.

UTKIN, V.; GULDNER, J.; SHI, J. **Sliding Mode Control in Electro-Mechanical Systems**. \[s.l.\] CRC Press, 2009. <https://doi.org/10.1201/9781420065619>.

VALAVANIS, K. P.; VACHTSEVANOS, G. J. UAV Applications: Introduction. In: VALAVANIS, K. P.; VACHTSEVANOS, G. J. (Eds.). **Handbook of Unmanned Aerial Vehicles**. Dordrecht: Springer Netherlands, 2015. p. 2639--2641. <https://doi.org/10.1007/978-90-481-9707-1_151>.

VARGAS MORENO, A. E. **Machine learning techniques to estimate the dynamics of a slung load multirotor UAV system**. PhD---\[s.l.\] University of Glasgow, 2017. Disponível em: <http://theses.gla.ac.uk/8513/>. Acesso em: 15 set. 2019.

WANG, W. et al. Design of a stable sliding-mode controller for a class of second-order underactuated systems. **IEE Proceedings - Control Theory and Applications**, v. 151, n. 6, p. 683--690, nov. 2004. <https://doi.org/10.1049/ip-cta:20040902>.

WANG, W.; LIU, X. D; YI, J. Q. Structure design of two types of sliding-mode controllers for a class of under-actuated mechanical systems. **IET Control Theory Applications**, v. 1, n. 1, p. 163--172, jan. 2007. <https://doi.org/10.1049/iet-cta:20050435>.

XIONG, J.-J.; ZHENG, E.-H. Position and attitude tracking control for a quadrotor UAV. **ISA Transactions**, v. 53, n. 3, p. 725--731, 1 maio 2014. <https://doi.org/10.1016/j.isatra.2014.01.004>.

XU, R.; ÖZGÜNER, Ü. Sliding mode control of a class of underactuated systems. **Automatica**, v. 44, n. 1, p. 233--241, 1 jan. 2008. <https://doi.org/10.1016/j.automatica.2007.05.014>.

ZHENG, E.-H.; XIONG, J.-J.; LUO, J.-L. Second order sliding mode control for a quadrotor UAV. **ISA Transactions**, Disturbance Estimation and Mitigation. v. 53, n. 4, p. 1350--1356, 1 jul. 2014. <https://doi.org/10.1016/j.isatra.2014.03.010>.

ZHOU, X. et al. **Stabilization of a Quadrotor With Uncertain Suspended Load Using Sliding Mode Control**. ASME Proceedings \| 40th Mechanisms and Robotics Conference. **Anais**\... In: 40TH MECHANISMS AND ROBOTICS CONFERENCE. Charlotte, North Carolina, USA: ASME, 21 ago. 2016Disponível em: \<http://dx.doi.org/10.1115/DETC2016-60060\>. Acesso em: 10 mar. 2018. <https://doi.org/10.1115/DETC2016-60060>

ZÜRN, M. et al. **MPC controlled multirotor with suspended slung Load: System architecture and visual load detection**. 2016 IEEE Aerospace Conference. **Anais**\... In: 2016 IEEE AEROSPACE CONFERENCE. mar. 2016. <https://doi.org/10.1109/AERO.2016.7500543>.

# APÊNDICE I -- TRANSFORMAÇÕES CINEMÁTICAS

Este apêndice apresenta o procedimento de obtenção das transformações cinemáticas entre o sistema de coordenadas inercial $\Sigma_{i}$ e o não inercial $\Sigma_{b}$ representadas no texto pelas equações (2.1) e (2.2). Ressalta-se que o conteúdo desta seção é baseado em (BRESCIANI, 2008).

## Ângulos de Euler {#ângulos-de-euler .unnumbered}

Como informado na seção 2.1, para representar a orientação do sistema de coordenadas não inercial $\Sigma_{b}$ em relação ao referencial fixo $\Sigma_{i}$, utiliza-se a notação de Euler em que, partindo-se do referencial inercial, aplica-se três rotações consecutivas como ilustra a Figura I-1:

![](media/image53.emf){width="5.78013779527559in" height="1.78125in"}

Figura I-1 -- Sequência de rotações entre o eixo de coordenadas inercial e o não inercial.

Como ilustrado na Figura I-1, primeiramente rotaciona-se em torno do eixo z (${\overrightarrow{e}}_{z}$) de um ângulo $\psi$, resultando no sistema de coordenadas intermediário $\Sigma_{c} = \left\lbrack {\overrightarrow{e}}_{x}^{c},{\overrightarrow{e}}_{y}^{c},{\overrightarrow{e}}_{z}^{c} \right\rbrack$, sendo que ${\overrightarrow{e}}_{z}^{c} = {\overrightarrow{e}}_{z}$. Depois, rotaciona-se em torno de ${\overrightarrow{e}}_{y}^{c}$ de um ângulo $\theta$, resultando no referencial intermediário $\Sigma_{d} = \left\lbrack {\overrightarrow{e}}_{x}^{d},{\overrightarrow{e}}_{y}^{d},{\overrightarrow{e}}_{z}^{d} \right\rbrack$, ${\overrightarrow{e}}_{y}^{d} = {\overrightarrow{e}}_{y}^{c}$. Enfim, rotaciona-se em torno de sendo ${\overrightarrow{e}}_{x}^{c}$ de um ângulo $\phi$, resultando no sistema de coordenadas não inercial $\Sigma_{b} = \left\lbrack {\overrightarrow{e}}_{x}^{b},{\overrightarrow{e}}_{y}^{b},{\overrightarrow{e}}_{z}^{b} \right\rbrack$.

Assim, um vetor escrito em determinado sistema de coordenadas que esteja rotacionado em torno de um dos eixos ortogonais de outro referencial pode ser transferido para este multiplicando- o pela matriz de rotação associada àquele eixo. As equações (I-1), (I-2) e (I-3) apresentam as matrizes de rotação para os eixo $x$, $y$ e $z$ respectivamente.

  ----------------------------------------------------------------- -------
  $$\mathbf{R}_{\mathbf{x}}(\phi) = \begin{bmatrix}                 (I-1)
  1 & 0 & 0 \\
  0 & \cos\phi & - \sin\phi \\
  0 & \sin\phi & \cos\phi \\
  \end{bmatrix}$$

  $$\mathbf{R}_{\mathbf{y}}(\theta) = \begin{bmatrix}               (I-2)
  \cos\theta & 0 & \sin\theta \\
  0 & 1 & 0 \\
   - \sin\theta & 0 & \cos\theta \\
  \end{bmatrix}$$

  $$\mathbf{R}_{\mathbf{z}}(\psi) = \begin{bmatrix}                 (I-3)
  \cos\psi & - \sin\psi & 0 \\
  \sin\psi & \cos\psi & 0 \\
  0 & 0 & 1 \\
  \end{bmatrix}$$
  ----------------------------------------------------------------- -------

Assim, a matriz de rotação do referencial do corpo para o referencial inercial $\mathbf{R}$ apresentado no texto pela Eq. (2.1) é obtida multiplicando-se as matrizes de rotação simples na sequência definida:

  ------------------------------------------------------------------------------------------------------------ -------
  $$\mathbf{R} = \mathbf{R}_{\mathbf{z}}(\psi)\mathbf{R}_{\mathbf{y}}(\theta)\mathbf{R}_{\mathbf{x}}(\phi)$$   (I-1)

  ------------------------------------------------------------------------------------------------------------ -------

Ressalta-se que, para fazer a transformação inversa: do referencial inercial para o não inercial, basta fazer o mesmo cálculo com a inversa da matriz $\mathbf{R}$, que corresponde à sua transposta.

## Transformação da Velocidade Angular {#transformação-da-velocidade-angular .unnumbered}

A lei de controle explorada demanda a determinação da taxa de variação dos ângulos de Euler $\overrightarrow{\Omega} = \left\lbrack \dot{\phi},\dot{\theta},\dot{\psi} \right\rbrack$, mas as velocidades angulares são lidas e se manifestam nas equações dinâmicas no referencial do corpo, requerendo, portanto, uma transformação entre estas coordenadas. A matriz de rotação não se aplica a este caso, pois as taxas de variação dos ângulos de Euler não estão definidas no referencial inercial. Observa-se na figura Figura I-1 que $\dot{\psi}$ ocorre no eixo ${\overrightarrow{e}}_{z}$, $\dot{\theta}$ ocorre no eixo ${\overrightarrow{e}}_{y}^{c}$ e $\dot{\phi}$ acontece no eixo ${\overrightarrow{e}}_{x}^{b}$. Assim, a matriz de transformação demandada é obtida como mostra a Eq. (I-1):

  --------------------------------------------------------------------------------- -------
  $$\left\{ \begin{array}{r}                                                        (I-1)
  p \\
  q \\
  r \\
  \end{array} \right\} = \left\{ \begin{array}{r}
  \dot{\phi} \\
  0 \\
  0 \\
  \end{array} \right\} + R_{x}^{T}(\phi)\left\{ \begin{array}{r}
  0 \\
  \dot{\theta} \\
  0 \\
  \end{array} \right\} + R_{x}^{T}(\phi)R_{y}^{T}(\theta)\left\{ \begin{array}{r}
  0 \\
  0 \\
  \dot{\psi} \\
  \end{array} \right\}$$

  --------------------------------------------------------------------------------- -------

Obtendo-se:

  ----------------------------------------------------------------- -------
  $$\left\{ \begin{array}{r}                                        (I-2)
  p \\
  q \\
  r \\
  \end{array} \right\} = \begin{bmatrix}
  1 & 0 & - \sin\theta \\
  0 & \cos\phi & \sin\phi\cos\theta \\
  0 & - \sin\phi & \cos\phi\cos\theta \\
  \end{bmatrix}\left\{ \begin{array}{r}
  \dot{\phi} \\
  \dot{\theta} \\
  \dot{\psi} \\
  \end{array} \right\}$$

  ----------------------------------------------------------------- -------

Observa-se que a Eq. (I-2) corresponde à relação inversa da Eq. (2.2).

# APÊNDICE II -- INTERPOLAÇÃO POLINOMIAL POR PARTES

Esta seção apresenta a técnica de interpolação polinomial por partes, comumente chamada de *spline*, utilizada na solução de planejamento de trajetórias deste trabalho. Primeiramente, ilustra-se o conceito para polinômios de 3ª ordem com velocidade contínua e depois se estende a análise para polinômios de ordem maior. Ressalta-se que o conteúdo desta seção é baseado em (SPONG; HUTCHINSON; VIDYASAGAR, 2005).

O procedimento parte da definição da posição entre dois pontos $q\left( t_{0} \right)$ e $q\left( t_{f} \right)$ como um polinômio de terceiro grau em função do tempo:

  ----------------------------------------------------------------- --------
  $$q(t) = a_{0} + a_{1}t + a_{2}t^{2} + a_{3}t^{3}$$               (II-1)

  ----------------------------------------------------------------- --------

Derivando-se a Eq. (II-1), obtém-se a expressão para a velocidade:

  ----------------------------------------------------------------- --------
  $$v(t) = a_{1} + 2a_{2}t + 3a_{3}t^{2}$$                          (II-2)

  ----------------------------------------------------------------- --------

Observa-se que existem quatro coeficientes desconhecidos: $a_{0}$, $a_{1}$, $a_{2}$ e $a_{3}$; necessitando de quatro restrições para defini-las. Além da especificação das posições final e inicial, determina-se valores desejados para as velocidades nestes pontos, de modo que:

  ------------------------------------------------------------------ --------
  $$q_{0} = a_{0} + a_{1}t_{0} + a_{2}t_{0}^{2} + a_{3}t_{0}^{3}$$   (II-2)

  $$q_{f} = a_{0} + a_{1}t_{f} + a_{2}t_{f}^{2} + a_{3}t_{f}^{3}$$   (II-3)

  $$v_{0} = a_{1} + 2a_{2}t_{0} + 3a_{3}t_{0}^{2}$$                  (II-4)

  $$v_{f} = a_{1} + 2a_{2}t_{f} + 3a_{3}t_{f}^{2}$$                  (II-5)
  ------------------------------------------------------------------ --------

As Equações II-2 a II-5 formam um sistema linear (Eq. II-6) do qual é possível obter os valores dos coeficientes:

  ----------------------------------------------------------------- --------
  $$\begin{bmatrix}                                                 (II-6)
  1 & t_{0} & t_{0}^{2} & t_{0}^{3} \\
  0 & 1 & 2t_{0} & 3t_{0}^{2} \\
  1 & t_{f} & t_{f}^{2} & t_{f}^{3} \\
  0 & 1 & 2t_{f} & 3t_{f}^{2} \\
  \end{bmatrix}\left\{ \begin{array}{r}
  a_{0} \\
  a_{1} \\
  a_{2} \\
  a_{3} \\
  \end{array} \right\} = \left\{ \begin{array}{r}
  q_{0} \\
  v_{0} \\
  q_{f} \\
  v_{f} \\
  \end{array} \right\}$$

  ----------------------------------------------------------------- --------

A Figura II-1 apresenta uma curva gerada por este método para os tempos $\left\lbrack t_{0},t_{f} \right\rbrack = \lbrack 0,4\rbrack$ e restrições $\left\lbrack q_{0},q_{f},v_{0},v_{f} \right\rbrack = \lbrack 0,4,0,0\rbrack$.

![](media/image54.emf){width="3.6979166666666665in" height="2.7717825896762904in"}

Figura II-1 -- Exemplo de trajetória polinomial cúbica com restrições de posição e velocidade nos pontos extremos.

Para definir uma trajetória com múltiplos pontos de passagem, basta gerar polinômios em partes de modo que a posição e velocidade inicial de uma parte coincida com a posição e velocidade final da curva anterior.

Para estender o conceito para polinômios de ordem maior, basta identificar o padrão da solução. Verifica-se que ao especificar a posição e a velocidade dos dois pontos, completa-se 4 restrições que solucionam a obtenção dos 4 coeficientes do polinômio que determina a posição ao longo do tempo. Assim, para cada diferenciação acrescida, aumenta-se o número de restrições e a ordem do polinômio da posição em dois. Sendo $p$ a ordem do polinômio da posição e $d$ a ordem da maior derivada especificada nos pontos de interesse, tem-se que $p = 2d + 1$. Portanto trajetórias com especificação até a sexta derivada são obtidas polinômios de 13ª ordem.

[^1]: Disponível em: <http://www.capitalpress.com/Oregon/20151118/helicopters-cleared-for-christmas-tree-harvest>. Acesso em: 17 ago. 2018

[^2]: Disponível em: <https://br.pinterest.com/pin/863213453549609958/>. Acesso em: 17 ago. 2018.

[^3]: Disponível em: <https://www.tdworld.com/transmission/bc-hydro-delivers-power-and-progress>. Acesso em: 17 ago. 2018.

[^4]: Alternativamente, pode-se substituir a Eq. (2.24) na Eq. (2.23) e resultar em uma equação em função da posição da carga ${\overrightarrow{r}}_{L}$. O resultado é semelhante à Eq. (2.24).

[^5]: No que diz respeito à geração de trajetória, Sreenath, Michael e Kumar (2013) também considera momentos em que a tensão no cabo é nula, desenvolvendo um modelo dinâmico híbrido. O presente trabalho faz referência apenas à análise do modelo para quando a tensão no cabo não é nula.
