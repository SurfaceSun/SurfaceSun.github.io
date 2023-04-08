---
layout: post
title: "Daily Report"
date:   2020-03-23
tags: [Test]
comments: true
author: Qinbo Sun
toc: true
---

A hybrid vehicle has an internal combustion engine, a motor/generator connected to a storage battery, and a conventional (friction) brake. 

We consider a (highly simpliﬁed) model of a parallel hybrid vehicle, in which both the motor/generator and the engine are directly connected to the drive wheels. The engine can provide power to the wheels, and the brake can take power from the wheels, turning it into heat. 

The motor/generator can act as a motor, when it uses energy stored in the battery to deliver power to the wheels, or as a generator, when it takes power from the wheels or engine, and uses the power to charge the battery. When the generator takes power from the wheels and charges the battery, it is called regenerative braking; unlike ordinary friction braking, the energy taken from the wheels is stored, and can be used later. 

The vehicle is judged by driving it over a known, ﬁxed test track to evaluate its fuel eﬃciency. 

A diagram illustrating the power ﬂow in the hybrid vehicle is shown below.

# Weekly Report

Qinbo Sun 09/03/2020-16/03/2020

## Problem statement

Re-Formulate energy planning problem as a general case in general recycling robots.

Then, we can find the state of art method to solve this problem.

### Goal

Proposed a general scheme to compare with state of art on energy planning problem.

## Review

There are some works in vehicles, especially the hybrid vehicles. I choose a typical problem stated below. Then, we can abstract an energy planning scheme for general robotics system.

#### Exmaple: *Optimal operation of a hybrid vehicle*. 

A hybrid vehicle has an internal combustion engine, a motor/generator connected to a storage battery, and a conventional (friction) brake. 

We consider a (highly simpliﬁed) model of a parallel hybrid vehicle, in which both the motor/generator and the engine are directly connected to the drive wheels. The engine can provide power to the wheels, and the brake can take power from the wheels, turning it into heat. 

The motor/generator can act as a motor, when it uses energy stored in the battery to deliver power to the wheels, or as a generator, when it takes power from the wheels or engine, and uses the power to charge the battery. When the generator takes power from the wheels and charges the battery, it is called regenerative braking; unlike ordinary friction braking, the energy taken from the wheels is stored, and can be used later. 

The vehicle is judged by driving it over a known, ﬁxed test track to evaluate its fuel eﬃciency. 

A diagram illustrating the power ﬂow in the hybrid vehicle is shown below.

<img src=".\img\image-20200316223251789.png" alt="image-20200316223251789" style="zoom:30%;" />

The arrows indicate the direction in which the power ﬂow is considered positive. The engine power peng, for example, is positive when it is delivering power; the brake power pbr is positive when it is taking power from the wheels. The power preq is the required power at the wheels. It is positive when the wheels require power (e.g., when the vehicle accelerates, climbs a hill, or cruises on level terrain). The required wheel power is negative when the vehicle must decelerate rapidly, or descend a hill. 

#### Transfer the scheme to general energy-recycling robot system

The new diagram is shown below,

<img src=".\img\image-20200316231112112.png" alt="image-20200316231112112" style="zoom:20%;" />

The arrows indicate the direction in which the power ﬂow is considered positive. The harvest energy power Pheng. It is combined with the battery, thus, the battery can be charged and the battery can powering the right-sides components. The actuators power is Pact, which represents the total actuators in the robots. The total power in sensors is Psen, The Dynamic disturbance or signals is Pdyn. It can be captured by stochastic process model. The output is the required power act on the robot system.

--------

### Model these to scheme as mathematical problem [Next]

In the next week, I'm going to formulate the former two problems and convert the problem as convex optimization problem. Then I can utilize other literature which are using the optimization method in energy planning. 

Based on these general scheme, I can make a comparison with the other proposed method numerically and analytically.

------------



All of these powers are functions of time, which we discretize in one second intervals, with $t=1,2, \ldots, T .$ The required robot system power $p_{\mathrm{req}}(1), \ldots, p_{\mathrm{req}}(T)$ is ***given*** ***or unknown***. (The speed of the vehicle on the track is specified, so together with known road slope information, and known aerodynamic and other losses, the power required at the wheels can be calculated.) Power is conserved, which means we have
$$
p_{\text {req }}(t)=p_{\text {eng }}(t)+p_{\text {act }}(t)-p_{\text {sen }}(t)-p_{\text {dyn }}(t), \quad t=1, \ldots, T
$$
The sensors can only consume power, so we have $p_{\mathrm{sen}}(t) \geq 0$ for each $t .$ The engine can only provide power, and only up to a given limit $P_{\mathrm{eng}}^{\max },$ i.e., we have
$$
0 \leq p_{\mathrm{eng}}(t) \leq P_{\mathrm{eng}}^{\max }, \quad t=1, \ldots, T
$$
The motor/generator power is also limited: $p_{\mathrm{act}}$ must satisfy
$$
P_{\mathrm{act}}^{\min } \leq p_{\mathrm{act}}(t) \leq P_{\mathrm{act}}^{\max }, \quad t=1, \ldots, T
$$
Here $P_{\mathrm{act}}^{\text {max }}>0$ is the maximum motor power, and $-P_{\mathrm{act}}^{\text {min }}>0$ is the maximum generator power. The battery charge or energy at time $t$ is denoted $E(t), t=1, \ldots, T+1 .$ The battery energy satisfies
$$
E(t+1)=E(t)-p_{\mathrm{act}}(t)-\eta\left|p_{\mathrm{act}}(t)\right|, \quad t=1, \ldots, T+1
$$
where $\eta>0$ is a known parameter. (The term $-p_{\mathrm{act}}(t)$ represents the energy removed or added the battery by the motor/generator, ignoring any losses. The term $-\eta\left|p_{\mathrm{act}}(t)\right|$ represents energy lost through inefficiencies in the battery or motor/generator.) The battery charge must be between 0 (empty) and its limit $E_{\text {batt }}^{\max }$ (full), at all times. (If $E(t)=0,$ the battery is fully discharged, and no more energy can be extracted from it; when $E(t)=E_{\text {batt }}^{\text {max }},$ the battery is full and cannot be charged.) To make the comparison with non-hybrid vehicles fair, we fix the initial battery charge to equal the final battery charge, so the net energy change is zero over the track: $E(1)=E(T+1) .$ We do not specify the value of the initial (and final) energy. The objective in the problem is the total fuel consumed by the engine, which is
$$
F_{\text {total }}=\sum_{t=1}^{T} F\left(p_{\text {eng }}(t)\right)
$$
where $F: \mathbf{R} \rightarrow \mathbf{R}$ is the fuel use characteristic of the engine. We assume that $F$ is positive, increasing, and convex. Formulate this problem as a convex optimization problem, with variables $p_{\mathrm{eng}}(t), p_{\mathrm{mg}}(t)$ and $p_{\mathrm{br}}(t)$ for $t=1, \ldots, T,$ and $E(t)$ for $t=1, \ldots, T+1 .$ Explain why your formulation is equivalent to the problem described above.

----

We first collect the given objective and constraints to form the problem
$$
\begin{array}{cl}
\underset{t=1}{\operatorname{minimize}} & \sum_{t=1}^{T} F\left(p_{\mathrm{eng}}(t)\right) \\
\text { subject to } & p_{\mathrm{req}}(t)=p_{\mathrm{eng}}(t)+p_{\mathrm{act}}(t)-p_{\mathrm{sen}}(t)-p_{\mathrm{dyn}}(t) \\
& \left.\left.E(t+1)=E(t)-p_{\mathrm{act}}(t)\right)-\eta | p_{\mathrm{act}}(t)\right) | \\
& 0 \leq E(t) \leq E_{\mathrm{batt}}^{\max } \\
& E(1)=E(T+1) \\
& 0 \leq p_{\mathrm{eng}}(t) \leq P_{\mathrm{eng}}^{\max } \\
& P_{\mathrm{act}}^{\min } \leq p_{\mathrm{act}}(t) \leq P_{\mathrm{act}}^{\max } \\
& 0 \leq p_{\mathrm{sen}}(t)\\
& 0 \leq p_{\mathrm{dyn}}(t)
\end{array}
$$
where each constraint is imposed for the appropriate range of $t .$ The fuel use function $F$ is convex, so the objective function is convex. With the exception of the battery charge equations, each constraint is a linear equality or linear inequality. So in this form the problem is not convex. We need to show how to deal with the nonconvex constraints
$$
\left.E(t+1)=E(t)-p_{\mathrm{act}}(t)-\eta | p_{\mathrm{act}}(t)\right) |
$$
One approach is to replace this constraint with the relaxation,
$$
\left.E(t+1) \leq E(t)-p_{\mathrm{act}}(t)-\eta | p_{\mathrm{act}}(t)\right) |
$$
which is convex, in fact, two linear inequalities. Intuitively, this relaxation means that we open the possibility of throwing energy from the battery away at each step. This sounds like a bad idea, when fuel efficiency is the goal, and indeed, it is easy to see that if we solve the problem with the relaxed battery charge constraints, the optimal $E^{\star}$ satisfies
$$
\left.E^{\star}(t+1)=E^{\star}(t)-p_{\mathrm{act}}(t)-\eta | p_{\mathrm{act}}(t)\right) |
$$
and therefore solves the original problem. To argue formally that this is the case, suppose that the solution of the relaxed problem does throw away some energy at some step $t$ We then construct a new trajectory, where we do not throw away the extra energy, and instead, use the energy to power the wheels, and reduce the engine power. This reduces the fuel consumption since the fuel consumption characteristic is increasing, which shows that the original could not have been optimal.





Ref. [1] Convex Optimization, Stephen boyd, Cambridge University Compress.

-----------

## Related works

### **Renewable Energy for Robots and Robots for Renewable Energy – A Review 2019**

the integration between robots and renewable energy sources is discussed. In other words, two main points are investigated: (1) how can renewable energy be a viable **source** of energy for robots and (2) how can the renewable energy industry benefit from **utilizing robots in the execution** of renewable energy-related tasks.

Solar Energy

Wind Energy

Biomass Energy

Ocean Energy

Hydroelectric Energy

Geothermal Energy

Hydrogen Energy

##### Robot Classification

- Geometry: serial and parallel manipulators. 
- Workspace: reachable and dexterous workspace. 
- Actuation: electrical, hydraulic, pneumatic, etc. 
- Control: servo (closed loop control) and non-servo (open loop control) robots. Application: assembly and non-assembly robots.

###### Industrial Manipulators

###### Mobile: UAV, USV, ASV, AUV

###### Terrestrial: Wheeled robots, Tracked Robots, Legged Robots, Hybrid Robots. 

###### ![Screen Shot 2020-03-20 at 7.08.34 AM](img/Screen Shot 2020-03-20 at 7.08.34 AM.png)



### A Review on Energy-Saving Optimization Methods for Robotic and Automatic Systems 2017**

Indeed, this is the case for robotic and automatic systems, for which, in the past, the minimization of energy demand was not considered a design objective. The proper design and operation of industrial robots and automation systems represent a great opportunity for reducing energy consumption in the industry.

This review paper classifies and analyses several methodologies and technologies that have been developed with the aim of providing a reference of existing methods, techniques and technologies for enhancing the energy performance of industrial robotic and mechatronic systems. **Hardware** and **software** methods, including several subcategories, are considered and compared, and emerging ideas and possible future perspectives are discussed.

The third is the mixed type.

##### **Hardware**

1. *Robot type:* The selection of more energy-efficient mechatronic and robotic systems available for a given application [4,5,11,19]
2. *Hardware replacement*: The re-design or substitution of components with more efficient components (i.e., more-efficient or lighter components, or both) [1,2,6,7,10,12,16,18]
3. *Hardware addition*: The addition of components for storing and recovering energy $[20-36]$

##### **software**

1. ***Trajectory optimization***: A modification of the path or the motion profile, or both, are performed. (Point-to-point, PTP, Multi-point (MP))

   - minimum effort (i.e., sum of squared torque) [40,59]
   - minimum torque-rate (i.e., sum of squared-derivative torgue) $[59]_{i}$
   - minimum electrical energy $[42-47,51-53,55,57,58],$ expressed as $\int_{0}^{T} e(t) i(t) d t$
   - minimum mechanical energy $[38,39,41],$ expressed as $\int_{0}^{T} \not \partial(t) \tau(t) d t$ $\cdot$
   - minimum of grid energy (i.e., considers the energy exchanged between axes due to recovery process) [49] :
   - minimum of losses in a robot DC motor drive $[56] .$
     In addition to the energy expenditure, other quantities can be minimized, such as the residual vibration amplitude [38,39]
   - A wide range of optimization algorithms are used to find the optimum trajectory. These include the following:
   - gradient-based optimization algorithms [48], and more specifically, sequential quadratic programming (SQP) [56]:
   - genetic algorithms (GA) [41,57]
   - real-coded genetic algorithms (RGA) $[43-47,51-53]$
   - radial-basis function networks and genetic algorithms (RBF-GA) [38]
   - Pontryagin's minimum algorithm (PMP) [55,58]
   - metaheuristic algorithms (MA) [39]
   - discrete dynamic programming (DDP) [59]
   - iterative dynamic programming (DP) [42]![Screen Shot 2020-03-20 at 5.46.29 AM](img/Screen Shot 2020-03-20 at 5.46.29 AM.png)
   - 

2. ***Operation scheduling***: Re-scheduling of subsequent movements and operations is considered.

   ***Time Scaling***

   ***Sequence Scheduling***: The goal of the sequence planning approach [66,67] is to reduce the energy consumption of individual and interacting robots in a working station without changing the original paths or the total cycle time. The optimization problem is formulated as the minimization of the weighted squared angular accelerations for all joints (approximate measure of the energy consumption). 

   this approach is applied to optimize the motion of an anthropomorphic robotic cell (electro-mechanical model is needed) from the end of a process to the home position, in order not to affect the overall industrial process.

#####  Mixed Approaches

1. Natural Motion
   - Natural dynamics modification (NDM), in which the mechatronic system body or parts of it are designed to perform a given periodic task efficiently; that is, the system's natural frequency is adapted to the task. 
   - Natural dynamics exploitation (NDE), in which the mechatronic system motion is altered in order to exploit the system's natural frequency; that is, the task is adapted to the system's characteristics.
2. Optimal Sharing



### Methods and tools for community energy planning: A review 2014*

Community Energy planning (CEP) method, Community Master Plan (CMP), Community Regulatory Plan (CRP), Community Site Plan (CSP), Architectural Design (AD)

The **functions** (particularly focused on the existence of the functions of energy demand prediction, renewable energy resource assessment and whole community energy system optimization) and **characteristics** of these computer tools for community energy system design (CESD) were listed and compared.

One of the current **challenges** is predicting community energy demand. There is still a shortage of available tools for determining energy-related indictors at the CRP stage. 

A built Community Energy Consumption data Monitoring and Statistic System (CECMSS) may help to predict a community's secondary energy demand.

![Screen Shot 2020-03-20 at 4.02.19 AM](img/Screen Shot 2020-03-20 at 4.02.19 AM.png)

**EnergyPLAN**: The main purpose of this model is to assist in the design of national energy planning strategies on the basis of the technical and economic analysis of the consequences of different national energy systems and investments.

**E_GIS:** t formulates an E-GIS Database for the environmental and energy planning of a city

**SUNtool:** It can optimize the layout of neighborhoods buildings (composed by 50–500 buildings), microclimate (temperature, velocity and air pressure) design and sustainable energy plans.