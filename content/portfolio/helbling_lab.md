+++

title = "Quadrupedal Microrobot" 
description = "Custom flex PCB + embedded control stack for untethered sensing and closed-loop heading/range control on a 1.5 g quadruped microrobot through the Helbling Robotics Laboratory at Cornell." 
weight = 1 
date = 2025-11-17

[extra] 
local_image = "/portfolio/comt_old_untethered.png" 
github = "https://github.com/Helbling-Lab" 

 +++


## Overview
I developed onboard electronics and embedded control software for the Cornell MicroTerrestrial (COMT) robot, a 1.5 g electromagnetically actuated quadruped, with the goal of enabling untethered sensing and closed-loop motion control.

<!-- FIGURE (hero): COMT robot photo or “robot on ruler/scale” shot -->
![COMT microrobot (untethered prototype)](/COMT/comt_old_untethered.png) 

## Goals and constraints
- **System goals:** feedback control, autonomous operation, and repeatable control authority in a compact platform.
- **SWaP constraints:** fit within the chassis envelope while keeping mass and power overhead low relative to the actuators.
- **Performance targets:** <300 mA system current (normal gait), <500 mg PCB mass, peak control loop ≥2× drive frequency, PCB width <19 mm.

## What I built

### Flex PCB for onboard sensing + actuation
I designed a bent flex PCB integrating the microcontroller, inertial sensing, time-of-flight (ToF) ranging, and four motor drivers to provide independent control over all eight actuator coils.

<!-- FIGURE: PCB render (top/bottom) + assembled board photo -->
![Custom bent flex PCB for COMT](/COMT/fig_pcbfold.png)

- **Bent packaging:** PCB bend forms the ToF mount, reducing footprint and mass. ToF inclination is adjustable by shifting the mounting position of the top half of the PCB.

- **Mechanical design:** 25 µm Kapton stackup (approx. 120 µm total thickness), staggered trace routing was used in flexed regions to reduce stress (especially near the ToF-mounted connector), and cut-outs were added to reduce the force needed to secure the board in its bent configuration.


### Real-time embedded control architecture
I implemented an interrupt-driven firmware structure that separates actuation timing from lower-rate sensing and control.

Phase control is well-suited to legged microrobots because locomotion is dominated by the relative timing of stance and swing events. Adjusting inter-leg phase provides a direct, low-overhead way to to control trajectory. We introduce PWM current control to improve open-loop tuning capability and control authority.


- **High-rate actuation loop:** periodic interrupt advances through a precomputed gait calendar with 256 phase slices per gait cycle for timing.
- **Supervisory control loop:** two fixed-point 2-DOF PID controllers (distance and yaw) combine ToF range and IMU heading into per-side duty commands for differential drive trajectory control.
- **Sensor handling:** interrupt-driven asynchronous I2C state machines for ToF updates and IMU FIFO streaming.

## Modeling and performance characterization

### Transmission characterization
I quantified lift/swing coupling in the series transmission using a measured geometry matrix and used this characterization to interpret leg-tip motion limits and usable operating regions.


### Resonance-aware speed modeling
I fit straight-line speed versus frequency data and found that a two-mode series transmission model consistently outperformed a single-mode model, supporting the presence of two dominant resonance modes in the measured response. Dominant peaks were observed near 53 Hz and 85 Hz, with an additional feature near 30 Hz consistent with a predicted body mode.

<!-- FIGURE: speed vs frequency plot with model fits and annotated peaks -->
![Straight-line speed vs frequency with model fits](/COMT/fig_speed_freq.png)

## Evaluation status
- **Demonstrated:** per-leg variable displacement control and a complete sensing-to-control pipeline capable of feeding actuation targets in real time.
- **Pending in this work:** full verification of untethered closed-loop control authority under extended trials (limited by manufacturing and timeline constraints).

## My role
I owned the control system end-to-end: electronics architecture, PCB design, and embedded software implementation. Mechanical chassis, legs, and actuator development were led by Julie Villamil, an ECE Ph.D. candidate.

## Applications
This platform targets low-footprint autonomous monitoring in sensitive or hazardous environments, where small robots can reduce disturbance and improve safety (for example, confined inspection spaces).

---