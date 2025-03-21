+++
title = "Lab 7"
date = "2025-03-17"

[taxonomies]

[extra]
comment = true
+++ 
# Previous: [Lab 6](/fast-robots/lab6)

## Lab Tasks

### System Identification 

If we approximate the open-loop robot system as first-order with respect to velocity, using Newton's second law to write 

$ F = ma = m\ddot{x} $

with dynamics 

$ \ddot{x} = \frac{d}{m} \dot{x} + \frac{u}{m}$

where $d$ and $m$ are lumped-parameter terms to roughly describe the drag (incorporating resistance from the air, ground, and motor dynamics) and intertia (incorporating mostly car mass, but also motor dynamics).

Choosing a state 

$ x = \vec{x,\dot{x} } $

we can then describe the open-loop system in state-space form as 





With a PWM input of 80 (around the center of my applied range in [Lab 5](/fast-robots/lab5)), I measured the following open-loop data and applied a low-pass-filter with $\alpha$ = 1 Hz:

<img src="/files/lab7/sysID_v2_pwm80.png" alt="sysID_v2_pwm80"  width = 800 >

I then applied an exponential fit to the velocity data and marked the steady-state and 90% rise time points.

<img src="/files/lab7/sysID_v2_curvefit.png" alt="curve fit"  width = 800 >




with a steady-state velocity of -2091.5 mm/s $\approx$ -2.09 m/s, and 90% rise time of 2.55 s, and setting u = 1 N (unit input), we have 

$d = \frac{1 N}{2.09 m/s} = .478 kg/s$

$m = \frac{.478 kg/s * 2.55 s}{ln(.1)} = .529 kg$

leading to 

A = [0 1 ; 0 -d/m ] = [0 1 ; 0 -.903]

B = [0 ; 1/m] = [0 ; 1.74]

We're only directly measuring the TOF data $x$, so it makes sense for our state-space system to output exclusively the position of the system (relative to a wall). In other words, 

C = [-1 ; 0] 

D = [0 ; 0]


I get new motor inputs (not necessarily new TOF values ) every 20 ms, so I used that as my Kalman filter timestep. 

uncertanties?? for now use dt = .02 dx = .05 per stephan


I then used the provided `kf()` (link site) to filter my data. 

With my process and sensor noise of $\sigma_1 = \sigma_2$ = 70.7 and $\sigma_3$ = 44.7, my filtered data looked like this:

<img src="/files/lab7/kf_low.png" alt="KF 1"  width = 800 >

Decreasing process noise (i.e. if I become more certain about the model I think my robot operates on, and less certain of my sensor data relative to the model), I get the following: 

<img src="/files/lab7/kf_lowprocess_highsensor.png" alt="KF 2"  width = 800 >

After tweaking the covariances some more, I was able to get the following filtered data, which tracks TOF readings pretty well:

<img src="/files/lab7/kf_okiedokie.png" alt="KF 3"  width = 800 >










## Collaborations

I worked with [Lucca Correia](https://correial.github.io/) and [Trevor Dales](https://trevordales.github.io/) extensively. I used ChatGPT to help with curve fitting and generating nice plots. 


# Next: [Lab 7](/fast-robots/lab7)



