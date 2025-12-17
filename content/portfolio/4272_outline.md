+++

title = "Wind Turbine Blade Design" 
description = "The final project for MAE 4272: Fluids and Heat Transfer Laboratory" 
weight = 2
date = 2025-11-18

[extra] 
local_image = "/portfolio/4272.jpg" 

[extra.latex_viewer]
path = "static/4272/allDragNoSwag.pdf"
title = "Report"
description = "Read the whole report here:"
pdf_path = "4272/allDragNoSwag.pdf"

 +++
<!-- 
As part of MAE 4272, you will create a simple professional engineering portfolio and publish a short webpage describing your Blade Design Project. This page is something you can share externally for example with recruiters.

-->

 <!-- Create a page in your portfolio that summarizes your team’s blade design work. This should be written at a level appropriate for a hiring manager or technical recruiter.

Your page must include:

Project overview: What you were asked to design and why

Design process: Key decisions, iterations, or modeling steps your team performed

Testing summary: How the blades were tested and what data you analyzed

Your contribution: Briefly describe what you worked on

Figures: Plots, CAD image, and photos  --> 


# Overview

MAE 4272 is a lab-based class intended to enage students to actively design, build,
and analyze fluid-thermal systems, linking the fluid-mechanical and thermodynamic/heat-transfer components of the Cornell Mechanical and Aerospace Engineering core curriculum. 

For this class' final project, students (in teams of 3 to 4) were tasked with designing the blades of a small-scale wind turbine according to a small set of mechanical and environmental constraints. We did this to demonstrate the application of the fluid-thermal analytical techniques learned in class to an open-ended design project. 

# Project Constraints

Key constraints we designed around included:
- **Blade span $\le 6\,\text{in}$**, **root compatible with a $1\,\text{in}$ hub**, and **speed $\le 3000\,\text{RPM}$** for safe operation
- **Resin 3D-printed blades** (grey resin) and a required **factor of safety ($\text{FOS} = 1.5$)**
- Wind tunnel operating conditions characterized using a **Weibull wind-speed distribution**, with **mean wind speed $4.782\,\text{m/s}$** and **$\pm 1.052\,\text{m/s}$** standard deviation

---

# Design process

## 1) Modeling approach
We combined:
- **$1\text{-D}$ momentum theory** (axial induction factor / Betz-limit framework)
- **Blade Element Momentum (BEM) theory** to connect flow conditions and airfoil performance to a spanwise blade geometry (chord and twist)

We targeted a **tip-speed ratio ($\text{TSR} = 6$)** as a high-efficiency operating point, then swept candidate operating points around that target.

## 2) Airfoil choice
We selected **NACA 66(1)-212**, motivated by:
- Similarity to airfoils already proven reliable in the lab (NACA 4412 / 0012)
- A slightly higher **lift-to-drag ratio** near our expected operating angles of attack

## 3) Optimization loop (geometry selection)
A MATLAB script searched over **rotor speed ($\Omega$)** and **angle of attack ($\alpha$)** to maximize predicted power:
- Discretized the blade into **$40$ radial stations**
- Swept **$\alpha$ from $6^\circ$ to $12^\circ$**
- Used **low-Re XFOIL polar data** for $C_L(\alpha)$ and $C_D(\alpha)$
- Computed spanwise inflow angle and set twist via **$\beta(r) = \phi(r) - \alpha$**
- Computed an optimal chord distribution based on Betz-Glauert and integrated tangential force → **torque**, then **power** $P = \Omega T$, selecting the best case

## 4) Structural check + CAD
We ran a simplified bending-stress estimate along the blade radius to check structural margin under expected conditions, then moved into manufacturing with appropriate caution about modeling simplifications.

The final blade geometry was lofted in **Fusion 360** using interpolated **NACA 66(1)-212** sections with the optimized chord and twist profiles.

---

# Testing summary

## Test method (wind tunnel)
We tested across multiple wind tunnel fan settings to capture performance outside a single “design point”:
- Zeroed the pressure transducer, then increased fan speed until the turbine began spinning (below $\sim 3\,\text{Hz}$ it did not spin)
- At each fan speed, incremented the torque brake voltage by **$0.4\,\text{V}$**, waited for steady RPM, recorded data, and continued until the rotor **stalled** (interpreted as reaching max power) or the brake hit its voltage limit

## Data products (what we analyzed)
We generated **power curves** at **$4\,\text{Hz}$, $6\,\text{Hz}$, $8\,\text{Hz}$, and $10\,\text{Hz}$**, corresponding to wind speeds of **$2.18\,\text{m/s}$, $3.17\,\text{m/s}$, $4.33\,\text{m/s}$, and $5.394\,\text{m/s}$**.

Key outcomes reported:
- At **$4\,\text{Hz}$** and **$6\,\text{Hz}$**, the turbine **stalled before** reaching the **$10\,\text{V}$** brake limit (so max power could be identified in those trials)
- At **$8\,\text{Hz}$** and **$10\,\text{Hz}$**, the turbine **did not stall** before hitting the brake limit, indicating additional power capability beyond the test hardware limit
- With a **$9.6\,\text{V}$** brake max setting, the **$8\,\text{Hz}$** and **$10\,\text{Hz}$** trials produced **$0.73\,\text{W}$ at $1126\,\text{RPM}$** and **$1.4\,\text{W}$ at $2213\,\text{RPM}$**, respectively
- The **maximum torque ($0.0062\,\text{N}\cdot\text{m}$)** occurred near the wind speed closest to the optimizer’s design point, supporting the modeling/optimization approach
- The most efficient reported case was the **$8\,\text{Hz}$ ($4.33\,\text{m/s}$)** condition, with an example efficiency calculation yielding **$C_P \approx 21\%$**

## What we’d improve next
Proposed next iteration ideas:
- Retune tower height to push anti-resonance near peak-power RPM
- Use a higher-torque brake and improve hub–blade tolerancing
- Refine CAD loft resolution and smooth hub/tip transitions to reduce parasitic drag

# My contribution: 

I worked primarily on the Matlab optimization pipeline.
