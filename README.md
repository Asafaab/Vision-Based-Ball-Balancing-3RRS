# Vision-Based Ball Balancing Using a 3-RRS Parallel Mechanism
## Mounted on a Differential-Drive Mobile Robot

<p align="center">
  <img src="Figures/A_Isometrica.png"
       alt="Isometric CAD view of the complete mobile ball-balancing robot"
       width="720">
</p>

<p align="center">
  <em>Isometric CAD representation of the complete robotic platform,
  integrating the differential-drive mobile base with the 3-RRS
  Ball-and-Plate mechanism.</em>
</p>

---

## Project Overview

This repository contains the mechanical design, CAD models, technical
drawings, software, electronic documentation, and supporting material
developed for the project:

**Vision-Based Ball Balancing Using a 3-RRS Parallel Mechanism Mounted
on a Differential-Drive Mobile Robot.**

The proposed system combines a differential-drive mobile robot with an
actively actuated 3-RRS parallel Ball-and-Plate mechanism. The objective
is to regulate the position of a ball on a movable platform using
vision-based feedback while maintaining the capability of mobile
locomotion.

The robotic architecture is divided into two main processing platforms.
An **NI myRIO** is used for differential-drive motor control, encoder
acquisition, and inclination measurement of the mobile base. A
**Raspberry Pi 5** processes the images acquired by an **Arducam OV9281
global-shutter camera** and controls the three servomotors responsible
for actuating the 3-RRS balancing mechanism.

The 3-RRS mechanism modifies the orientation of the upper platform in
roll and pitch, allowing the controller to compensate for ball-position
errors and disturbances introduced by the motion or inclination of the
mobile base.

## Main Features

- Differential-drive mobile robotic platform
- 3-RRS parallel Ball-and-Plate mechanism
- Vision-based ball position estimation
- Arducam OV9281 global-shutter camera
- Raspberry Pi 5 for image processing and platform actuation
- NI myRIO for mobile-base control and inclination measurement
- Encoder feedback for the driving motors
- CAD design and complete mechanical assembly in Autodesk Inventor
- STEP files for CAD interoperability
- Technical drawings and manufacturing documentation
- Modular architecture for independent development of locomotion,
  perception, and balancing control

## System Architecture

The project is organized into the following main subsystems:

- **Mobile base:** differential-drive locomotion using two actuated wheels
  and passive support wheels.
- **3-RRS mechanism:** parallel mechanism responsible for modifying the
  orientation of the Ball-and-Plate platform.
- **Vision subsystem:** ball detection and position estimation using the
  OV9281 camera.
- **Inclination measurement:** estimation of the mobile-base orientation
  using the NI myRIO onboard sensing system.
- **Control architecture:** distributed processing between NI myRIO and
  Raspberry Pi 5.
