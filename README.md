# Vision-Based Ball Balancing Using a 3-RRS Parallel Mechanism
## Mounted on a Differential-Drive Mobile Robot

This repository contains the mechanical design, manufacturing files,
software, electronic documentation, and supporting material developed
for the project:

**Vision-Based Ball Balancing Using a 3-RRS Parallel Mechanism
Mounted on a Differential-Drive Mobile Robot**

The system combines a differential-drive mobile robot with an actively
actuated 3-RRS Ball-and-Plate mechanism.

Ball position is estimated using computer vision with an Arducam OV9281
camera and a Raspberry Pi 5. The differential-drive mobile base is
controlled using an NI myRIO, while three servomotors actuate the 3-RRS
parallel mechanism.

## System architecture

The project is divided into the following main subsystems:

- Differential-drive mobile base
- 3-RRS Ball-and-Plate mechanism
- Vision-based ball position estimation
- Mobile-base inclination measurement
- Distributed control architecture

## Repository structure

```text
CAD/
    Complete_Robot/
    3RRS_Mechanism/
    Manufacturing_Files/

Figures/

Software/
    Raspberry_Pi/
    myRIO/

Electronics/

Documentation/
