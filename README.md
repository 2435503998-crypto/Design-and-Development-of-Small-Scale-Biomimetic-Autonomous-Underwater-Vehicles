# Biomimetic Robotic Fish

This project is a MSc project — ”Design and Development of Small-Scale Biomimetic   Autonomous Underwater Vehicles“ Aims to design a robot goldfish(~10 cm in length), controlled by an ESP32-C3 microcontroller.  
It implements closed-loop tail fin oscillation control based on angle sensor feedback and supports Bluetooth-based mode switching and data transmission.  
The outer shell and tail fin support structure are 3D-printed, resulting in a compact, modular, and easily maintainable design.

---

## Features
- **Closed-loop control**: PD control based on angle sensor feedback for precise tail oscillation.
- **Bluetooth communication**: Wireless mode switching (start, straight, turn, stop).
- **Modular design**: 3D-printed shell and tail support for easy assembly and replacement.
- **Visual debugging**: Serial output of reference angle, measured angle, and control signal for performance analysis.

---

## Repository Structure
```
RobotGoldfish/
│
├─ code/        # Control programs
├─ simulation/       # Simulink model and simulation results
├─ data/        # land_test.csv
├─ hardware/    # 3D models, BOM
├─ images/      # Photos, test results
└─ README.md    # Project description
```


## Hardware Requirements
- uPesy ESP32-C3 Mini board  
- Single-PWM-input motor driver  
- Angular position sensor  
- 3D-printed shell & tail support
- 3D-printed gears
- Li-ion battery & wireless charging module (optional)  

---

## Author
Haodi Zhang  
University of Glasgow  
2025
