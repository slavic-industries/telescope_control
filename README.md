# Telescope Control

Telescope Control is a program that is used to control any telescope mount (Eqatorial ans Alt-AZ)
that is fitted with the Telescope Control hardware. The brains of the controller is a Raspberry Pi 4
and the movement is done by stepper motors attached to the telescope mount. 

To Do:
- [ ] Make rough layout of the architecture
  - [ ] Setup nodes
  - [ ] Setup communication between nodes
  - [ ] Setup communication to motor drivers
- [ ] Figure out hardware
- [ ] Make hardware schematics

### Road Map

(Roughly outline the steps that need to be done to get an MVP)
- [ ] Make rough layout of the architecture (Using ROS2)
  - [ ] Setup nodes
  - [ ] Setup communication between nodes
  - [ ] Setup communication to motor drivers
- [ ] Figure out hardware
  - [x] Find suitable stepper motor drivers -> TMC2130
  - [x] Find a way to easily control the movement of the steppermotors -> TMC429
  - [x] Test the hardware compatibility -> Quick arduino program
- [ ] Figure out firmware
  - [ ] Setup TMC2130 communication
  - [ ] Setup TCM429 communication
  - [ ] Setup SI5351 communication

- [ ] Make hardware schematics
- [ ] Design PCB hat for RPi headers


### Telescope control Hardware

- 2 x Bipolar stepper motor
- 2 x TMC2130 Stepper motor driver
- 1 x TMC429 Stepper motor controller
- 1 x SI5351 Clock generator for TCM429
- 1 x Raspberry Pi 4 Model B
- 1 x Bread board
- Jumper wires