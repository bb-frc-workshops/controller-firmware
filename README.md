# BB FRC Workshops - Robot Controller Firmware
This repository contains firmware and libraries for the robot controllers used in BB FRC Workshops.

## IMPORTANT NOTE
When using the A-Star Robot Controller board (https://www.pololu.com/product/3117), you should use the `ServoT3` library (contained in the `libs` folder) instead of the regular `Servo` library. This ensures that Timer3 is used for PWM generation and will not interfere with the motor control on that board (which uses Timer1). The `ServoT3` folder in `libs` should be copied to your Arduino libraries folder (on Windows, it is C:\Program Files (x86)\Arduino\libraries).

## Theory of Operation
The firmware here runs directly on microcontroller boards (e.g. Arduino, A-Star, etc) and serves as an interface between a _host_ and the actual hardware that makes up a _robot_. The controller firmware is responsible for taking in messages over serial (e.g. "drive digital pin 1 HIGH") and actually manipulating the hardware.

All the firmware uses the [Firmata Protocol](https://github.com/firmata/protocol). The protocol is well documented, and many client and hardware implementations already exist for it. Extensions to the protocol (implemented using `sysex messages`) will be documented below.

For a normal Arduino board (e.g. the Arduino Leornado or Uno), it is possible to just use the included `StandardFirmata` firmware and an appropriate port configuration to interact with robot hardware. For boards that act as custom robot controllers (e.g. Pololu A-Star Robot Controller), the firmware is modified to reflect the actual features on the board, and provide appropriate mappings (e.g. PWM 0 and 1 on this board will map to the M1 and M2 motor outputs).