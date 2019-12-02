# BB FRC Workshop Controller - Pololu A-Star 32u4 Robot Controller
This firmware is designed for the [Pololu A-Star 32u4 Robot Controller](https://www.pololu.com/product/3117). It contains custom mappings for PWM ports, Digital I/O ports and Analog Input ports to support the built-in functionality of the board.

## Required Arduino Libraries
- [ServoT3](https://github.com/bb-frc-workshops/controller-firmware/tree/master/libs/ServoT3) - Modified build of Servo library that uses Timer3
- [AStar32U4 Arduino Library](https://www.pololu.com/docs/0J61/7) - (Install via instructions in link) Library that manages onboard hardware of the A-Star 32u4 Robot Controller

## Port Mapping
![Pololu A-Star 32u4 Robot Controller Pinout](https://a.pololu-files.com/picture/0J6675.1200.jpg?41fd5790d0536c3a42cf478ba6d4edbf)
The diagram above shows the pin mapping on the Pololu A-Star Robot Controller. This firmware uses a subset of those pins. The table below shows the pins that are in use and their virtual functions as defined in the firmware.

| Arduino Pin | Virtual Pin | Type    |
|-------------|-------------|---------|
|  4          | 0           | Digital |
|  5          | 1           | Digital |
|  6          | 2           | Digital |
|  7          | 3           | Digital |
|  8          | 4           | Digital |
|  14         | S2 (5)      | Servo   |
|  15         | S3 (6)      | Servo   |
|  16         | S4 (7)      | Servo   |
|  17         | S5 (8)      | Servo   |
|  18         | A0 (9)      | Analog  |
|  19         | A1 (10)     | Analog  |
|  20         | A2 (11)     | Analog  |
|  21         | A3 (12)     | Analog  |
|  22         | A4 (13)     | Analog  |
|  23         | A5 (14)     | Analog  |

 The firmware provides 15 pins, with 5 Digital I/O pins, 6 servo pins (2 are virtual, and are used to control the motors) and 6 analog inputs.

 Contrary to how `StandardFirmata` works (where all pins can act as digital I/O), the custom firmware here only allows Digital Pins to be configured as input/output, while all other pins (servo, analog) are hard-coded to only respond to the behavior that they are assigned to.

 ## Hardware Assembly
 Some assembly work needs to be done for each board. This involves soldering pin headers on the various functional pins indicated in the section above, which would allow standard 3-pin servo connectors to be used to hook up sensors, servos and actuators.

 ![](https://a.pololu-files.com/picture/0J6677.1200.jpg?6f41519bf4e3def24cfaf1c73544139a)

 As shown in this diagram, the power rails are split and thus need to be bridged. These bridged power rails then need to be connected to the `5V` (VCC) distribution pins.

 ## Firmata Implementation
 The firmware takes many cues from `StandardFirmata`, with the exception of having a custom set of pins and functional mappings that get sent to the client during the `CAPABILITY_QUERY` part of the handshake. Most of the functionality that Firmata offers is available here, with mapping to physical pins happening under the covers.