#include <ServoT3.h>
#include <Firmata.h>

// Version Information
#define FIRMWARE_VER_MAJOR  1
#define FIRMWARE_VER_MINOR  0

// Minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL   1

/*
Arduino Pin | Virtual Pin | Type
------------------------------------
 4          | 0           | Digital
 5          | 1           | Digital
 6          | 2           | Digital
 7          | 3           | Digital
 8          | 4           | Digital
 14         | S0 (5)      | Servo
 15         | S1 (6)      | Servo
 16         | S2 (7)      | Servo
 17         | S3 (8)      | Servo
 18         | A0 (9)      | Analog
 19         | A1 (10)     | Analog
 20         | A2 (11)     | Analog
 21         | A3 (12)     | Analog
 22         | A4 (13)     | Analog
 23         | A5 (14)     | Analog

 Analog pins 0 - 5 (A0 - A5)
*/
#define BRD_MAX_SERVOS  4
#define BRD_TOTAL_PINS  15
#define BRD_TOTAL_PORTS 3

#define BRD_IS_DIGITAL_PIN(p)   ((p) >= 0 && (p) < 5) // 5 DIO pins only
#define BRD_IS_ANALOG_PIN(p)    ((p) >= 9 && (p) < TOTAL_PINS)
#define BRD_IS_SERVO_PIN(p)     ((p) >= 5 && (p) <= 8)
#define BRD_PIN_TO_ANALOG(p)    (p) - 9

// NOTE: We can probably pre-attach all 4 servo ports

#define BRD_NUM_DIGITAL_PINS    5
#define BRD_NUM_SERVO_PINS      6 // 4 actual, 2 virtual pins that pipe to M1/M2
#define BRD_NUM_ANALOG_PINS     6

// Map from virtual pin numbers to actual pins
byte digitalPinMap[BRD_NUM_DIGITAL_PINS] = {4, 5, 6, 7, 8}; // TODO to test the OUTPUT capability, assign each index in turn to pin 13
byte servoPinMap[BRD_NUM_SERVO_PINS] = {127, 127, 14, 15, 16, 17};
byte analogPinMap[BRD_NUM_ANALOG_PINS] = {A0, A1, A2, A3, A4, A5};

/*===============================================================
 * GLOBAL VARIABLES
 *=============================================================*/
#ifdef FIRMATA_SERIAL_FEATURE
SerialFirmata serialFeature;
#endif

// Analog Inputs
int analogInputsToReport = 0; // bitwise array to store pin reporting

// Digital Input ports
byte reportPins = 0;       // 1 = report this port, 0 = silence
byte previousDigitalPins = 0;     // previous 8 bits sent

// Pin configuration
byte digitalPortConfigInput = 0; // we only have 1 virtual "port" since there are only 5 DIOs

// Timer variables
unsigned long currentMillis;        // Store the current value from millis()
unsigned long previousMillis;       // For comparison with currentMiilis
unsigned int samplingInterval = 19; // how often to run the main loop (in ms)

Servo servos[BRD_MAX_SERVOS];
byte detachedServoCount = 0;
byte servoCount = 0;

boolean isResetting = false;

void outputDigitalPort(byte portValue, byte forceSend) {
    // Pins not configured as INPUT are cleared to zeros
    portValue = portValue & digitalPortConfigInput;

    if (forceSend || previousDigitalPins != portValue) {
        Firmata.sendDigitalPort(0, portValue);
        previousDigitalPins = portValue;
    }
}

byte readDigitalPort() {
    byte portValue = 0;
    for (byte i = 0; i < BRD_NUM_DIGITAL_PINS; i++) {
        portValue |= ((digitalRead(digitalPinMap[i]) == HIGH ? 1 : 0) << i);
    }

    return portValue;
}

void checkDigitalInputs(void) {
    // Since we only have 1 virtual port, just do a naive digitalRead on all of it
    byte portValue = readDigitalPort();

    outputDigitalPort(portValue, false);
}

/**
 * Sets the pin mode to the correct state and sets the relevant bits in the
 * bit-array that tracks Digital I/O status.
 *
 * We only allow setting modes on Digital IO pins here. All analog pins will
 * remain as analog, and all servo pins remain as servos
 */
void setPinModeCallback(byte pin, int mode) {
    // TODO: Maybe we don't need this function? This is a custom board with very
    // specific layout. The only thing we need to do is to set the direction of
    // the DIO pins.
    if (BRD_IS_DIGITAL_PIN(pin)) {
        if (mode == INPUT || mode == PIN_MODE_PULLUP) {
            digitalPortConfigInput |= (1 << (pin & 7));
        }
        else {
            digitalPortConfigInput &- (1 << (pin & 7));
        }
    }

    Firmata.setPinState(pin, 0);

    switch (mode) {
        case PIN_MODE_ANALOG:
            if (BRD_IS_ANALOG_PIN(pin)) {
                Firmata.setPinMode(pin, PIN_MODE_ANALOG);
            }
            break;
        case INPUT:
            if (BRD_IS_DIGITAL_PIN(pin)) {
                pinMode(digitalPinMap[pin], INPUT); // disable output driver
                Firmata.setPinMode(pin, INPUT);
            }
            break;
        case PIN_MODE_PULLUP:
            if (BRD_IS_DIGITAL_PIN(pin)) {
                pinMode(digitalPinMap[pin], INPUT_PULLUP);
                Firmata.setPinMode(pin, PIN_MODE_PULLUP);
                Firmata.setPinState(pin, 0);
            }
            break;
        case OUTPUT:
            if (BRD_IS_DIGITAL_PIN(pin)) {
                pinMode(digitalPinMap[pin], OUTPUT);
                Firmata.setPinMode(pin, OUTPUT);
            }
        default:
            Firmata.sendString("Unsupported pin mode");
    }
}

void setPinValueCallback(byte pin, int value) {
    if (BRD_IS_DIGITAL_PIN(pin)) {
        if (Firmata.getPinMode(pin) == OUTPUT) {
            Firmata.setPinState(pin, value);
            digitalWrite(digitalPinMap[pin], value);
        }
    }
}

void analogWriteCallback(byte pin, int value) {
    if (pin < BRD_NUM_SERVO_PINS) {
        // Special case for pin 0 and 1, which map to motors
        if (pin < 2) {
            // TODO motor control here

        }
        else {
            // Write the angle to the servo
            // NOTE: the `servos` array only holds physical servo objects, hence
            // we need to drop the pin-count to get it back to zero based
            servos[pin-2].write(value);
            Firmata.setPinState(servoPinMap[pin], value);
        }
    }
}

void digitalWriteCallback(byte port, int value) {
    // We get sent a port (8 bits) to write to. This does NOT map to the
    // physical port. We'll probably ALWAYS get port 0, and value is a bitmask
    // of what to write

    for (byte pin = 0; pin < BRD_NUM_DIGITAL_PINS; pin++) {
        byte val = (value >> pin) & 0x1;
        setPinValueCallback(pin, val == 0 ? LOW : HIGH);
    }
}

void reportAnalogCallback(byte analogPin, int value) {
    if (analogPin < BRD_NUM_ANALOG_PINS) {
        if (value == 0) {
            analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
        }
        else {
            analogInputsToReport = analogInputsToReport | (1 << analogPin);

            // prevent during system reset, or all analog pins will be reported
            // which might report noise for unconnected pins
            if (!isResetting) {
                // Send pin value immediately
                Firmata.sendAnalog(analogPin, analogRead(analogPin));
            }
        }
    }
}

void reportDigitalCallback(byte port, int value) {
    // We are only getting port 0, and value is a bitmask of which pins to read
    reportPins = (byte)value;

    if (value) {
        // Send the port value right away
        outputDigitalPort(readDigitalPort(), true);
    }
}

void systemResetCallback() {
    isResetting = true;

    // TODO: Shutdown motor controllers

    reportPins = 0;
    previousDigitalPins = 0;
    digitalPortConfigInput = 0;

    for (byte i = 0; i < BRD_TOTAL_PINS; i++) {
        // Pins with analog capability default to analog input
        if (BRD_IS_ANALOG_PIN(i)) {
            // turn off pullup, configures everything
            setPinModeCallback(i, PIN_MODE_ANALOG);
        }
        else if (BRD_IS_DIGITAL_PIN(i)) {
            setPinModeCallback(i, OUTPUT);
        }
    }

    // By default, do not report any analog Inputs
    analogInputsToReport = 0;

    detachedServoCount = 0;
    servoCount = 0;

    isResetting = false;
}

void sysexCallback(byte command, byte argc, byte *argv) {
    switch (command) {
        case I2C_REQUEST:
            Firmata.sendString("I2C not supported");
            break;
        case I2C_CONFIG:
            Firmata.sendString("I2C_CONFIG not supported");
            break;
        case SERVO_CONFIG:
            // Map from Servo/PWM pins -> actual HW pins
            // TODO Implement
            if (argc > 4) {
                byte pin = argv[0];
                int minPulse = argv[1] + (argv[2] << 7);
                int maxPulse = argv[3] + (argv[4] << 7);

                Firmata.sendString("SEVO_CONFIG called");

                // TODO Remember to ignore pin 0 and 1
            }

            break;
        case EXTENDED_ANALOG:
            // ???
            break;
        case CAPABILITY_QUERY:
            // TODO Implement
            // We will need to show the valid list of ports
            // Basically implement the IS_PIN_* family of functions
            Firmata.write(START_SYSEX);
            Firmata.write(CAPABILITY_RESPONSE);
            for (byte pin = 0; pin < BRD_TOTAL_PINS; pin++) {
                if (BRD_IS_DIGITAL_PIN(pin)) {
                    Firmata.write((byte)INPUT);
                    Firmata.write(1);
                    Firmata.write((byte)PIN_MODE_PULLUP);
                    Firmata.write(1);
                    Firmata.write((byte)OUTPUT);
                    Firmata.write(1);
                }
                if (BRD_IS_ANALOG_PIN(pin)) {
                    Firmata.write(PIN_MODE_ANALOG);
                    Firmata.write(10);
                }
                if (BRD_IS_SERVO_PIN(pin)) {
                    Firmata.write(PIN_MODE_SERVO);
                    Firmata.write(14);
                }
                Firmata.write(127);
            }
            Firmata.write(END_SYSEX);
            break;
        case PIN_STATE_QUERY:
            if (argc > 0) {
                byte pin = argv[0];
                Firmata.write(START_SYSEX);
                Firmata.write(PIN_STATE_RESPONSE);
                Firmata.write(pin);
                if (pin < TOTAL_PINS) {
                    Firmata.write(Firmata.getPinMode(pin));
                    Firmata.write((byte)Firmata.getPinState(pin) & 0x7F);
                    if (Firmata.getPinState(pin) & 0xFF80) Firmata.write((byte)(Firmata.getPinState(pin) >> 7) & 0x7F);
                    if (Firmata.getPinState(pin) & 0xC000) Firmata.write((byte)(Firmata.getPinState(pin) >> 14) & 0x7F);
                }
                Firmata.write(END_SYSEX);
            }
            break;
        case ANALOG_MAPPING_QUERY:
            Firmata.write(START_SYSEX);
            Firmata.write(ANALOG_MAPPING_RESPONSE);
            for (byte pin = 0; pin < BRD_TOTAL_PINS; pin++) {
                Firmata.write(BRD_IS_ANALOG_PIN(pin) ? BRD_PIN_TO_ANALOG(pin) : 127);
            }
            Firmata.write(END_SYSEX);
            break;
    }
}

void setup() {
    // Controller Firmware Version
    Firmata.setFirmwareNameAndVersion("bbfrc-astar-32u4-rc", FIRMWARE_VER_MAJOR, FIRMWARE_VER_MINOR);

    // Attach servos
    // We start indexing at 2 because BRD_NUM_SERVO_PINS refers to the
    // TOTAL number of servo pins available (including virtual pins)
    for (byte i = 2; i < BRD_NUM_SERVO_PINS; i++) {
        // NOTE: The servo channels 0 and 1 are reserved for the motors
        // Thus, the map of logical servo pins to physical pins is
        // offset by 2
        Firmata.setPinMode(servoPinMap[i], PIN_MODE_SERVO);

        // the `servos` array only holds the number of physical servos
        // hence the need to adjust the index
        // .attach() essentially tells the Servo library that the pin
        // specified should be used as a servo output
        servos[i-2].attach(servoPinMap[i]);
    }

    // Attach message callbacks
    Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
    Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
    Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
    Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
    Firmata.attach(SET_PIN_MODE, setPinModeCallback);
    Firmata.attach(SET_DIGITAL_PIN_VALUE, setPinValueCallback);
    Firmata.attach(SYSTEM_RESET, systemResetCallback);
    Firmata.attach(START_SYSEX, sysexCallback);

    // Start!
    Firmata.begin(57600);
    while (!Serial) {
        ; // wait for serial port to connect. Needed on ATMega32u4 boards
    }

    // Reset the system to default configuration
    systemResetCallback();
}

void loop() {
    byte pin, analogPin;
    // Digital Read as fast as possible, check for changes, and output
    checkDigitalInputs();

    while (Firmata.available()) {
        Firmata.processInput();
    }

    currentMillis = millis();
    if (currentMillis - previousMillis > samplingInterval) {
        previousMillis += samplingInterval;

        // Analog Read
        for (pin = 0; pin < BRD_TOTAL_PINS; pin++) {
            if (BRD_IS_ANALOG_PIN(pin) && Firmata.getPinMode(pin) == PIN_MODE_ANALOG) {
                analogPin = BRD_PIN_TO_ANALOG(pin);
                if (analogInputsToReport & (1 << analogPin)) {
                    Firmata.sendAnalog(analogPin, analogRead(analogPin));
                }
            }
        }
    }
}
