# UHCC_Imua_Arliss_2018

This repository contains code that was used to pilot the UHCC ARLISS 2018 quadcopter and is designed for autonomous flight, not for remote control, but can be adapted to take inputs from a controller.

## Getting Started

All code assumes you are using the same quadcopter components.

- 3s 11.1v 2000mAh lipo battery
- Little Bee 30A ESC
  - signal on pins 10, 11, 12, 13
  - 10: Front Left    Clockwise
  - 11: Front Right   Counter-Clockwise
  - 12: Back Left     Counter-Clockwise
  - 13: Back Right    ClockWise
- Feather M0 Adalogger
- Adafruit 10-DOF IMU Breakout - L3GD20H + LSM303 + BMP180
- Adafruit Ultimate GPS v3
- a voltage divider on pin A0
  - resistor 1: 14.7kOhms
  - resistor 2: 4/7kOhms

*all other components are user prefference

Other Components used specifically for the UHCC ARLISS quadcopter.
- 5x3 in carbon fiber props
- Carbon Fiber Mini 250 FPV Quadcopter Frame
- a dead trigger ( pull-up ) on pin 9
  - used to detect deployment from the ARLISS rocket

### Prerequisites

Requires the Arduino IDE to compile and the libraries needed to use the Adafruit 10-DOF IMU Breakout, and Ultimate GPS v3.

### Installing

A step by step series of examples that tell you how to get a development env running

Step 1: Install Arduino IDE

```
//TODO Give the example
```

Step 2: Importing/Running code

```
//TODO until finished
```

an example

## Running the tests

//TODO Explain how to run the automated tests for this system

### Break down into end to end tests

//TODO Explain what these tests test and why

```
//TODO Give an example
```

### And coding style tests

//TODO Explain what these tests test and why

```
//TODO Give an example
```

## Deployment

//TODO How to develop

## Built With
- [Arduino](https://www.arduino.cc/) - compiler

## Contributing

//TODO 

## Versioning

//TODO 

## Authors

//TODO 

## License

- This project is licenced under the GNU GENERAL PUBLIC LICENSE - see the [LICENSE.md](LICENSE.md) file for details


## Acknowledgments

* //TODO 

