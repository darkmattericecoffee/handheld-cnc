## Description
Compass is a DIY handheld CNC router designed to make CNC machining more accessible. Unlike traditional CNC machines that move a cutting tool within a fixed workspace, Compass relies on users to guide the device around the workpiece directly. It automatically adjusts the cutting tool to stay on the programmed design path, enabling a significantly smaller device footprint while still handling large-scale cuts.

This is all accomplished using optical flow mouse sensors to track the position of the device, as well as a lead screw driven gantry to adjust the tool. Everything is driven by a Teensy 4.1 microcontroller.

## Getting Started
All of the source to build a Handheld CNC Router - the firmware, CAD files for 3D printing/manufacturing, BOM, and cable diagrams - are all in this repo. Before you continue, make sure you understand the following disclaimer.

*Disclaimer:* This project is still a work-in-progress. Development is rapidly occuring, so design files and firmware may change.

### Firmware
The main firmware is written for [PlatformIO](https://docs.platformio.org/en/latest/core/quickstart.html) (as opposed to Arduino IDE) for better project organization and explicit library management.

### Hardware
Most 

### Electronics

### BOM and Other Parts

## Debugging
To produce a debug logFile to use with `animate.ipynb`, make sure the variable `outputMode` in your firmware is set to `1`. Then run the following code in a *platformIO terminal*:
```
pio device monitor > /logFiles/logFile_XX.txt
```