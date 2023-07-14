# CPP Deployment Guide

This will be a relatively loose guide of how to run your shit properly with c++ using VS Code.

## Big Idea

- Download [VSCode](https://code.visualstudio.com/)
- From withing VSCode you can install the [Platform IO](https://platformio.org/) extension. This is what let's you compile and flash cpp code to the Teensy. It takes care of all the backend configuration and driver stuff, so we can just run it as easily as the Arduino IDE...
- Write code like normal! We have `Arduino.h` imported by default, so you don't have to write cpp code for everything: for example you can still just write `Serial.println("hello world")` like with Arduino. But you now notice that our main file is called `main.cpp`! It is now a proper `.cpp` file, not an `.ino` file.

## Useful Stuff

- Check out [this tutorial](https://docs.platformio.org/en/latest/integration/ide/pioide.html) for getting started with Platform IO. It allows some pretty powerful debugging and other features.

## To Do

- Cam: successfully build and upload this project to Teensy.