# Stop-Watch
![Version](https://img.shields.io/badge/Version-1-brightgreen)
![Author](https://img.shields.io/badge/Authors-Mohamed%20Mabrouk-blue)
![Date](https://img.shields.io/badge/Date-31%20JAN%202024-orange)       
Developing a system that control the stop-watch time and display it on 7-Segments.
## Table of Contents

- [Description](#description)
- [Features](#features)
- [Requirements](#requirements)
- [Usage](#usage)
- [Author](#author)

## Description

This project contains an implementation of a stop watch application for AVR platform using Atmega32 microcontroller. It utilizes timers, GPIOs, and interrupts to provide basic stop watch functionalities such as start, stop, pause, and reset.

## Features

- Start/Stop functionality.
- Pause functionality.
- Reset functionality.
- Displays time on a 7-segment display.
- Supports resume functionality.

## Requirements

- AVR development board with Atmega32 microcontroller.
- AVR toolchain.
- AVR programmer.

## Usage

1. Connect your AVR development board to your computer.
2. Compile the code using your AVR toolchain.
3. Upload the compiled binary to your AVR development board using an AVR programmer.
4. Power on your AVR development board.
5. Use external buttons connected to INT0, INT1, and INT2 pins to control the stop watch:
   - INT0: Reset button.
   - INT1: Pause button.
   - INT2: Resume button.

## Author

- [Mohamed Mabrouk](https://github.com/mohamed-mabrouk)


