# Microprocessors Laboratory @ NTUA

## Students (equal contribution):

- [Dimitrios Dimitrakopoulos](https://github.com/ddimitrakopoulos)

- [Dimitrios Saviolakis](https://github.com/dsaviolakis)

This repository contains laboratory exercises for the **Microprocessors Lab** course at the National Technical University of Athens (NTUA), focusing on programming the ATmega328PB microcontroller.

##  Exercises Overview

### Exercise 1 - Assembly Programming Fundamentals & Simulation
- Delay routine implementation with configurable timing
- Logical function calculations (F0, F1) with iterative variable updates
- Moving wagon simulation using PORTD with directional control
- MPLAB X simulation and Stopwatch tool usage

### Exercise 2 - External Interrupts
- INT0 and INT1 interrupt handling in Assembly and C
- Button debouncing techniques
- Binary counter with interrupt control
- Lighting automation system with timeout reset

### Exercise 3 - Timers & ADC (Analog-to-Digital Converter)
- Timer/Counter1 configuration for PWM generation
- Fast PWM mode with variable duty cycle
- ADC conversion with polling and interrupt methods
- Analog voltage measurement and LED indication

### Exercise 4 - 2×16 Character LCD Display
- LCD initialization and control routines
- Voltage measurement display with ADC
- CO gas monitoring system with threshold detection
- Display messages and sensor value presentation

### Exercise 5 - External Port Expansion (TWI/I²C)
- PCA9555 I/O expander configuration
- TWI (I²C) communication protocol implementation
- Logical function output through expander
- Keypad input handling and LCD interfacing

### Exercise 6 - 4×4 Keypad Interface
- Keypad scanning with debouncing
- ASCII conversion for key presses
- Electronic lock system with team code validation
- Real-time key display on LCD

### Exercise 7 - DS18B20 Temperature Sensor & 1-Wire Communication
- 1-wire protocol implementation
- Temperature measurement routines
- Decimal temperature display on LCD
- "NO Device" detection and reporting

### Exercise 8 - IoT Application - Hospital Monitoring System
- UART communication with ESP8266 WiFi module
- HTTP communication with cloud server
- Patient monitoring with temperature and pressure sensors
- JSON payload formatting and transmission
- Edge computing architecture implementation

## Technical Setup

### Hardware Requirements
- ntuAboard_G1 development board
- ATmega328PB microcontroller
- Various sensors: DS18B20, potentiometers, keypad, LCD
- ESP8266 WiFi module
- USB-to-serial adapter

### Software Requirements
- MPLAB X IDE
- AVR-GCC compiler
- Serial terminal software
