# STM32 FreeRTOS LED Control Program

## Overview

This project is an STM32-based application using FreeRTOS to demonstrate multitasking and shared resource access. It involves controlling three LEDs (Green, Red, and Blue) and managing a critical section in a multitasking environment. The program uses two FreeRTOS threads (GreenLedTask and RedLedTask) to simulate concurrent access to a shared resource, represented by the AccessSharedData function. A blue LED indicates resource contention.

## Features

- **FreeRTOS Multitasking**:
  - Three tasks (GreenLedTask, RedLedTask, and OrangeLedTask) run independently, controlling Green, Red, and Orange LEDs.

- **Critical Section Management**:
  - Shared resource access is protected using FreeRTOS critical section APIs (`taskENTER_CRITICAL` and `taskEXIT_CRITICAL`).

- **Contention Handling**:
  - A blue LED is turned on if resource contention is detected.

## Hardware Requirements

- **STM32 microcontroller** (e.g., STM32F4xx series)
- **Four LEDs connected to GPIO pins**:
  - Green LED: `GPIOA Pin 0`
  - Red LED: `GPIOA Pin 1`
  - Blue LED: `GPIOA Pin 2`
  - Orange LED: `GPIOA Pin 3`
- **FreeRTOS** enabled in the project

## File Structure

- **`main.c`**:
  - Contains the main program logic, FreeRTOS task definitions, and critical section handling.
- **CMSIS/FreeRTOS Files**:
  - FreeRTOS kernel files for multitasking.

## Setup and Configuration

### GPIO Configuration

1. Enable GPIOA clock in the `MX_GPIO_Init` function.
2. Configure GPIO pins for the LEDs:
   - **Mode**: Output Push-Pull
   - **Speed**: Low frequency
   - **Pull**: No pull-up/down resistors

### FreeRTOS Configuration

1. Add FreeRTOS middleware to your STM32CubeMX project.
2. Define three tasks in the `osThreadNew` function:
   - **GreenLedTask**: Controls the Green LED.
   - **RedLedTask**: Controls the Red LED.
   - **OrangeLedTask**: Toggles the Orange LED at a fixed interval.
3. Set task priorities and stack sizes.

## Task Descriptions

### GreenLedTask

- Turns on the Green LED.
- Accesses the shared resource using `AccessSharedData`.
- Delays for 200ms before repeating.

### RedLedTask

- Turns on the Red LED.
- Accesses the shared resource using `AccessSharedData`.
- Delays for 550ms before repeating.

### OrangeLedTask

- Toggles the Orange LED at a 50ms interval without accessing shared resources.

## Shared Resource: AccessSharedData

This function simulates accessing a shared resource with contention handling:

- Checks the `StartFlag` variable to determine if the resource is free.
- Simulates read/write operations with a delay.
- Indicates contention by turning on the Blue LED.

## How to Build and Run

1. Open the project in your STM32 IDE (e.g., STM32CubeIDE).
2. Build the project to generate the firmware.
3. Flash the firmware to the STM32 microcontroller.
4. Monitor LED behavior:
   - Green and Red LEDs blink according to their respective task delays.
   - Orange LED toggles continuously.
   - Blue LED lights up briefly if resource contention occurs.

## Known Issues

- Resource contention is handled only through a visual indicator (Blue LED). Further enhancements can include using semaphores or mutexes.

## Future Enhancements

- Replace critical section handling with FreeRTOS semaphores or mutexes for more robust resource management.
- Add UART communication to log task behavior.
- Integrate an OLED display to visualize task status and resource access logs.

https://github.com/user-attachments/assets/d9279293-9866-43a3-8586-8fff6c2c31cd

