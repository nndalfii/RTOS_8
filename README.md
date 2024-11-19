# STM32 FreeRTOS LED Control Project

## Overview

This project demonstrates multithreading using FreeRTOS on an STM32 microcontroller. It controls three LEDs (`Red`, `Green`, `Orange`) and manages a shared resource using a semaphore to ensure synchronization between tasks.

---

## Features

1. **Multitasking with FreeRTOS**
   - Three tasks:
     - `RedLed`: Controls the red LED with periodic toggling.
     - `GreenLed`: Controls the green LED with periodic toggling.
     - `OrangeLed`: Toggles the orange LED continuously.
   - Tasks have different priorities and delays.

2. **Semaphore for Shared Resource**
   - A semaphore protects access to a simulated shared resource.
   - Prevents contention between `RedLed` and `GreenLed` tasks.
   - A blue LED indicates resource contention.

---

## Hardware Configuration

- **MCU:** STM32 (e.g., STM32F103C8T6)
- **GPIO Pins:**
  - `GPIOC_PIN_13`: Blue LED (contention indicator)
  - `GPIOC_PIN_14`: Green LED
  - `GPIOC_PIN_15`: Red LED
  - `GPIOA_PIN_0`: Orange LED

---

## Software Requirements

- **Toolchain:**
  - STM32CubeIDE
  - FreeRTOS Middleware
- **Libraries Used:**
  - `HAL` for GPIO operations
  - `CMSIS-RTOS2` for FreeRTOS APIs

---

## Functionality

### Tasks

1. **RedLed Task**
   - Turns the red LED on and off with a delay of 550 ms.
   - Accesses the shared resource and handles contention.

2. **GreenLed Task**
   - Turns the green LED on and off with a delay of 200 ms.
   - Accesses the shared resource similarly to the red LED.

3. **OrangeLed Task**
   - Continuously toggles the orange LED every 50 ms.

### Shared Resource

- Protected using a semaphore:
  ```c
  osSemaphoreAcquire(CriticalResourceSemaphoreHandle, WaitTimeMilliseconds);
  AccessSharedData();
  osSemaphoreRelease(CriticalResourceSemaphoreHandle);
---


https://github.com/user-attachments/assets/63b29438-1dc0-4c83-b6d0-ce122c078da6


