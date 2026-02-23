# STM32_HAL_Project

## Introduction
This project develops a smart sensor system based on the STM32 microcontroller. The system consists of tasks for reading temperature, humidity, light, and ultrasonic sensors; logic control for buzzer, door, and light; and displaying data on LCD and TFT screens. The DHT11, HCSR05, TFT, and LCD sensors are supported by custom-built HAL-level drivers. This is a basic STM32 project for practicing the use of protocols such as SPI, I2C, UART, Timer, PWM, and ADC.

## Task Structure

### 1. Sensor_Task
- Read DHT11 (temperature, humidity)
- Read light via ADC
- Read HC-SR05 (distance)
- Send struct to queue:

```c
typedef struct {
    uint8_t temperature;
    uint8_t humidity;
    uint16_t light;
    float distance;
} SensorData_t;
```

### 2. Control_Task
- Receive data from queue
- If temperature > threshold → turn on buzzer
- If distance < 10cm → open servo
- If distance < 10cm & low light → turn on light

### 3. Display_Task
- Display data on LCD
- Show door status on TFT
---

## Highlights
- **Driver abstraction**: Drivers for DHT11, HCSR05, LCD, and TFT are built at the HAL level, making them easy to reuse and extend.
- **Clear task management**: Tasks are clearly separated, using queues to transfer data between tasks.
- **Protocol practice**: Utilizes SPI, I2C, UART, Timer, PWM, and ADC on STM32.

## Main Directories
- `Core/` : Main code, STM32 configuration
- `myLib/` : Custom libraries for sensors and peripherals
- `freertos/` : FreeRTOS kernel and configuration
- `Drivers/` : HAL and CMSIS drivers

## Hardware Requirements
- STM32F103C8T6 (Blue Pill)
- DHT11 sensor, HC-SR05, light sensor, LCD 1602, TFT 1.8"
- Servo90, Buzzer, LED