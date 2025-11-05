# STM32 DHT11 Driver Library and Demo

This repository serves two purposes:

1. **Providing a robust, reusable DHT11 driver library** for STM32 microcontrollers.

2. **Demonstrating its usage** in an application that logs data via UART.

The core DHT11 logic is abstracted into a custom library to ensure clean separation from the main application code.

## Project Structure and Driver Location

The core driver files are located in the `Libraries` folder, making them modular and easy to include in other projects.

| **Location** | **File** | **Description** |
| :--- | :--- | :--- |
| `Libraries/Inc/` | `dht11.h` | **The Driver Header File.** Defines the `dht11_handle_t` struct, status enumerations, and the public API for the driver. |
| `Libraries/Src/` | `dht11.c` | **The Core Driver Implementation.** Contains the logic for the precise timing sequence, GPIO mode switching, data bit extraction, and checksum validation. |
| `Core/Src/` | `main.c` | **Demonstration Application.** Initializes peripherals, calls the DHT11 driver functions, includes error handling, and prints continuous readings via UART. |

## Getting Started

### Hardware Requirements

* **Microcontroller:** STM32F411RE Nucleo Board (or any compatible STM32F4 microcontroller)
* **Sensor:** DHT11 Temperature and Humidity Sensor 
* **Connections:** See **Pinout Configuration** below.

### Software Requirements

* STM32CubeIDE (or compatible ARM-based IDE)
* GNU ARM Embedded Toolchain

### Timer Configuration
Configure a timer running at exactly 1 MHz (clock cycle of 1 μs). Refer the "Timer Configuration" section in [this](https://stm32world.com/wiki/STM32_Microsecond_Delay) link for more information.

### Pinout Configuration

The demo application is configured to use the following pins on the STM32:

| **DHT11 Pin** | **STM32 Pin** |
| :--- | :--- |
| **Data** | GPIOA Pin 1 (PA1) |
| **VCC** | Any 5V Pin |
| **GND** | Any GND Pin |

## Driver API

The following are the primary functions provided by the DHT11 driver library:

### `dht11_init`

Initializes the sensor handle, linking it to the specified hardware peripherals (GPIO and Timer).

```c
void dht11_init(dht11_handle_t *handle, GPIO_TypeDef *dht11_port, uint16_t dht11_pin, TIM_HandleTypeDef *dht11_tim);
```

### `dht11_read`

Executes the full 40-bit reading cycle, including sending the start signal, processing the sensor's response, extracting the data, and performing checksum validation.

```c
dht11_response dht11_read(dht11_handle_t *handle);
```

## Compilation and Running

### DHT11 Driver
* Clone this repository locally.
* Copy the `Libraries` folder into your workspace.
* Add the `Libraries\Inc` folder path into the project (`Properties -> C/C++ General -> Paths and Symbols -> Includes -> Add`).
* Add the `Libraries\Src` those as source Locations (`Properties -> C/C++ General -> Paths and Symbols -> Source Locations -> Add Folder`).

### Complete Project
* Clone this repository locally.
* Import the project into `STM32CubeIDE using File > Import... > General > Existing Projects into Workspace`.
* Ensure your target hardware configuration (`STM32F411RE`) matches your board.
* Build the project.
* Flash the generated `.elf` file onto your STM32 using an ST-Link/J-Link debugger.
* Open a serial terminal configured to `115200` Baud to view the live temperature and humidity data.