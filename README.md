# CAN-LIB
## General CAN library for Purdue Solar Racing
### Supported Microcontrollers
 - STM32F0-4
 - Arduino Due
 - Arduino Uno (with MCP2515)
 - Arduino Mega (with MCP2515)
 - ~~Raspberry Pi Pico~~ **(Planned)**

### Including library
Microcontroller type must be defined before including any library headers.

Example:
```c
#define BOARD_STM32F 3 // Board Selection
#include "can_lib.h"
```

Pre-processor definitions are also required during compilation.

Example:
```
$ gcc <source-files> -DBOARD_STM32F=4 -o program
```

### Microcontroller defines
| Microcontroller	| C Define Statement 			|
| ----------------- | ----------------------------- |
| STM32F0xx			| `#define BOARD_STM32F 0`		|
| STM32F1xx			| `#define BOARD_STM32F 1`		|
| STM32F2xx			| `#define BOARD_STM32F 2`		|
| STM32F3xx			| `#define BOARD_STM32F 3`		|
| STM32F4xx			| `#define BOARD_STM32F 4`		|
| Arduino Due		| `#define BOARD_ARDUINO_DUE`	|
| Arduino Uno		| `#define BOARD_ARDUINO_UNO`	|
| Arduino Mega		| `#define BOARD_ARDUINO_MEGA`	|
| Raspberry Pi Pico | `#define BOARD_RPI_PICO`		|