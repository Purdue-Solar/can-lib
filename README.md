# CAN-LIB
## General CAN library for Purdue Solar Racing
### Supported Microcontrollers
 - STM32F3xx
 - STM32F4xx
 - Arduino Due

### Including library
Microcontroller type must be defined before including in source any library headers.

Example:
```c
#define BOARD_STM32F3 // Board Selection
#include "can_lib.h"
#include "can_types.h"
```

### Microcontroller defines
| Microcontroller	| C Define Statement 			|
| ----------------- | ----------------------------- |
| STM32F3xx			| `#define BOARD_STM32F3`		|
| STM32F4xx			| `#define BOARD_STM32F4`		|
| Arduino Due		| `#define BOARD_ARDUINO_DUE`	|