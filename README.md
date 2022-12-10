# CAN-LIB
## General CAN library for Purdue Solar Racing
### Supported Microcontrollers
 - STM32F Series
 - Arduino Due
 - Arduino Uno (with MCP2515)
 - Arduino Mega (with MCP2515)

# Installation

## STM32 Cube IDE

 1. Download and unzip repository to your local drive.
 2. In your STM32 Cube project, right-click on the project and select `Import...`
 3. Select `File System` from the `General` Tab.
 4. Select the `can-lib` folder from your local filesystem.
 5. In the dropdown for the `can-lib` folder select the `src` and `inc` folders and hit Finish
 7. Right click the added `can-lib` folder in the project root, and select `Add/remove include path` and hit OK.
 8. Right click on the project and select `Properties`
 9. Navigate to `C/C++ General` and then `Paths and Symbols`
 10. Under `Source Location` click `Add Folder`
 11. Select the `can-lib/src` folder and hit Okay.
 12. Repeat Step 11 for each build configuration.
 13. Under the `Symbols` tab, click `Add` to add a new compilation symbol.
 14. Type `BOARD_STM32F` in the name field.
 15. In the value field put the number of the STM32F board that is being used `0` for F0, `1` for F1, etc.
 16. Check both the `Add to all configurations` and `Add to all languages` checkboxes and hit OK.
 17. Close the Properties menu.
 18. You can now include the header files and compile the program with the library.

## Arduino IDE
 1. Download and unzip repository to your local drive
 2. `cd` to repository root directory
 3. Run `./scripts/build_arduino_lib.sh <board_name>` where `<board_name>` is the name of the Arduino board (`arduino_due`, `arduino_mega`, or `arduino_uno`)
 4. The output zip file is in the `build` folder
 5. Install the zip file in the Arduino IDE

# Including library
## Microcontroller type must be defined before including any library headers.

Example:
```c
#define BOARD_STM32F 3 // Board Selection
// Or: #define BOARD_ARDUINO_UNO
// Or: #define BOARD_ARDUINO_MEGA
// Or: #define BOARD_ARDUINO_DUE
#include "can_lib.h"
```

## Pre-processor definitions are also required during compilation.

Example:
```
$ gcc <...> -DBOARD_STM32F=4 <...>
```

## Microcontroller define statements
| Microcontroller	| C/C++ Define Statement 		|
| ----------------- | ----------------------------- |
| STM32Fx			| `#define BOARD_STM32F x`		|
| Arduino Due		| `#define BOARD_ARDUINO_DUE`	|
| Arduino Uno		| `#define BOARD_ARDUINO_UNO`	|
| Arduino Mega		| `#define BOARD_ARDUINO_MEGA`	|