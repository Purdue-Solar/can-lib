# CAN-LIB
## General CAN library for Purdue Solar Racing
### Supported Microcontrollers
 - STM32F series (bxCAN)
 - STM32G series
 - STM32H series

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
 14. Type `STM32_PROCESSOR` in the name field.
 15. In the value field put the number of the STM32 board that is being used `f0` for F0, `f1` for F1, etc.
 16. Check both the `Add to all configurations` and `Add to all languages` checkboxes and hit OK.
 17. Close the Properties menu.
 18. You can now include the header files and compile the program with the library.

# Including library
## Microcontroller type must be defined before including any library headers.

Example:
```c
#define STM32_PROCESSOR f3 // Board Selection
#include "can_lib.hpp"
```

## Pre-processor definitions are also required during compilation.

Example:
```
$ gcc <...> -DSTM32_PROCESSOR=f4 <...>
```

## Microcontroller define statements
| Microcontroller	| C/C++ Define Statement 			|
| ----------------- | --------------------------------- |
| STM32Fx			| `#define STM32_PROCESSOR fx`		|
| STM32Gx			| `#define STM32_PROCESSOR gx`		|
| STM32Hx			| `#define STM32_PROCESSOR hx`		|
