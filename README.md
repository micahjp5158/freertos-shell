
# freertos-shell
 A FreeRTOS-based application for STM32 microcontrollers that exposes a shell interface over UART to allow users to interact with hardware peripherals through text-based commands in a serial terminal.
### Hardware Requirements
- An STM32 development board. This project as-is targets the [STM32F4DISCOVERY](https://www.st.com/en/evaluation-tools/stm32f4discovery.html) board, but can be adapted for other hardware configurations as well.
- A [USB-UART adapter](https://www.adafruit.com/product/954), used to interface to the MCU from a host PC.

### Software Requirements
- Install the [Arm GNU Toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain). It is recommended to install these by downloading the compressed file from Arm directly instead of using your package manager, as that version may not include all executables including `arm-none-eabi-gdb`.
- Install [OpenOCD](https://openocd.org/):
	```
	sudo apt install openocd
	```
- Install a terminal emulator that supports serial connections, such as [PuTTY](https://www.putty.org/)

Debugging configurations will be included for [Visual Studio Code](https://code.visualstudio.com/) using the [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) extension.

### Compiling and Running
Bring in the project submodules:
```
git submodule update --init --recursive
```
Then, from the project's root folder:
-   `make` to compile the project
-   `make clean` to clean the project
-   `make flash` to download the project to your development board

### Usage
1. Connect the USB-UART adapter to the STM32DISCOVERY board
		- USB-UART GND (black wire) <-> STM32 GND
		-  USB-UART TX (green wire) <-> STM32 RX (PA3)
		- USB-UART RX (white wire) <-> STM32 TX (PA2)
2. Configure and open a serial connection in a terminal emulator with configuration:
		- COM port of the USB-UART adapter
		- 115200 baud rate
		- 8 data bits
		- 1 stop bit
		- No parity bit
		- No flow control
3. Start the FreeRTOS-Shell application by running `make flash`
4. Follow prompts and enter commands in the terminal window

### Debugging
This project also includes a debug configuration file for use in Visual Studio Code using the Cortex-Debug extension. To use this debug configuration:
1.  Install [Visual Studio Code](https://code.visualstudio.com/).
2.  Install the [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug) extension.
3.  Install `arm-none-eabi-gdb` dependencies:
    ```
    sudo apt install libncursesw5
    ```
4. Install Python 3.8
	```
	sudo apt install software-properties-common
	sudo add-apt-repository ppa:deadsnakes/ppa
	sudo apt install python3.8
	```
-   Open the subfolder containing the project you wish to debug in Visual Studio code.
-   Open main.c and press F5 to begin debugging.
