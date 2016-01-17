##Electronic Sweet Peas - WorkStation40 Library

This is a simple library for controlling the WorkStation40 module.

The WorkStation40 is a complex system boards that allows you to configure and use peripherals such as timers, A/D converters, D/A converters, GPIO and more. The hart of the module is a low power Cortex-M0 device from Atmel. This MCU together with a I2C based GPIO expander provides the functionality of the board. 

Communication between the controller module and the WorkStation board is done with I2C. The module currently supports I2C communication up to 400Kbit/s, but we're looking in to having it run at 1Mbit/s.

The I2C port expander used on the module is the popular PCA9554A, an 8bit port expander with programmable interrupt and selectable polarity. This device is supported by the library with mehtods which are similar to that of the Arduino standard libraries. Look at the examples for more information on how to use the API.

The microcontroller found on the board is a device called SAMD10. The SAMD10 is a low-end Cortex-M0 based micro controller with a bunch of features. The API between the ESP210 and the Workstation40 board is extremly simple. It consists of two methods. One for reading 32bit words from the module and one method for writing 32 bit data to the module.

Data is read and written directly into the memory map of the microcontroller, making it possible to access any register or memory location inside the SAMD10. This is a very powerfull interface, but with power comes responsibilities. You need to make sure that the register or memory location that you are accessing is a valid one.

The SAMD10 microcontroller also has a I2C bootloader that allows you to replace the firmware. This allows you to upload updates to the firmware when it is available. You can also completely replace our firmware with your own, making any sorts of advanced communication schemes or interfaces possible.
