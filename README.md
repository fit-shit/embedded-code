# Embedded Code for IMU Modules

This code base contains the embedded software for the IMU module's ATMEGA328P MCU running the Arduino Bootloader.

## Build/Run Instructions

Follow the steps below to load the embedded software in this repository onto your IMU module's MCU.

1. Download/clone this repository to your local system.
2. Open the .ino and .h files in the Arduino IDE ([download here](https://www.arduino.cc/en/Main/Software) if you dont already have it on your system)
3. Plug the USB cable into your machine (literally the most important step)
4. Set the comm port of your USB serial interface by clicking Tools > Port > { port_name } (will look like `/dev/cu.usbserial-AL00EQKE`).
5. Set the board type by clicking Tools > Board > Arduino Uno
6. Upload your code to the MCU by clicking the 'Upload' button (right arrow) on the top left of the IDE.

Once the Rx/Tx LEDs on the board stop flashing and only the green Tx LED remains on, you can open the serial monitor by clicking the `_//` button on the top right of the screen.
