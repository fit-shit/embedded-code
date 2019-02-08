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


## Adding Required Libraries

If you have not installed the provided libraries (see 'Required Libraries' folder in repository), or have tried running the code on Arduino and noticed an error pretaining to a missing library, follow the followin steps.

1. Download/clone this repository to your local system. Unzip the folder if you downloaded the .zip.
2. Open the 'Required Libraries' folder.
3. Copy the library folders included within 'Required Libraries'.
4. Paste the folders under Documents > Arduino > Libraries.
5. Restart Arduino.
