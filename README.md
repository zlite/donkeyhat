This is the repo for the new Donkeycar RC hat (aka Donkeyhat) for the Raspberry Pi. It has a Waveshare RP2040-zero microprocessor running CircuitPython 9.x and has the following functions:

* Read three channels of RC input from a standard RC receiver
* Control two servo channels (steering and throttle)
* Communicates with the RaspberryPi via hardware serial/UART
* Display data on an OLED screen (controlled by the Raspberry Pi via I2C)
* Reads two wheel/shaft encoders, and can handle both single channel and quadrature encoders
* Blinks a Neopixel LED to show normal operation


Instructructions:
* Copy the CircuitPython firmware to the RP2040 following these instructions: https://circuitpython.org/board/waveshare_rp2040_zero/
* Copy the required Neopixel library files to the "lib" directory of your RP2040 as instructed here: https://learn.adafruit.com/circuitpython-essentials/circuitpython-neopixel
* Copy the `code.py` file to the root for the RP2040 (CircuitPython always runs code.py at startup)
* Use the Mu Editor for modifying the code: https://codewith.mu/
