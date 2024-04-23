# Donkey Car Driver for 2040-based boards such as the Raspberry Pi Pico and KB2040
#
# Notes:
#   This is to be run using CircuitPython 9.x
#   Last Updated: 4/08/2024

import time
import board
import busio
import rotaryio
import neopixel
from pulseio import PulseIn
from digitalio import DigitalInOut, Direction
from pwmio import PWMOut

# Customisation these variables
DEBUG = False
USB_SERIAL = False
ENCODER = False
SMOOTHING_INTERVAL_IN_S = 0.025
ACCEL_RATE = 10


pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)


## cannot have DEBUG and USB_SERIAL
if USB_SERIAL:
    DEBUG = False

## functions


def servo_duty_cycle(pulse_ms, frequency = 60):
    """
    Formula for working out the servo duty_cycle at 16 bit input
    """
    period_ms = 1.0 / frequency * 1000.0
    duty_cycle = int(pulse_ms / 1000 / (period_ms / 65535.0))
    return duty_cycle


def state_changed(control):
    """
    Reads the RC channel and smooths value
    """
    control.channel.pause()
    for i in range(0, len(control.channel)):
        val = control.channel[i]
        # prevent ranges outside of control space
        if(val < 1000 or val > 2000):
            continue
        # set new value
        control.value = (control.value + val) / 2

    control.channel.clear()
    control.channel.resume()


class Control:
    """
    Class for a RC Control Channel
    """

    def __init__(self, name, servo, channel, value):
        self.name = name
        self.servo = servo
        self.channel = channel
        self.value = value
        self.servo.duty_cycle = servo_duty_cycle(value)


# set up serial UART to Raspberry Pi
# note UART(TX, RX, baudrate)
uart = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0.001)

# set up servos
steering_pwm = PWMOut(board.D8, duty_cycle=2 ** 15, frequency=60)
throttle_pwm = PWMOut(board.D9, duty_cycle=2 ** 15, frequency=60)
mode_pwm = PWMOut(board.D10, duty_cycle=2 ** 15, frequency=60)

# set up RC channels.  NOTE: input channels are RCC3 & RCC4 (not RCC1 & RCC2)
steering_channel = PulseIn(board.D2, maxlen=64, idle_state=0)
throttle_channel = PulseIn(board.D3, maxlen=64, idle_state=0)
mode_channel = PulseIn(board.D4, maxlen=64, idle_state=0)


# setup Control objects.  1500 pulse is off and center steering
steering = Control("Steering", steering_pwm, steering_channel, 1500)
throttle = Control("Throttle", throttle_pwm, throttle_channel, 1500)
mode = Control("Mode", mode_pwm, mode_channel, 1500)

last_update = time.monotonic()

# GOTO: main()
def main():
    global last_update
    last_toggle_time = time.monotonic()
    interval = 1  # Seconds
    data = bytearray()
    datastr = ''
    last_input = 0
    steering_val = steering.value
    throttle_val = throttle.value
    led_state = False
    color=(0, 0, 255)

    while True:
        current_time = time.monotonic()
        if current_time - last_toggle_time >= interval:
            if led_state:
                pixel.fill((0, 0, 0))  # Turn off the NeoPixel
            else:
                pixel.fill(color)  # Set the NeoPixel to the specified color
            pixel.show()
            led_state = not led_state
            last_toggle_time = current_time
        # only update every smoothing interval (to avoid jumping)
        if(last_update + SMOOTHING_INTERVAL_IN_S > current_time):
            continue
        last_update = time.monotonic()

        # check for new RC values (channel will contain data)
        if(len(throttle.channel) != 0):
            state_changed(throttle)

        if(len(steering.channel) != 0):
            state_changed(steering)

        if(len(mode.channel) != 0):
            state_changed(mode)

        if(USB_SERIAL):
            # simulator USB
            print("%i, %i" % (int(steering.value), int(throttle.value)))
        else:
            # write the RC values to the RPi Serial
            uart.write(b"%i, %i\r\n" % (int(steering.value), int(throttle.value)))
            print(int(steering.value), int(throttle.value), int(mode.value))

        while True:
            # wait for data on the serial port and read 1 byte
            byte = uart.read(1)

            # if no data, break and continue with RC control
            if(byte is None):
                break
            last_input = time.monotonic()

            # if data is recieved, check if it is the end of a stream
            if(byte == b'\r'):
                data = bytearray()
                break

            data[len(data):len(data)] = byte

            # convert bytearray to string
            datastr = ''.join([chr(c) for c in data]).strip()

        # if we make it here, there is serial data from the previous step
        if(len(datastr) >= 10):
            steering_val = steering.value
            throttle_val = throttle.value
            try:
                steering_val = int(datastr[:4])
                throttle_val = int(datastr[-4:])
            except ValueError:
                None

            data = bytearray()
            datastr = ''
            last_input = time.monotonic()
            print("Set: steering=%i, throttle=%i" % (steering_val, throttle_val))

        if(last_input + 10 < time.monotonic()):
            # set the servo for RC control
            steering.servo.duty_cycle = servo_duty_cycle(steering.value)
            throttle.servo.duty_cycle = servo_duty_cycle(throttle.value)
        else:
            # set the servo for serial data (recieved)
            steering.servo.duty_cycle = servo_duty_cycle(steering_val)
            throttle.servo.duty_cycle = servo_duty_cycle(throttle_val)


# Run
print("Run!")
main()

