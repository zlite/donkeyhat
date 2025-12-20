# Donkey Car Driver for 2040-based boards
# Refactored for stability, non-blocking I/O, and LED heartbeat
import time
import board
import busio
import neopixel
from pulseio import PulseIn
from pwmio import PWMOut
import digitalio
import rotaryio

# Customisation variables
DEBUG = False
USB_SERIAL = False
SMOOTHING_INTERVAL_IN_S = 0.020 
USE_QUADRATURE = False 

# Pin assignments
RC1, RC2, RC3 = board.GP27, board.GP26, board.GP29
Steering, Throttle = board.GP11, board.GP10
Encoder1A_pin, Encoder1B_pin = board.GP8, board.GP9
Encoder2A_pin, Encoder2B_pin = board.GP13, board.GP14

# Hardware Setup
if USE_QUADRATURE:
    encoder1 = rotaryio.IncrementalEncoder(Encoder1A_pin, Encoder1B_pin)
    encoder2 = rotaryio.IncrementalEncoder(Encoder2A_pin, Encoder2B_pin)
else:
    encoder1 = digitalio.DigitalInOut(Encoder1A_pin)
    encoder1.direction = digitalio.Direction.INPUT
    encoder1.pull = digitalio.Pull.DOWN
    encoder2 = digitalio.DigitalInOut(Encoder2A_pin)
    encoder2.direction = digitalio.Direction.INPUT
    encoder2.pull = digitalio.Pull.DOWN

pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
if USB_SERIAL: DEBUG = False

def servo_duty_cycle(pulse_ms, frequency=50):
    period_ms = 1000.0 / frequency
    duty_cycle = int((pulse_ms / 1000.0) / (period_ms / 65535.0))
    return max(0, min(65535, duty_cycle))

def state_changed(control):
    if len(control.channel) > 0:
        val = control.channel[-1] 
        if 900 <= val <= 2100:
            control.value = (control.value * 0.7) + (val * 0.3)
            if 1480 < control.value < 1520:
                control.value = 1500
    control.channel.clear()

class Control:
    def __init__(self, name, servo, channel, value):
        self.name, self.servo, self.channel, self.value = name, servo, channel, value
        self.servo.duty_cycle = servo_duty_cycle(value)

# Communication and PWM Setup
uart = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0) 
steering_pwm = PWMOut(Steering, duty_cycle=0, frequency=50)
throttle_pwm = PWMOut(Throttle, duty_cycle=0, frequency=50)

steering_channel = PulseIn(RC1, maxlen=64, idle_state=0)
throttle_channel = PulseIn(RC2, maxlen=64, idle_state=0)

steering = Control("Steering", steering_pwm, steering_channel, 1500)
throttle = Control("Throttle", throttle_pwm, throttle_channel, 1500)

# Global States
last_update = time.monotonic()
continuous_mode = False
continuous_delay = 0
position1, position2 = 0, 0
datastr = ""

def main():
    global last_update, continuous_mode, continuous_delay, position1, position2, datastr
    last_led_toggle = time.monotonic()
    last_input = 0
    led_state = False
    
    if not USE_QUADRATURE:
        last_state1, last_state2 = encoder1.value, encoder2.value

    while True:
        current_time = time.monotonic()
        
        # --- Heartbeat LED (Blue flash every 0.5s) ---
        if current_time - last_led_toggle >= 0.5:
            led_state = not led_state
            pixel.fill((0, 0, 255) if led_state else (0, 0, 0))
            pixel.show()
            last_led_toggle = current_time

        # --- Encoder Logic ---
        if USE_QUADRATURE:
            position1, position2 = encoder1.position, encoder2.position
        else:
            curr1, curr2 = encoder1.value, encoder2.value
            if curr1 != last_state1 and not curr1: position1 += 1
            if curr2 != last_state2 and not curr2: position2 += 1
            last_state1, last_state2 = curr1, curr2

        # --- RC Smoothing & Serial Out ---
        if current_time - last_update >= SMOOTHING_INTERVAL_IN_S:
            if len(throttle.channel): state_changed(throttle)
            if len(steering_channel): state_changed(steering)
            
            if not USB_SERIAL:
                uart.write(f"{int(steering.value)}, {int(throttle.value)}\r\n".encode())
            last_update = current_time

        # --- Non-Blocking UART Read ---
        incoming = uart.read(32)
        if incoming:
            for b in incoming:
                char = chr(b)
                if char in ('\r', '\n'):
                    if datastr:
                        # Process steering/throttle command (e.g., "15001500")
                        if len(datastr) >= 8 and datastr[:1].isdigit():
                            try:
                                s_val = int(datastr[:4])
                                t_val = int(datastr[-4:])
                                steering.servo.duty_cycle = servo_duty_cycle(s_val)
                                throttle.servo.duty_cycle = servo_duty_cycle(t_val)
                                last_input = current_time
                            except ValueError: pass
                    datastr = ""
                else:
                    datastr += char

        # --- Control Handover ---
        if current_time > (last_input + 0.1):
            steering.servo.duty_cycle = servo_duty_cycle(steering.value)
            throttle.servo.duty_cycle = servo_duty_cycle(throttle.value)

print("Donkey Driver Ready!")
main()