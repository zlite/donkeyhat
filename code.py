# Donkey Car Driver for 2040-based boards
# Updated: Added E-Stop on RC3 (GP29) and Visual Feedback

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
Steering_Pin, Throttle_Pin = board.GP11, board.GP10
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

def servo_duty_cycle(pulse_ms, frequency=60):
    period_ms = 1000.0 / frequency
    duty_cycle = int((pulse_ms / 1000.0) / (period_ms / 65535.0))
    return max(0, min(65535, duty_cycle))

def state_changed(control):
    if len(control.channel) > 0:
        val = control.channel[-1] 
        if 900 <= val <= 2100:
            # EMA Smoothing
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
steering_pwm = PWMOut(Steering_Pin, duty_cycle=0, frequency=60)
throttle_pwm = PWMOut(Throttle_Pin, duty_cycle=0, frequency=60)

steering_channel = PulseIn(RC1, maxlen=64, idle_state=0)
throttle_channel = PulseIn(RC2, maxlen=64, idle_state=0)
estop_channel = PulseIn(RC3, maxlen=64, idle_state=0) # E-Stop Input

steering = Control("Steering", steering_pwm, steering_channel, 1500)
throttle = Control("Throttle", throttle_pwm, throttle_channel, 1500)

# Global States
last_update = time.monotonic()
continuous_mode = False
continuous_delay = 0
position1, position2 = 0, 0
datastr = ""
rc_estop_val = 1000 # Default to "Off"

def handle_command(command):
    global position1, position2, continuous_mode, continuous_delay
    command = command.strip()
    
    if command == 'r':
        position1, position2 = 0, 0
        if USE_QUADRATURE:
            encoder1.position = 0
            encoder2.position = 0
        print("Positions reset")
        
    elif command == 'p':
        send_telemetry()
        
    elif command.startswith('c'):
        if len(command) > 1 and command[1:].isdigit():
            continuous_delay = int(command[1:])
            continuous_mode = True
        else:
            continuous_mode = not continuous_mode
        print(f"Continuous: {continuous_mode}")

def send_telemetry():
    now_ms = int(time.monotonic() * 1000)
    msg = f"{int(steering.value)}, {int(throttle.value)}, {position1}, {now_ms}; {position2}, {now_ms}\r\n"
    uart.write(msg.encode())

def main():
    global last_update, continuous_mode, continuous_delay, position1, position2, datastr, rc_estop_val
    last_led_toggle = time.monotonic()
    last_continuous_send = time.monotonic()
    last_input = 0
    led_state = False
    
    if not USE_QUADRATURE:
        last_state1, last_state2 = encoder1.value, encoder2.value

    while True:
        current_time = time.monotonic()
        
        # --- Read E-Stop Channel ---
        if len(estop_channel) > 0:
            rc_estop_val = estop_channel[-1]
            estop_channel.clear()

        # --- Heartbeat LED (Switches to RED if E-Stop is active) ---
        if current_time - last_led_toggle >= 0.5:
            led_state = not led_state
            if rc_estop_val > 1700:
                pixel.fill((255, 0, 0) if led_state else (0, 0, 0)) # Red Flash
            else:
                pixel.fill((0, 0, 255) if led_state else (0, 0, 0)) # Blue Flash
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

        # --- Continuous Telemetry Mode ---
        if continuous_mode and (current_time - last_continuous_send >= continuous_delay / 1000.0):
            send_telemetry()
            last_continuous_send = current_time

        # --- Non-Blocking UART Read & Command Handling ---
        incoming = uart.read(32)
        s_val_in, t_val_in = None, None
        if incoming:
            for b in incoming:
                char = chr(b)
                if char in ('\r', '\n'):
                    if datastr:
                        if len(datastr) >= 8 and datastr[0].isdigit():
                            try:
                                s_val_in = int(datastr[:4])
                                t_val_in = int(datastr[-4:])
                                last_input = current_time
                            except ValueError: pass
                        else:
                            handle_command(datastr)
                    datastr = ""
                else:
                    datastr += char

        # --- Control Handover & E-Stop Logic ---
        if rc_estop_val > 1700:
            # E-Stop Active: Overrides everything to neutral throttle
            steering.servo.duty_cycle = servo_duty_cycle(steering.value)
            throttle.servo.duty_cycle = servo_duty_cycle(1500)
        elif current_time < (last_input + 0.1) and t_val_in is not None:
            # Serial/Pi Control
            steering.servo.duty_cycle = servo_duty_cycle(s_val_in)
            throttle.servo.duty_cycle = servo_duty_cycle(t_val_in)
        else:
            # RC Control
            steering.servo.duty_cycle = servo_duty_cycle(steering.value)
            throttle.servo.duty_cycle = servo_duty_cycle(throttle.value)

print("Donkey Driver Ready with E-Stop (CH3)!")
main()