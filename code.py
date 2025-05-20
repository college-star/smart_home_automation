# main.py

from machine import Pin
import time
import _thread
MODE_BUTTON_PIN = 0  # GP0

# LEDs
WHITE_LED1_PIN = 1   # GP1
WHITE_LED2_PIN = 2   # GP2
WHITE_LED3_PIN = 3   # GP3
WHITE_LED4_PIN = 4   # GP4
RED_LED_PIN = 5      # GP5 (for gas alert)
GREEN_LED_PIN = 6    # GP6 (for gas normal)

# Matrix Keypad
# Rows (Outputs)
KEYPAD_ROW_PINS = [10, 11, 12, 13] # GP10, GP11, GP12, GP13
# Columns (Inputs with Pull-ups)
KEYPAD_COL_PINS = [14, 15, 16]    # GP14, GP15, GP16

# OLED Display (I2C)
OLED_SDA_PIN = 20 # GP20 (Default I2C0 SDA)
OLED_SCL_PIN = 21 # GP21 (Default I2C0 SCL)
OLED_WIDTH = 128
OLED_HEIGHT = 64

# Buzzer
BUZZER_PIN = 7    # GP7

# Stepper Motor (connected to ULN2003 driver IN1, IN2, IN3, IN4)
STEPPER_IN1_PIN = 17 # GP17
STEPPER_IN2_PIN = 18 # GP18
STEPPER_IN3_PIN = 19 # GP19
STEPPER_IN4_PIN = 22 # GP22

# MQ-2 Sensor
MQ2_ANALOG_PIN = 26  # ADC0 / GP26

SSD1306 OLED Driver ---
# You MUST have ssd1306.py file on your Pico for this to work.
# Example: from ssd1306 import SSD1306_I2C
# For now, I'll create a dummy class so the rest of the code can be structured.
# Replace this with the actual import and instantiation.
# ---
try:
    from ssd1306 import SSD1306_I2C
except ImportError:
    print("WARNING: SSD1306 library not found. OLED functions will be dummies.")
    class SSD1306_I2C: # Dummy class
        def __init__(self, width, height, i2c, addr=0x3C):
            self.width = width
            self.height = height
            self.i2c = i2c
            self.addr = addr
            print(f"Dummy OLED initialized: {width}x{height} on I2C, addr {addr}")
        def poweron(self): pass
        def poweroff(self): pass
        def contrast(self, c): pass
        def invert(self, i): pass
        def fill(self, c): print(f"OLED: fill({c})")
        def pixel(self, x, y, c): pass
        def text(self, s, x, y, c=1): print(f"OLED: text('{s}', {x}, {y})")
        def show(self): print("OLED: show()")
        def scroll(self, dx, dy): pass


GAS_ALERT_THRESHOLD = 180 # Example value, calibrate this!


# --- Global State Variables ---
current_program_mode = 1  # 1: Smart Door, 2: Smart Light, 3: Gas Detection
last_mode_button_press = 0
MODE_BUTTON_DEBOUNCE_MS = 200

# Smart Door State
CORRECT_PIN = "1234" # Change this!
entered_pin = ""
incorrect_pin_attempts = 0
door_is_open = False
secure_mode_enabled = False
alarm_active = False
last_key_press_time = 0
KEY_DEBOUNCE_MS = 200

# Smart Light State
led_states = [False, False, False, False] # For 4 white LEDs

# --- Hardware Initialization ---
# Mode Button
mode_button = machine.Pin(MODE_BUTTON_PIN, machine.Pin.IN, machine.Pin.PULL_UP)

# LEDs
white_leds = [
    machine.Pin(WHITE_LED1_PIN, machine.Pin.OUT),
    machine.Pin(WHITE_LED2_PIN, machine.Pin.OUT),
    machine.Pin(WHITE_LED3_PIN, machine.Pin.OUT),
    machine.Pin(WHITE_LED4_PIN, machine.Pin.OUT)
]
red_led = machine.Pin(RED_LED_PIN, machine.Pin.OUT)
green_led = machine.Pin(GREEN_LED_PIN, machine.Pin.OUT)

# Buzzer
buzzer = machine.Pin(BUZZER_PIN, machine.Pin.OUT)

# MQ-2 Sensor
mq2_adc = machine.ADC(machine.Pin(MQ2_ANALOG_PIN))

# Stepper Motor Pins (connected to ULN2003 driver)
stepper_pins = [
    machine.Pin(STEPPER_IN1_PIN, machine.Pin.OUT),
    machine.Pin(STEPPER_IN2_PIN, machine.Pin.OUT),
    machine.Pin(STEPPER_IN3_PIN, machine.Pin.OUT),
    machine.Pin(STEPPER_IN4_PIN, machine.Pin.OUT)
]
# Half-step sequence for 28BYJ-48 (8 steps)
stepper_sequence = [
    [1,0,0,0], [1,1,0,0], [0,1,0,0], [0,1,1,0],
    [0,0,1,0], [0,0,1,1], [0,0,0,1], [1,0,0,1]
]
# For 28BYJ-48, it's ~4096 steps for 360 degrees in half-step mode
# So, 180 degrees is ~2048 steps.
STEPS_FOR_180_DEG = 2048 # Adjust if your motor or gearing is different
STEP_DELAY_MS = 2 # Adjust for motor speed/torque

# Keypad
keypad_rows = [machine.Pin(pin, machine.Pin.OUT) for pin in KEYPAD_ROW_PINS]
keypad_cols = [machine.Pin(pin, machine.Pin.IN, machine.Pin.PULL_UP) for pin in KEYPAD_COL_PINS]
keypad_map = [
    ['1', '2', '3'],
    ['4', '5', '6'],
    ['7', '8', '9'],
    ['*', '0', '#']
]

# OLED Display
try:
    i2c = machine.I2C(0, scl=machine.Pin(OLED_SCL_PIN), sda=machine.Pin(OLED_SDA_PIN), freq=400000)
    oled = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c)
    oled.poweron()
    oled.contrast(255)
    oled_available = True
except Exception as e:
    print(f"Error initializing OLED: {e}")
    oled = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, None) # Dummy if error
    oled_available = False


# --- Helper Functions ---
def oled_display(lines, title=""):
    if not oled_available: return
    oled.fill(0)
    if title:
        oled.text(title, (OLED_WIDTH - len(title) * 8) // 2, 0) # Centered title
        y_offset = 10
    else:
        y_offset = 0

    for i, line in enumerate(lines):
        if isinstance(line, tuple): # (text, x_offset)
            oled.text(line[0], line[1], y_offset + i * 10)
        else:
            oled.text(line, 0, y_offset + i * 10)
    oled.show()

def beep(duration_ms=50):
    buzzer.on()
    time.sleep_ms(duration_ms)
    buzzer.off()

def short_beep():
    _thread.start_new_thread(beep, (30,)) # Non-blocking short beep for keypad

def alarm_on():
    global alarm_active
    alarm_active = True
    print("ALARM ACTIVATED")
    # This will block if not threaded. For a simple alarm, it's okay.
    # For continuous alarm while checking keys, threading or asyncio is needed.
    # _thread.start_new_thread(continuous_alarm_thread, ())

def alarm_off():
    global alarm_active
    alarm_active = False
    buzzer.off() # Ensure buzzer is off
    red_led.off() # Or whatever visual indication
    print("ALARM DEACTIVATED")

def continuous_alarm_thread():
    # This would be a separate thread for a non-blocking alarm
    while alarm_active:
        buzzer.on()
        red_led.toggle()
        time.sleep_ms(200)
        buzzer.off()
        time.sleep_ms(200)
    buzzer.off()
    red_led.off()

def read_keypad():
    global last_key_press_time
    key = None
    current_time = time.ticks_ms()

    if time.ticks_diff(current_time, last_key_press_time) < KEY_DEBOUNCE_MS:
        return None # Debounce

    for r_idx, row_pin in enumerate(keypad_rows):
        row_pin.low()
        for c_idx, col_pin in enumerate(keypad_cols):
            if col_pin.value() == 0: # Key pressed
                time.sleep_ms(20) # Basic debounce part 1
                if col_pin.value() == 0: # Confirm press
                    key = keypad_map[r_idx][c_idx]
                    last_key_press_time = current_time
                    # Wait for key release (optional, but good practice)
                    # while col_pin.value() == 0:
                    #     pass
                    row_pin.high() # Reset row
                    return key
        row_pin.high() # Reset row
    return None

def step_motor(steps, direction_clockwise=True, delay_ms=STEP_DELAY_MS):
    """ Rotates the stepper motor.
        Steps: number of steps to rotate.
        direction_clockwise: True for clockwise, False for anti-clockwise.
    """
    num_sequences = len(stepper_sequence)
    for _ in range(steps):
        for step_idx in range(num_sequences):
            actual_step_idx = step_idx if direction_clockwise else (num_sequences - 1 - step_idx)
            current_step_config = stepper_sequence[actual_step_idx]
            for pin_idx, pin_val in enumerate(current_step_config):
                stepper_pins[pin_idx].value(pin_val)
            time.sleep_ms(delay_ms)
    # Power off coils to save power and prevent overheating (optional)
    for pin in stepper_pins:
        pin.value(0)

def open_door():
    global door_is_open
    if not door_is_open:
        oled_display(["Door Opening..."], title="Smart Door")
        step_motor(STEPS_FOR_180_DEG, direction_clockwise=True)
        door_is_open = True
        oled_display(["Door Open", "Wait 10s..."], title="Smart Door")
        time.sleep(10)
        close_door()

def close_door():
    global door_is_open
    if door_is_open:
        oled_display(["Door Closing..."], title="Smart Door")
        step_motor(STEPS_FOR_180_DEG, direction_clockwise=False)
        door_is_open = False
        oled_display(["Door Closed"], title="Smart Door")
        time.sleep(1) # Brief pause
        # Reset to PIN entry for next use
        global entered_pin, incorrect_pin_attempts
        entered_pin = ""
        # incorrect_pin_attempts = 0 # Don't reset attempts here, only on correct PIN
        oled_display(["Enter PIN:", ""], title="Smart Door")


# --- Program 1: Smart Door ---
def smart_door_init():
    global entered_pin, incorrect_pin_attempts, secure_mode_enabled, alarm_active
    entered_pin = ""
    # incorrect_pin_attempts = 0 # Reset only if desired on mode switch
    secure_mode_enabled = False # Reset secure mode on re-entering this program
    alarm_off() # Ensure alarm is off when starting
    oled_display(["Enter PIN:", ""], title="Smart Door")
    # Ensure motor is not powered if door is meant to be closed initially
    for pin in stepper_pins: pin.value(0)


def run_smart_door():
    global entered_pin, incorrect_pin_attempts, secure_mode_enabled, door_is_open, alarm_active

    if alarm_active:
        # Simple alarm: buzzer on, red LED on. Any key press to turn off.
        buzzer.on()
        red_led.on()
        key = read_keypad()
        if key:
            short_beep() # Beep for key press during alarm
            alarm_off()
            incorrect_pin_attempts = 0 # Reset attempts after alarm acknowledged
            entered_pin = ""
            oled_display(["Enter PIN:", ""], title="Smart Door")
        return # Skip normal operation while alarm is on UI

    key = read_keypad()
    if key:
        short_beep() # Sound for every key press

        if len(entered_pin) < 8: # Max PIN length
            if key.isdigit():
                entered_pin += key
                oled_display(["Enter PIN:", "*" * len(entered_pin)], title="Smart Door")
            elif key == '*' and not secure_mode_enabled and door_is_open == False: # Enter PIN or manual motor control
                if entered_pin == CORRECT_PIN:
                    oled_display(["PIN Correct!"], title="Smart Door")
                    incorrect_pin_attempts = 0
                    secure_mode_enabled = False # Reset secure mode for this successful entry
                    time.sleep(1)
                    open_door() # This will open, wait, then close
                elif entered_pin != "": # If '*' is pressed and PIN field is not empty, treat as PIN entry attempt
                    incorrect_pin_attempts += 1
                    entered_pin = ""
                    oled_display(["Incorrect PIN", f"Attempts: {incorrect_pin_attempts}"], title="Smart Door")
                    time.sleep(1)
                    if incorrect_pin_attempts >= 3:
                        oled_display(["ALARM!", "Too many attempts"], title="Smart Door")
                        alarm_on() # Activate alarm
                        # Alarm will be handled at the start of this function in next loop
                    else:
                        oled_display(["Enter PIN:", ""], title="Smart Door")
                elif entered_pin == "" and not secure_mode_enabled: # Manual motor control with '*'
                    if not door_is_open:
                        oled_display(["Manual Open"], title="Smart Door")
                        step_motor(STEPS_FOR_180_DEG, direction_clockwise=True)
                        door_is_open = True
                        oled_display(["Door Open Manually"], title="Smart Door")
                    else:
                        oled_display(["Manual Close"], title="Smart Door")
                        step_motor(STEPS_FOR_180_DEG, direction_clockwise=False)
                        door_is_open = False
                        oled_display(["Door Closed Manually", "Enter PIN:", ""], title="Smart Door")
                    # After manual operation, reset to PIN entry
                    entered_pin = ""


            elif key == '#': # Toggle secure mode or clear PIN
                if entered_pin == CORRECT_PIN or secure_mode_enabled: # Enable/disable secure mode only if PIN was correct or already in secure mode
                    secure_mode_enabled = not secure_mode_enabled
                    if secure_mode_enabled:
                        oled_display(["Secure Mode: ON", "Motor Disabled"], title="Smart Door")
                    else:
                        oled_display(["Secure Mode: OFF", "PIN to open"], title="Smart Door")
                    # After toggling secure mode, expect PIN entry again or manual motor control
                    entered_pin = ""
                    time.sleep(1)
                    oled_display(["Enter PIN:", ""], title="Smart Door")

                else: # If '#' is pressed during PIN entry (and not for secure mode toggle)
                    entered_pin = "" # Clear current PIN entry
                    oled_display(["Enter PIN:", ""], title="Smart Door")


# --- Program 2: Smart Light ---
def smart_light_init():
    global led_states
    # Turn off all LEDs initially when entering this mode
    for i in range(len(white_leds)):
        white_leds[i].off()
        led_states[i] = False
    update_smart_light_display()

def update_smart_light_display():
    lines = []
    for i in range(len(led_states)):
        status = "ON" if led_states[i] else "OFF"
        lines.append(f"Light {i+1}: {status}")
    oled_display(lines, title="Smart Light")

def run_smart_light():
    global led_states
    key = read_keypad()

    if key:
        short_beep()
        if '1' <= key <= '4':
            led_index = int(key) - 1
            led_states[led_index] = not led_states[led_index]
            white_leds[led_index].value(led_states[led_index])
        elif key == '5':
            # If any LED is on, turn all off. Otherwise, turn all on.
            all_on = all(led_states)
            new_state = not all_on
            for i in range(len(led_states)):
                led_states[i] = new_state
                white_leds[i].value(new_state)
        
        update_smart_light_display()

# --- Program 3: Gas Detection ---
def gas_detection_init():
    global alarm_active
    alarm_off() # Ensure door alarm is off if it was active
    red_led.off()
    green_led.on() # Default to normal
    buzzer.off()
    update_gas_detection_display("Initializing...", "Normal")


def update_gas_detection_display(air_value_text, status_text):
    oled_display([f"Air Value: {air_value_text}", status_text], title="Gas Detection")

def run_gas_detection():
    air_value = mq2_adc.read_u16() # Reads 0-65535
    
    # Simple mapping for display, you might want to calibrate this.
    # This is just a raw ADC value. MQ-2 needs calibration for PPM.
    display_value = air_value // 100 # Scale down for display simplicity

    if air_value < GAS_ALERT_THRESHOLD: # Calibrate this threshold
        oled_display([f"Air Value: {display_value}", "ALERT! Gas Detected"], title="Gas Detection")
        red_led.value(1)
        green_led.value(0)
        buzzer.value(1) # Continuous beep for gas alert
    else:
        oled_display([f"Air Value: {display_value}", "Air Normal"], title="Gas Detection")
        red_led.value(0)
        green_led.value(1)
        buzzer.value(0)
    
    time.sleep_ms(100) # Read sensor periodically


# --- Main Program Loop ---
def main():
    global current_program_mode, last_mode_button_press

    # Initialize first mode
    if current_program_mode == 1:
        smart_door_init()
    elif current_program_mode == 2:
        smart_light_init()
    else: # Mode 3
        gas_detection_init()

    while True:
        current_time = time.ticks_ms()
        # Mode switching
        if not mode_button.value(): # Button pressed (pull-up, so low is pressed)
            if time.ticks_diff(current_time, last_mode_button_press) > MODE_BUTTON_DEBOUNCE_MS:
                last_mode_button_press = current_time
                current_program_mode += 1
                if current_program_mode > 3:
                    current_program_mode = 1
                
                print(f"Switching to mode: {current_program_mode}")
                # Reset/Initialize the new mode
                if current_program_mode == 1:
                    smart_door_init()
                elif current_program_mode == 2:
                    smart_light_init()
                else: # Mode 3
                    gas_detection_init()
                
                # Wait for button release to prevent multiple switches
                while not mode_button.value():
                    time.sleep_ms(10)
        
        # Run current program's logic
        if current_program_mode == 1:
            run_smart_door()
        elif current_program_mode == 2:
            run_smart_light()
        elif current_program_mode == 3:
            run_gas_detection()
        
        time.sleep_ms(10) # Small delay to prevent busy-looping too fast

# --- Entry Point ---
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program stopped by user.")
    finally:
        # Cleanup (turn off LEDs, buzzer, motor, etc.)
        if oled_available:
            oled.fill(0)
            oled.text("System Offline", 0,0)
            oled.show()
            oled.poweroff()
        for led_pin in white_leds: led_pin.off()
        red_led.off()
        green_led.off()
        buzzer.off()
        for motor_pin in stepper_pins: motor_pin.value(0)
        print("Cleanup complete. Exiting.")