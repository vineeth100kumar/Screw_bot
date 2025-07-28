ROUTE TRAIN CODE
import RPi.GPIO as GPIO
import time
import json
import os
import threading
import sys
import termios
import tty
import select

# --- GLOBAL CONFIGURATION ---
# Motor Control Pins (BCM numbering)
motors = {
    1: {'pinA': 17, 'pinB': 18},
    2: {'pinA': 22, 'pinB': 23},
    3: {'pinA': 24, 'pinB': 25},
    4: {'pinA': 4, 'pinB': 27}
}

STEP_DURATION = 0.5
RECORDED_ROUTES_DIR = "recorded_routes"
recorded_path = []

# --- GPIO Setup ---
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for m in motors.values():
        GPIO.setup(m['pinA'], GPIO.OUT)
        GPIO.setup(m['pinB'], GPIO.OUT)
        GPIO.output(m['pinA'], GPIO.LOW)
        GPIO.output(m['pinB'], GPIO.LOW)
    print("GPIO setup complete. All motors initialized to OFF.")

# --- Motor Control Functions ---
def motor_forward(motor_num):
    if motor_num in motors:
        m = motors[motor_num]
        GPIO.output(m['pinA'], GPIO.HIGH)
        GPIO.output(m['pinB'], GPIO.LOW)

def motor_backward(motor_num):
    if motor_num in motors:
        m = motors[motor_num]
        GPIO.output(m['pinA'], GPIO.LOW)
        GPIO.output(m['pinB'], GPIO.HIGH)

def stop_motor(motor_num):
    if motor_num in motors:
        m = motors[motor_num]
        GPIO.output(m['pinA'], GPIO.LOW)
        GPIO.output(m['pinB'], GPIO.LOW)

def stop_all_motors():
    for motor_num in motors:
        stop_motor(motor_num)
    print("All motors stopped.")

def move_all_forward(duration=None):
    for motor_num in motors:
        motor_forward(motor_num)
    if duration:
        time.sleep(duration)
        stop_all_motors()

def move_all_backward(duration=None):
    for motor_num in motors:
        motor_backward(motor_num)
    if duration:
        time.sleep(duration)
        stop_all_motors()

def move_all_left(duration=None):
    motor_forward(1)
    motor_forward(3)
    motor_backward(2)
    motor_backward(4)
    if duration:
        time.sleep(duration)
        stop_all_motors()

def move_all_right(duration=None):
    motor_backward(1)
    motor_backward(3)
    motor_forward(2)
    motor_forward(4)
    if duration:
        time.sleep(duration)
        stop_all_motors()

# --- Non-Blocking Input ---
old_settings = None

def set_non_blocking_input():
    global old_settings
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

def restore_input_settings():
    if old_settings:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def get_char_nonblocking():
    return sys.stdin.read(1) if sys.stdin in select.select([sys.stdin], [], [], 0)[0] else None

# --- Recording Logic ---
def record_movement(action_type, duration_val):
    global recorded_path

    if action_type == "forward":
        move_all_forward(duration_val)
    elif action_type == "backward":
        move_all_backward(duration_val)
    elif action_type == "pivot_left":
        move_all_left(duration_val)
    elif action_type == "pivot_right":
        move_all_right(duration_val)
    elif action_type == "stop":
        stop_all_motors()
        duration_val = 0

    recorded_path.append({
        "action": action_type,
        "duration": duration_val,
        "timestamp": time.time()
    })
    print(f"Recorded: {action_type} for {duration_val}s.")

def save_recorded_route(label):
    safe_label = "".join(c for c in label if c.isalnum() or c in ('_', '-')).strip()
    if not safe_label:
        safe_label = "unlabeled_route_" + str(int(time.time()))
        print("Warning: Empty or invalid label. Saving as:", safe_label)

    os.makedirs(RECORDED_ROUTES_DIR, exist_ok=True)
    filepath = os.path.join(RECORDED_ROUTES_DIR, f"{safe_label}.json")

    with open(filepath, 'w') as f:
        json.dump(recorded_path, f, indent=4)

    print(f"\nRoute saved to: {filepath}")
    print(f"Label '{label}' should be encoded in the QR code for this route.")

def replicate_route(route_data):
    print("\n--- Starting Route Replication ---")
    if not route_data:
        print("No route data to replicate.")
        return

    for step_num, step in enumerate(route_data):
        action = step["action"]
        duration = step["duration"]

        print(f"Executing step {step_num + 1}: {action} for {duration}s...")
        if action == "forward":
            move_all_forward(duration)
        elif action == "backward":
            move_all_backward(duration)
        elif action == "pivot_left":
            move_all_left(duration)
        elif action == "pivot_right":
            move_all_right(duration)
        elif action == "stop":
            stop_all_motors()
        time.sleep(0.1)

    print("--- Route Replication Completed ---")
    stop_all_motors()

# --- Main Loop ---
def main_training_loop():
    print("--- Rover Route Training System ---")
    setup_gpio()

    print("\nControl Keys:")
    print("  'w': Forward")
    print("  's': Backward")
    print("  'a': Pivot Left")
    print("  'd': Pivot Right")
    print("  'x': Stop")
    print("  'q': Quit and Save")
    input("\nPress ENTER to start recording...")

    set_non_blocking_input()

    try:
        while True:
            char = get_char_nonblocking()
            if char:
                char = char.lower()
                if char == 'w':
                    record_movement("forward", STEP_DURATION)
                elif char == 's':
                    record_movement("backward", STEP_DURATION)
                elif char == 'a':
                    record_movement("pivot_left", STEP_DURATION)
                elif char == 'd':
                    record_movement("pivot_right", STEP_DURATION)
                elif char == 'x':
                    record_movement("stop", 0)
                elif char == 'q':
                    print("\n'q' pressed. Stopping recording...")
                    break
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nTraining interrupted by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        restore_input_settings()
        stop_all_motors()

        if recorded_path:
            label = input("\nEnter a label for this route: ")
            save_recorded_route(label)
            if input("Replicate route now? (y/n): ").lower() == 'y':
                replicate_route(recorded_path)
        else:
            print("No movements recorded.")

        GPIO.cleanup()
        print("GPIO cleaned up. Exiting.")

if _name_ == "_main_":
    main_training_loop()
