ARM PICKUP CODE
import pigpio
import time

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    exit("Could not connect to pigpio daemon")

# Servo GPIO pins
SERVO1 = 21  # GPIO pin 11
SERVO2 = 20  # GPIO pin 13
SERVO3 = 16  # GPIO pin 15

# Smooth servo movement
def move_servo(pin, from_us, to_us, step=10, delay=0.01):
    direction = 1 if to_us > from_us else -1
    for us in range(from_us, to_us + direction * step, direction * step):
        pi.set_servo_pulsewidth(pin, us)
        time.sleep(delay)
    pi.set_servo_pulsewidth(pin, to_us)

# Define the pickup sequence
def pickup():
    print("Starting pickup sequence...")

    # Initial positions
    pi.set_servo_pulsewidth(SERVO1, 1000)
    pi.set_servo_pulsewidth(SERVO2, 1760)
    pi.set_servo_pulsewidth(SERVO3, 1250)
    time.sleep(0.5)

    # Move servos to pickup
    move_servo(SERVO3, 1250, 630)  # Gripper close
    time.sleep(1)

    move_servo(SERVO1, 1000, 2400)  # Arm raise
    time.sleep(0.5)

    move_servo(SERVO2, 1760, 800)   # Rotate or lift
    time.sleep(0.5)

    print("Pickup complete.")

# Example call
