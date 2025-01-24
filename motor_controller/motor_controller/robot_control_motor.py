import RPi.GPIO as GPIO
import time

# Pin Definitions for Motor Control
# Rear Left Motor
MR_PWMB = 13
MR_BI1 = 5
MR_BI2 = 6

# Rear Right Motor
MR_PWMA = 16
MR_AI1 = 21
MR_AI2 = 20

# Front Left Motor
MF_PWMB = 12
MF_BI1 = 1
MF_BI2 = 7

# Front Right Motor
MF_PWMA = 17
MF_AI1 = 27
MF_AI2 = 22

# PWM Setup
PWM_FREQ = 1000  # Frequency in Hz

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup pins as output
motor_pins = [MF_PWMA, MF_AI1, MF_AI2, MF_PWMB, MF_BI1, MF_BI2, MR_PWMA, MR_AI1, MR_AI2, MR_PWMB, MR_BI1, MR_BI2]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

# Initialize PWM
pwm_channels = {
    "RF": GPIO.PWM(MF_PWMA, PWM_FREQ),
    "LF": GPIO.PWM(MF_PWMB, PWM_FREQ),
    "RR": GPIO.PWM(MR_PWMA, PWM_FREQ),
    "LR": GPIO.PWM(MR_PWMB, PWM_FREQ),
}

# Start PWM with 0 duty cycle
for pwm in pwm_channels.values():
    pwm.start(0)  # Start PWM with 0 duty cycle


# Define movement directions in binary
MEC_STRAIGHT_FORWARD = 0b10101010
MEC_STRAIGHT_BACKWARD = 0b01010101
MEC_SIDEWAYS_RIGHT = 0b01101001
MEC_SIDEWAYS_LEFT = 0b10010110
MEC_ROTATE_CLOCKWISE = 0b01100110
MEC_ROTATE_COUNTERCLOCKWISE = 0b10011001


# Motor Control functions
def move_motors(speedRF, speedLF, speedRR, speedLR, dircontrol):
    # Right Front Motor
    GPIO.output(MF_AI1, (dircontrol & 0b10000000) > 0)
    GPIO.output(MF_AI2, (dircontrol & 0b01000000) > 0)
    pwm_channels["RF"].ChangeDutyCycle(abs(speedRF))

    # Left Front Motor
    GPIO.output(MF_BI1, (dircontrol & 0b00100000) > 0)
    GPIO.output(MF_BI2, (dircontrol & 0b00010000) > 0)
    pwm_channels["LF"].ChangeDutyCycle(abs(speedLF))

    # Right Rear Motor
    GPIO.output(MR_AI1, (dircontrol & 0b00001000) > 0)
    GPIO.output(MR_AI2, (dircontrol & 0b00000100) > 0)
    pwm_channels["RR"].ChangeDutyCycle(abs(speedRR))

    # Left Rear Motor
    GPIO.output(MR_BI1, (dircontrol & 0b00000010) > 0)
    GPIO.output(MR_BI2, (dircontrol & 0b00000001) > 0)
    pwm_channels["LR"].ChangeDutyCycle(abs(speedLR))

    # print("Actuating motors")

def stop_motors():
    for pwm in pwm_channels.values():
        pwm.ChangeDutyCycle(0)
    for pin in motor_pins:
        GPIO.output(pin, 0)

RAMP_UP_START_PWM = 75
RAMP_UP_END_PWM = 85
RAMP_DOWN_START_PWM = 85
RAMP_DOWN_END_PWM = 70
RAMP_TIME_MOVE = 0.5 # Seconds
RAMP_TIME_STOP = 0.7

def ramped_move_motors(dircontrol, ramp_time=RAMP_TIME_MOVE):
    """
    Ramps the PWM duty cycle from 80 to 95 in 0.5 seconds.

    Args:
        dircontrol: Direction control byte.
        ramp_time: The duration of the ramp up in seconds (defaults to 0.5).
    """
    start_pwm = RAMP_UP_START_PWM
    end_pwm = RAMP_UP_END_PWM
    num_steps = 50  #Number of steps
    dt = ramp_time / num_steps

    for i in range(num_steps + 1):  # +1 to include the final value
        pwm_value = start_pwm + (end_pwm - start_pwm) * (i / num_steps)
        move_motors(pwm_value, pwm_value, pwm_value, pwm_value, dircontrol) # Set all PWMs
        time.sleep(dt)


def ramped_stop_motors(ramp_time=RAMP_TIME_STOP):
    """
    Ramps the PWM duty cycle from 90 to 70 in 0.5 seconds, then stops the motors.

    Args:
        ramp_time: The duration of the ramp down in seconds (defaults to 0.5).
    """
    start_pwm = RAMP_DOWN_START_PWM
    end_pwm = RAMP_DOWN_END_PWM
    num_steps = 50  #Number of steps
    dt = ramp_time / num_steps

    for i in range(num_steps + 1):  # +1 to include the final value
        pwm_value = start_pwm + (end_pwm - start_pwm) * (i / num_steps)
        move_motors(pwm_value, pwm_value, pwm_value, pwm_value, 0b00000000) # Set all PWMs for stop.

        time.sleep(dt)
    
    stop_motors() # Fully Stop

# Example usage in a ROS2 node (Conceptual)
# if __name__ == '__main__':  #Added to prevent auto running when imported
#     try:
#         # Example parameters
#         # dircontrol = MEC_STRAIGHT_FORWARD
#         # move_motors(65, 65, 65, 65, MEC_STRAIGHT_BACKWARD)
#         # time.sleep(2)
#         # stop_motors()
#         # time.sleep(7)

#         # print("Ramping up...")
#         # ramped_move_motors(dircontrol)  # Ramp up to move

#         # # time.sleep(2)  #Move at the sped for two seconds
#         # # stop_motors()

#         # print("Ramping down...")
#         # ramped_stop_motors()  # Ramp down to stop

#         # print("Stopped")

#     except KeyboardInterrupt:
#         print("Exiting...")
#     finally:
#         stop_motors()
#         GPIO.cleanup()


# Code to test Movement
# try:
#     while True:
#         print("Straight Forward")
#         move_motors(100, 100, 100, 100, MEC_STRAIGHT_FORWARD)
#         time.sleep(1)
#         stop_motors()
#         time.sleep(7)

# except KeyboardInterrupt:
#     print("Exiting...")
# finally:
#     stop_motors()
#     GPIO.cleanup()
