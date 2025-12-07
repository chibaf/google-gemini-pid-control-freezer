import serial
import time
import RPi.GPIO as GPIO
from simple_pid import PID  # You will need to install this library

# --- Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
TARGET_TEMP = -20.0
GPIO_PIN = 18
OPERATION_CYCLE_SEC = 1800
FREEZER_ON_DURATION_SEC = 1500
FREEZER_OFF_DURATION_SEC = 300 # 1800 - 1500
# PID Constants (These will need tuning for your specific freezer)
KP = 5.0
KI = 0.1
KD = 0.5
# Output limits for PID (0=off, 100=on duty cycle percentage)
PID_OUTPUT_LIMITS = (0, 100)

# --- GPIO Setup ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_PIN, GPIO.OUT, initial=GPIO.LOW)

# --- Serial Setup ---
try:
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        timeout=1  # Read timeout
    )
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

# --- PID Controller Setup ---
pid = PID(KP, KI, KD, setpoint=TARGET_TEMP)
pid.output_limits = PID_OUTPUT_LIMITS
# The sample time for the PID controller is handled by the main loop sleep time.

def read_temperature(serial_port):
    """Reads a line from the serial port, parses it, and returns the temperature."""
    try:
        line = serial_port.readline().decode('utf-8').strip()
        if line:
            parts = line.split(',')
            if len(parts) >= 3:
                # The temperature value is the third element, multiplied by 10
                temp_x10_str = parts[2].strip()
                try:
                    temp_x10 = float(temp_x10_str)
                    return temp_x10 #/ 10.0
                except ValueError:
                    print(f"Bad temperature format in data: {temp_x10_str}")
        return None
    except Exception as e:
        print(f"Error reading/parsing serial data: {e}")
        return None

def control_freezer(pid_output):
    """Applies PID output (duty cycle percentage) within the operation cycle."""
    
    # Map the PID output to a duty cycle duration
    on_duration = (pid_output / 100.0) * FREEZER_ON_DURATION_SEC
    off_duration = FREEZER_ON_DURATION_SEC - on_duration
    
    # Operate the freezer for the calculated ON duration within the 1500 sec window
    if on_duration > 0:
        GPIO.output(GPIO_PIN, GPIO.HIGH) # Turn freezer ON
        time.sleep(on_duration)
        GPIO.output(GPIO_PIN, GPIO.LOW)  # Ensure it turns off

    # The remaining time of the 1500 sec window is OFF (already handled by time.sleep)
    # The 1500-1800 sec window is always OFF
    time.sleep(FREEZER_OFF_DURATION_SEC + off_duration)


try:
    print(f"Freezer control started. Target: {TARGET_TEMP}°C, GPIO: {GPIO_PIN}")
    while True:
        # 1. Read current temperature
        current_temp = read_temperature(ser)

        if current_temp is not None:
            print(f"Read temperature: {current_temp}°C")

            # 2. Calculate PID output
            # The PID library computes the control variable based on the input
            # In this case, output is a value between 0 (off) and 100 (full power)
            control_output_percent = pid(current_temp)
            print(f"PID output: {control_output_percent:.2f}%")

            # 3. Control freezer based on PID output and cycle constraints
            # This function blocks for the duration of the cycle (1800s)
            control_freezer(control_output_percent)

        else:
            print("Could not read valid temperature, waiting before next cycle attempt.")
            time.sleep(60) # Wait a bit if read fails

except KeyboardInterrupt:
    print("Program terminated by user.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    GPIO.cleanup()
    ser.close()
    print("GPIO pins reset and serial port closed.")
