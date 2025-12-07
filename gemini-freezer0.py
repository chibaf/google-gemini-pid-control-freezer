import serial
import time
import RPi.GPIO as GPIO
from simple_pid import PID
from datetime import datetime

# --- Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
GPIO_PIN = 18  # BCM pin number
TARGET_TEMP = -20.0  # Target temperature in Celsius
CYCLE_TIME = 1800  # Total cycle time in seconds (30 minutes)
MAX_ON_TIME = 1500 # Maximum ON time in a cycle (25 minutes)
MIN_OFF_TIME = 300 # Minimum OFF time in a cycle (5 minutes)

# --- GPIO Setup ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_PIN, GPIO.OUT, initial=GPIO.LOW)

# --- PID Controller Setup ---
# TUNE these values (Kp, Ki, Kd) for your specific freezer/sensor setup to prevent overshoot
# Generic starting points:
Kp = 2.0
Ki = 0.1
Kd = 0.5

pid = PID(Kp, Ki, Kd, setpoint=TARGET_TEMP)
# Set the output limits to be between 0 (off) and MAX_ON_TIME (max on time)
pid.output_limits = (0, MAX_ON_TIME)
pid.sample_time = None # We will manage the time manually

def read_temperature(ser):
    """Reads a line from serial, parses the temperature, and returns the float value."""
    try:
        line = ser.readline().decode('utf-8').strip()
        # Data format: "tag, date, temperature x10"
        parts = line.split(',')
        if len(parts) >= 3:
            temp_x10_str = parts[2].strip()
            # Extract only numeric characters if there are extra characters
            temp_val = float(''.join(filter(str.isdigit, temp_x10_str))) / 10.0
            return temp_val
    except Exception as e:
        print(f"Error reading or parsing serial data: {e}")
    return None

def control_freezer(duration_on):
    """Manages the ON/OFF cycle for the freezer."""
    # Ensure the duration_on is within the allowed window
    if duration_on > MAX_ON_TIME:
        duration_on = MAX_ON_TIME
    if duration_on < 0:
        duration_on = 0

    duration_off = CYCLE_TIME - duration_on

    print(f"Cycle control: ON for {duration_on:.2f}s, OFF for {duration_off:.2f}s")

    if duration_on > 0:
        GPIO.output(GPIO_PIN, GPIO.HIGH) # Turn freezer ON
        time.sleep(duration_on)
        GPIO.output(GPIO_PIN, GPIO.LOW)  # Turn freezer OFF

    if duration_off > 0:
        GPIO.output(GPIO_PIN, GPIO.LOW) # Ensure it is OFF
        time.sleep(duration_off)


# --- Main Loop ---
try:
    # Open serial connection
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) # Wait for serial to settle

    print(f"Freezer control started. Target Temp: {TARGET_TEMP}°C, Cycle Time: {CYCLE_TIME}s")

    while True:
        current_temp = read_temperature(ser)

        if current_temp is not None:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Current Temperature: {current_temp}°C")

            # Calculate PID output, which will be the "ON duration" for this cycle
            # The PID library update function calculates the output based on the error
            # We pass a 'dt' (delta time) equal to our cycle time for the I and D terms calculation.
            pid_output_on_time = pid(current_temp, dt=CYCLE_TIME)

            # Control the GPIO based on the calculated ON duration
            control_freezer(pid_output_on_time)
        else:
            print("Could not read valid temperature. Waiting a moment...")
            time.sleep(10) # Wait before trying to read again
            # In a real system, you might want more robust error handling here.

except KeyboardInterrupt:
    print("Program stopped by user")
except serial.SerialException as e:
    print(f"Serial communication error: {e}")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    GPIO.output(GPIO_PIN, GPIO.LOW) # Ensure freezer is off on exit
    GPIO.cleanup() # Clean up GPIO settings
    if 'ser' in locals() and ser.isOpen():
        ser.close()
