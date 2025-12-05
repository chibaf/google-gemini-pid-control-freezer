import time
import serial
import RPi.GPIO as GPIO
from simple_pid import PID

# --- Configuration ---
SERIAL_PORT = '/dev/ttyUSB0'  # Check this name (may be ttyUSB0 or similar)
BAUDRATE = 115200               # Match the baudrate of your serial device
GPIO_PIN = 18                 # The GPIO pin (BCM mode) connected to the freezer relay
TARGET_TEMP = -20.0           # Target temperature in Celsius
CYCLE_DURATION = 1800         # Total cycle duration in seconds (30 minutes)
CONTROL_DURATION = 1500       # Control phase duration in seconds (25 minutes)
OFF_DURATION = 300            # Off phase duration in seconds (5 minutes)

# PID parameters (these will likely need tuning for your specific freezer)
P = 10.0
I = 0.5
D = 2.0
OUTPUT_LIMIT = (0, 100) # Output as percentage of the cycle duration

# --- Setup ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2) # Wait for the serial port to initialize
    ser.reset_input_buffer()
except serial.SerialException as e:
    print(f"Error opening serial port: {e}. Check SERIAL_PORT configuration.")
    exit()

GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_PIN, GPIO.OUT)
GPIO.output(GPIO_PIN, GPIO.LOW) # Ensure freezer is off initially

pid = PID(P, I, D, setpoint=TARGET_TEMP)
pid.output_limits = OUTPUT_LIMIT
pid.sample_time = None # We will manage the sample time manually

def read_temperature_from_serial():
    """Reads the latest temperature value from the USB serial port."""
    try:
        # Clear buffer and read latest line
        ser.reset_input_buffer()
        line = ser.readline().strip().decode('utf-8')
        if line:
            # Assuming the serial output is a simple temperature string (e.g., "Temp: -18.5")
            # You may need to adjust the parsing logic based on your sensor's exact output format
            temp_str = line.split(",")
            temperature = float(temp_str[3])
            return temperature
    except Exception as e:
        print(f"Error reading or parsing serial data: {e}")
    return None

def control_freezer(output_value):
    """Controls the freezer (relay) based on the PID output using on/off control."""
    # The output value from PID is a percentage (0 to 100)
    # Use this to calculate the on-time for a short cycle (e.g., 60 seconds cycle within the main loop)
    # A simple on/off control within a sub-cycle is robust for freezers
    
    SUB_CYCLE_TIME = 60 # seconds
    on_time = (output_value / 100.0) * SUB_CYCLE_TIME
    off_time = SUB_CYCLE_TIME - on_time

    if on_time > 0:
        GPIO.output(GPIO_PIN, GPIO.HIGH) # Turn freezer ON
        time.sleep(on_time)
    
    if off_time > 0:
        GPIO.output(GPIO_PIN, GPIO.LOW) # Turn freezer OFF
        time.sleep(off_time)
    
    # Note: the actual loop sleep time in main() needs to accommodate this sub_cycle

# --- Main loop ---
try:
    while True:
        print(f"--- New {CYCLE_DURATION}s cycle started at {time.strftime('%Y-%m-%d %H:%M:%S')} ---")
        cycle_start_time = time.time()
        
        while time.time() - cycle_start_time <= CONTROL_DURATION:
            # Phase 1: 0 <= time <= 1500 (Control temperature)
            current_temp = read_temperature_from_serial()
            if current_temp is not None:
                pid_output = pid(current_temp)
                print(f"Time: {int(time.time() - cycle_start_time)}s | Temp: {current_temp}°C | PID Output: {pid_output:.2f}")
                
                # Control the freezer for a short interval
                control_freezer(pid_output) 
                
            # The control_freezer function already includes a sleep of SUB_CYCLE_TIME
            # so the main loop iteration time is consistent.

        # Phase 2: 1500 < time <= 1800 (Switch off freezer)
        print(f"--- Entering mandatory OFF period ({OFF_DURATION}s) ---")
        GPIO.output(GPIO_PIN, GPIO.LOW) # Ensure freezer is off
        time.sleep(OFF_DURATION)
        
        print("--- Cycle complete, starting next cycle ---")

except KeyboardInterrupt:
    print("Program terminated by user.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")

finally:
    GPIO.cleanup() # Clean up GPIO settings on exit
    ser.close()
    print("Cleanup finished.")

