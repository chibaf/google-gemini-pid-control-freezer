import serial
import time
import RPi.GPIO as GPIO
import struct

# --- Configuration ---
TARGET_TEMP = -20.0 # Target temperature in degrees Celsius
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
GPIO_PIN = 18 # BCM pin numbering
OPERATION_CYCLE = 1800 # Total cycle time in seconds
PID_RUN_TIME = 1500 # Time for PID control within the cycle
OFF_TIME = 300 # Time to switch off the freezer at end of cycle (1800-1500)
SAMPLE_TIME = 2.0 # PID sample time in seconds 2->5

# --- PID Controller Class (simple implementation) ---
class PID:
    def __init__(self, P=0.1, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.set_point = 0.0
        self.error = 0.0
        self.integral_error = 0.0
        self.derivative_error = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        self.sample_time = SAMPLE_TIME

    def update(self, feedback_value):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt >= self.sample_time:
            self.error = self.set_point - feedback_value
            self.integral_error += (self.error * dt)
            self.derivative_error = (self.error - self.last_error) / dt
            output = self.Kp * self.error + self.Ki * self.integral_error + self.Kd * self.derivative_error
            self.last_error = self.error
            self.last_time = current_time
            # Clamp output to a range appropriate for duty cycle (0 to 1)
            return max(0, min(1, output))
        return None # Return None if not enough time has passed

# --- Serial Data Reading Function ---
def read_temperature_from_serial(ser):
    try:
        # Read a line from the serial port
        line = ser.readline().strip()
        # Decode the bytes to string and split by comma
        data_parts = line.decode('utf-8').split(',')
        
        # Check if the format is correct and identifier is "03"
        if len(data_parts) == 12 and data_parts[0] == '03': # 1 identifier + 11 data fields
            # data1 is at index 1
            temperature_str = data_parts[2]
            return float(temperature_str)
        else:
            print(f"Skipping malformed or incorrect line: {line}")
            return None
    except Exception as e:
        print(f"Error reading serial data: {e}")
        return None

# --- Main Control Loop ---
def main():
    # Setup GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_PIN, GPIO.OUT, initial=GPIO.LOW) # Freezer initially OFF

    # Setup serial communication
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Wait for serial port to initialize
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        return

    # Setup PID controller
    # PID tuning is critical and these values are placeholders (P, I, D)
    pid = PID(P=0.5, I=0.01, D=0.1) 
    pid.set_point = TARGET_TEMP

    while True:
        cycle_start_time = time.time()
        while (time.time() - cycle_start_time) <= PID_RUN_TIME:
            # 1-1: 0~1500sec switch on/off freezer via gpio=18 using pid
            
            # Read temperature
            temp_c = read_temperature_from_serial(ser)
            
            if temp_c is not None:
                # Update PID controller and get output (0 to 1)
                pid_output = pid.update(temp_c)
                
                if pid_output is not None:
                    # PID output is a value between 0 and 1, representing the duty cycle
                    # For on/off control, we use a simple threshold or map to a PWM like behavior within the cycle
                    # Since the operation cycle is long (1800s), on/off based on output might be better with a faster sample time.
                    # A better way for freezers (long time constants) is simple on-off control with a hysteresis, 
                    # but adhering to the request, we use PID output for pulse modulation within the 1500s window.
                    
                    # Calculate on/off time based on PID output
                    # The PID output (0 to 1) dictates the proportion of time the freezer is ON
                    on_duration = SAMPLE_TIME * pid_output # Time ON within the sample period
                    off_duration = SAMPLE_TIME * (1 - pid_output) # Time OFF within the sample period
                    
                    if on_duration > 0:
                        GPIO.output(GPIO_PIN, GPIO.HIGH) # Turn freezer ON
                        time.sleep(on_duration)
                    
                    if off_duration > 0:
                        GPIO.output(GPIO_PIN, GPIO.LOW) # Turn freezer OFF
                        time.sleep(off_duration)
                    
                    print(f"Temp: {temp_c:.2f} C, PID output: {pid_output:.2f}, On duration: {on_duration:.2f}s, Off duration: {off_duration:.2f}s")
                else:
                    # Sleep for sample time if PID didn't update
                    time.sleep(SAMPLE_TIME)

        # 1-2: 1500~1800sec switch off freezer via gpio=18
        GPIO.output(GPIO_PIN, GPIO.LOW) # Ensure freezer is OFF for the remainder of the cycle
        print("PID run complete. Freezer forced OFF for 300 seconds.")
        time_remaining = OPERATION_CYCLE - (time.time() - cycle_start_time)
        if time_remaining > 0:
            time.sleep(time_remaining)
        
        # 2: The next operation cycle: go to (1) - The while True loop handles this

    ser.close()
    GPIO.cleanup()

if __name__ == "__main__":
    main()
