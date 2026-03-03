import time
import RPi.GPIO as GPIO

# Define GPIO pins connected to the line sensors
line_pin_left = 19    # Left line sensor connected to GPIO 19
line_pin_middle = 16  # Middle line sensor connected to GPIO 16
line_pin_right = 20   # Right line sensor connected to GPIO 20

def setup():
    # Set up GPIO settings for the line sensors
    GPIO.setwarnings(False)          # Disable GPIO warnings
    GPIO.setmode(GPIO.BCM)           # Set GPIO mode to BCM (Broadcom pin numbering)
    
    # Configure each line sensor pin as an input
    GPIO.setup(line_pin_right, GPIO.IN)    # Right sensor as input
    GPIO.setup(line_pin_middle, GPIO.IN)   # Middle sensor as input
    GPIO.setup(line_pin_left, GPIO.IN)     # Left sensor as input

def run():
    # Read the current state of each line sensor
    status_right = GPIO.input(line_pin_right)   # Read the right sensor's status
    status_middle = GPIO.input(line_pin_middle) # Read the middle sensor's status
    status_left = GPIO.input(line_pin_left)     # Read the left sensor's status
    
    # Print the status of each sensor in a formatted string
    print('LS: %d   MS: %d   RS: %d\n' % (status_left, status_middle, status_right))

# Main program execution
if __name__ == '__main__':
    try:
        setup()  # Initialize GPIO settings
        
        # Continuously read sensor values and print them until interrupted
        while True:
            run()              # Read and display sensor values
            time.sleep(0.2)    # Wait for 0.2 seconds before the next reading
            
    # Handle keyboard interruption (e.g., Ctrl+C) to safely exit
    except KeyboardInterrupt:
        print('Execution stopped.')    # Print exit message
        GPIO.cleanup()                 # Clean up GPIO settings to avoid issues on next run
