import RPi.GPIO as GPIO
import threading
import time

class class_LED:
    def __init__(self, LED_PIN):
        self.LED_PIN = LED_PIN
        self.LEDB_status = False  # Initialize LED status as False
    
    def switchSetup(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.LED_PIN, GPIO.OUT)
    
    def LEDB_statuschange(self, delay_status, stop_event):
        self.delay_status = delay_status
        self.stop_event = stop_event
        while not self.stop_event.is_set():  # Check the stop event periodically
            self.LEDB_status = not self.LEDB_status  # Toggle the LED status
            time.sleep(self.delay_status)
        print("Thread of LEDB_statuschange is stopped")
    
    def LEDB(self, LED_PIN, delay_blinking, stop_event):
        self.LED_PIN = LED_PIN
        self.delay_blinking = delay_blinking
        self.stop_event = stop_event
        while not self.stop_event.is_set():  # Check the stop event periodically
            if self.LEDB_status:
                GPIO.output(self.LED_PIN, GPIO.HIGH)
                time.sleep(self.delay_blinking)
            GPIO.output(self.LED_PIN, GPIO.LOW)
            time.sleep(self.delay_blinking)
        print("Thread of LEDB is stopped")
