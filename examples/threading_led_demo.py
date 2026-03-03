import RPi.GPIO as GPIO
import threading
import time
from class_LED import class_LED

LEDB_status=1
stop_thread=False

LED1_PIN = 5
LED2_PIN = 6
LED3_PIN = 13

def switchSetup(LED2_PIN, LED3_PIN):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED2_PIN, GPIO.OUT)
    GPIO.setup(LED3_PIN, GPIO.OUT)

def LEDB_statuschange(delay_status):
  global LEDB_status
  while not stop_thread:
    LEDB_status = not(LEDB_status)
    time.sleep(2)
    
def LEDB(LED1_PIN, delay_blinking):
  global LEDB_status
  while not stop_thread:
    if LEDB_status==1:
      GPIO.output(LED1_PIN, GPIO.HIGH)
      time.sleep(delay_blinking)
    GPIO.output(LED1_PIN,GPIO.LOW)
    time.sleep(delay_blinking)
      
def switch(port, status):
    if port == 0:
        if status == 1:
            GPIO.output(6, GPIO.HIGH)
        elif status == 0:
            GPIO.output(6,GPIO.LOW)
        else:
            pass
    elif port == 1:
        if status == 1:
            GPIO.output(13, GPIO.HIGH)
        elif status == 0:
            GPIO.output(13,GPIO.LOW)
        else:
            pass
    else:
        print('Wrong Command.')

def set_all_switch_off():
    switch(0,0)
    switch(1,0)

if __name__ == "__main__":
    switchSetup(LED2_PIN, LED3_PIN)
    stop_event = threading.Event()
    LED_Blink = class_LED(LED1_PIN)
    LED_Blink.switchSetup()
    delay_blinking = 0.2
    delay_status = 3
    thread1=threading.Thread(target = LED_Blink.LEDB, args = (LED1_PIN, delay_blinking, stop_event) )
    thread2=threading.Thread(target = LED_Blink.LEDB_statuschange, args = (delay_status, stop_event) )
    thread1.start()
    thread2.start()
    try:
      while 1:
        switch(0,1)  # Right LED
        switch(1,1)  # Left LED
        print("Light on...")
        time.sleep(0.5)
        set_all_switch_off()
        print("Light off...")
        time.sleep(0.5)
    except KeyboardInterrupt:
        stop_event.set()
        # Turn off all LEDs before exiting
        time.sleep(1)
        thread1.join()
        thread2.join()
        print('Threads are requested to stop safely - please wait!')
        time.sleep(2)
        print('Threads have been stopped.')
        
        