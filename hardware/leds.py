import RPi.GPIO as GPIO
import threading
import time

LEDB_status=1
stop_thread=False

def switchSetup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(5, GPIO.OUT)
    GPIO.setup(6, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)

def LEDB_statuschange():
  global LEDB_status
  while not stop_thread:
    LEDB_status = not(LEDB_status)
    time.sleep(2)
    
def LEDB():
  global LEDB_status
  while not stop_thread:
    if LEDB_status==1:
      GPIO.output(5, GPIO.HIGH)
      time.sleep(0.1)
    GPIO.output(5,GPIO.LOW)
    time.sleep(0.1)
    
      
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
    switchSetup()
    thread1=threading.Thread(target=LEDB)
    thread2=threading.Thread(target=LEDB_statuschange)
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
    except:
      stop_thread=True
      print('Threads are requested to stop safely - please wait!')
      time.sleep(2)
      print('Threads have been stopped.')
        
        