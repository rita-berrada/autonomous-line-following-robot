from mpu6050 import mpu6050
import time
import math

sensor = mpu6050(0x68)

def calculate_angles_mpu6050():
    accel_x = 0
    accel_y = 0
    accel_z = 0
    for i in range(0,10):
      accelerometer_data = sensor.get_accel_data()
      accel_x +=  accelerometer_data['x'] # - 1.432
      accel_y +=  accelerometer_data['y'] + 0.200
      accel_z +=  accelerometer_data['z'] - 1.940
    
    print('\nX=%.2f, Y=%.2f, Z=%.2f'%(accel_x/10, accel_y/10, accel_z/10))

    # Calculate roll and pitch angles
    roll = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi
    pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180 / math.pi
    
    print(f"\nRoll: {roll:.2f}, Pitch: {pitch:.2f}")

if __name__ == "__main__":
    try:
        while True:
            calculate_angles_mpu6050()    # Calculate and print angles
            time.sleep(0.5)             # Delay between readings
    except KeyboardInterrupt:
        print("\nProgram interrupted.")



#######################################################################
#
#          		Low level Programming
#
#######################################################################

'''
import smbus
import time
import math

# MPU6050 Registers
MPU6050_ADDR = 0x68           # MPU6050 I2C address
PWR_MGMT_1 = 0x6B             # Power management register
ACCEL_XOUT_H = 0x3B           # Accelerometer X-axis data register
ACCEL_YOUT_H = 0x3D           # Accelerometer Y-axis data register
ACCEL_ZOUT_H = 0x3F           # Accelerometer Z-axis data register

# Initialize I2C bus
bus = smbus.SMBus(1)
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)  # Wake up MPU6050

def read_word_smbus(register):
    """Reads two bytes of data from the specified register."""
    high = bus.read_byte_data(MPU6050_ADDR, register)
    low = bus.read_byte_data(MPU6050_ADDR, register + 1)
    value = (high << 8) + low
    if value >= 0x8000:        # Convert to signed value
        value = -((65535 - value) + 1)
    return value

def get_accel_data_smbus():
    """Fetches raw accelerometer data from the MPU6050."""
    accel_x = read_word_smbus(ACCEL_XOUT_H)
    accel_y = read_word_smbus(ACCEL_YOUT_H)
    accel_z = read_word_smbus(ACCEL_ZOUT_H)
    return accel_x, accel_y, accel_z

def calculate_angles_smbus():
    """Calculates and prints the roll and pitch angles based on accelerometer data."""
    accel_x, accel_y, accel_z = get_accel_data_smbus()

    # Calculate roll and pitch angles
    roll = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi
    pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180 / math.pi
    
    print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}")

if __name__ == "__main__":
    try:
        while True:
            calculate_angles_smbus()    # Calculate and print angles
            time.sleep(0.5)       # Delay between readings
    except KeyboardInterrupt:
        print("\nProgram interrupted.")
'''