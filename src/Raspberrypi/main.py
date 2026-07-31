import sys
sys.path.append("/home/peach/DFRobot_RaspberryPi_Expansion_Board")
from DFRobot_RaspberryPi_Expansion_Board import DFRobot_Expansion_Board_IIC as Board
from DFRobot_RaspberryPi_Expansion_Board import DFRobot_Expansion_Board_Servo as Servo

import serial
import time
from collections import deque
import RPi.GPIO as GPIO
import board as board_pins
import busio
import adafruit_bno055
COMPASS_AVAILABLE = True

print("Initializing GPIO...")
try:
    GPIO.cleanup()
except:
    pass

time.sleep(0.2)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# ============================================================================
# Initialize the Expansion Board and Servo controller
# ============================================================================
board = Board(1, 0x10)

while board.begin() != board.STA_OK:
    print("Board init failed, retrying...")
    time.sleep(1)
print("Board init success!")

servo = Servo(board)
servo.begin()

board.set_adc_enable()   # required before reading analog channels

VIN_MV = 5000  # 5V supply
# VIN_MV = 3300 # uncomment this line if the URM09 is powered from the 3.3V rail

def read_distance():
    vout = board.get_adc_value(board.A0)   # analog0 -> A0
    distance_cm = vout * 520 / VIN_MV
    return round(distance_cm, 2)

# ============================================================================
# Move servo + read distance
# ============================================================================
servo.move(1, 90)
time.sleep(1)
print(f"Distance: {read_distance()} cm")

servo.move(1, 0)
time.sleep(1)
print(f"Distance: {read_distance()} cm")

servo.move(1, 180)
time.sleep(1)
print(f"Distance: {read_distance()} cm")