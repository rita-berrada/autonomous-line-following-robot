#RPI Servo (WITH ALL THE CALIBRATIONS)
from __future__ import division
import time
import RPi.GPIO as GPIO
import sys
import Adafruit_PCA9685

import random

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)

init_pwm0 = 315
init_pwm1 = 320
init_pwm2 = 175

class ServoCtrl():

	def __init__(self):
		self.sc_direction = [1, 1, 1]
		self.initPos = [init_pwm0, init_pwm1, init_pwm2]
		self.nowPos  = [300, 320, 175]
		self.lastPos = [300, 320, 175]
		self.maxPos  = [500, 550, 275]
		self.minPos  = [100, 60,75]

		self.ctrlRangeMax = 550
		self.ctrlRangeMin = 75
		self.angleRange = 280

	def moveInit(self):
		for i in range(0,3):
			pwm.set_pwm(i,0,self.initPos[i])
			self.lastPos[i] = self.initPos[i]
			self.nowPos[i] = self.initPos[i]

	def pwmGenOut(self, angleInput):
		return int(round(((self.ctrlRangeMax-self.ctrlRangeMin)/self.angleRange*angleInput),0))

	def moveAngle(self, ID, angleInput):
		self.nowPos[ID] = int(self.initPos[ID] + self.sc_direction[ID]*self.pwmGenOut(angleInput))
		if self.nowPos[ID] > self.maxPos[ID]:self.nowPos[ID] = self.maxPos[ID]
		elif self.nowPos[ID] < self.minPos[ID]:self.nowPos[ID] = self.minPos[ID]
		self.lastPos[ID] = self.nowPos[ID]
		pwm.set_pwm(ID, 0, self.nowPos[ID])

if __name__ == '__main__':
	sc = ServoCtrl()
	while 1:
		sc.moveAngle(1,150)
		time.sleep(1)
		sc.moveAngle(1,0)
		time.sleep(1)
		sc.moveAngle(1,-150)
		time.sleep(1)

		#sc.moveAngle(0,(random.random()*100-50))
		#time.sleep(1)
		#sc.moveAngle(1,(random.random()*100-50))
		#time.sleep(1)
		pass
	pass
