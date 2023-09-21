import time, sys, json, math
from Sensors import Gyroscope, Magnetometer, Accelerometer
from Kalman import KalmanAngle
import smbus2

MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
#AK8963 registers
AK8963_ADDR  = 0x0C
AK8963_ST1   = 0x02
HXH          = 0x04
HYH          = 0x06
HZH          = 0x08
AK8963_ST2   = 0x09
AK8963_CNTL  = 0x0A

bus =  smbus2.SMBus(1) # start comm with i2c bus

class InertialSensor():
	def __init__(self) -> None:
		self.config_MPU()
		self.config_AK()
		self.gyro = Gyroscope(bus)
		self.accel = Accelerometer(bus)
		self.mag = Magnetometer(bus)

		self.restrict_pitch = True
  
		self.comp =[0,0,0]
  
		self.k = [KalmanAngle(), KalmanAngle(), KalmanAngle()]
		self.inertial = [0,0,0]
  
		self.time_init = time.time()
		self.time_ref = self.time_init
		self.time_prev = self.time_init
		self.cycle = 0
		
	def setTimeRef(self):
		self.time_ref = time.time()

	def getTimeEleapsed(self):
		return time.time() - self.time_ref

	def getTimeDelta(self):
		ot = time.time()
		dt = ot - self.time_prev 
		self.time_prev = ot
  
		return dt, ot
  
	def read_raw_data(self):
		gyro = self.gyro.read()
		accel = self.accel.read()
		mag = self.mag.read()
		t = time.time() - self.time_init
		c = self.cycle
		self.cycle = self.cycle + 1
		return gyro, accel, mag, t, c

	def read_angles(self):
		gyro = self.gyro.angles()
		accel = self.accel.angles()
		mag = self.mag.heading()

		t = time.time() - self.time_init
		c = self.cycle
		self.cycle = self.cycle + 1
		return gyro, accel, mag, t, c
	
	def comp_filter(self):
		self.setTimeRef()

		gyro = self.gyro.angles()
		accel = self.accel.angles()
		mag = self.mag.heading()

		t = self.getTimeEleapsed()

		self.comp[0] += 0.96* ( (self.comp[0] +  gyro[0]) *  t ) + 0.04*accel[0]
		self.comp[1] += 0.96* ( (self.comp[1] +  gyro[1]) *  t ) + 0.04*accel[1]
		self.comp[2] += 0.96* ( (self.comp[2] +  gyro[2]) *  t ) + 0.04*accel[2]

		t = time.time() - self.time_init
		c = self.cycle
		self.cycle = self.cycle + 1
		return gyro, accel, mag, t, c, self.comp

	def kalman(self):
		gyro = self.gyro.readForK()
		accel = self.accel.angles()
		mag = self.mag.heading()

		gRateX, gRateY, gRateZ = gyro
  
		roll, yaw, pitch = accel 
  
		self.k[0].setAngle(roll)
		self.k[1].setAngle(yaw)
		self.k[2].setAngle(pitch)
  
		dt, ot = self.getTimeDelta()
  
		if((pitch < -90 and self.inertial[2] >90) or (pitch > 90 and self.inertial[2] < -90)):
			self.k[2].setAngle(pitch)
			self.inertial[2] = pitch
		else:
			self.inertial[2] = self.k[2].getAngle(pitch, gRateZ, dt)
			
		if(abs( self.inertial[2] > 90)):
			gRateX = -gRateX
			self.inertial[0] = self.k[0].getAngle(roll, gRateX, dt)
   
		self.inertial[1] = self.k[1].getAngle(yaw, gRateY, dt)
  
		c = self.cycle
		self.cycle = self.cycle +1
  
		return self.inertial, mag, dt, ot, c

  

	def config_MPU(self):
		samp_rate_div = 0 # sample rate = 8 kHz/(1+samp_rate_div)
		bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, samp_rate_div)
		time.sleep(0.1)
		# reset all sensors
		bus.write_byte_data(MPU6050_ADDR,PWR_MGMT_1,0x00)
		time.sleep(0.1)
		# power management and crystal settings
		bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x01)
		time.sleep(0.1)
		#Write to Configuration register
		bus.write_byte_data(MPU6050_ADDR, CONFIG, int('0000110',2))
												#this remove vibrations???
		time.sleep(0.1)
		#Write to Gyro configuration register
		gyro_config_sel = [0b00000,0b010000,0b10000,0b11000] # byte registers
		gyro_config_vals = [250.0,500.0,1000.0,2000.0] # degrees/sec
		gyro_indx = 0
		bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, int(gyro_config_sel[gyro_indx]))
		time.sleep(0.1)
		#Write to Accel configuration register
		accel_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
		accel_config_vals = [2.0,4.0,8.0,16.0] # g (g = 9.81 m/s^2)
		accel_indx = 0                            
		bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, int(accel_config_sel[accel_indx]))
		time.sleep(0.1)
		# interrupt register (related to overflow of data [FIFO])
		bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)
		time.sleep(0.1)
	
	def config_AK(self):
		bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x00)
		time.sleep(0.1)
		AK8963_bit_res = 0b0001 # 0b0001 = 16-bit
		AK8963_samp_rate = 0b0110 # 0b0010 = 8 Hz, 0b0110 = 100 Hz
		AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate # bit conversion
		bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,AK8963_mode)
		time.sleep(0.1)