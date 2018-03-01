#!/usr/bin/python3

import smbus
import time
from math import *

bus = smbus.SMBus(1);			# 0 for R-Pi Rev. 1, 1 for Rev. 2

# General constants
EARTH_GRAVITY_MS2	= 9.80665 # m/s2
STANDARD_PRESSURE	= 1013.25 # hPa

# ADXL345 (accelerometer) constants 
ADXL345_ADDRESS	=	0x53
	
ADXL345_DEVID			=	0x00
ADXL345_THRESH_TAP		=	0x1D
ADXL345_OFSX			=	0x1E
ADXL345_OFSY			=	0x1F
ADXL345_OFSZ			=	0x20
	# Components Offset
	#The OFSX, OFSY, and OFSZ registers are each eight bits and
	#offer user-set offset adjustments in twos complement format
	#with a scale factor of 15.6 mg/LSB (that is, 0x7F = +2 g).
	# Real Offset : OFS_ x 15.625
ADXL345_DUR				=	0x21
ADXL345_Latent			=	0x22
ADXL345_Window			=	0x23
ADXL345_THRESH_ACT		=	0x24
ADXL345_THRESH_INACT	=	0x25
ADXL345_TIME_INACT		=	0x26
ADXL345_ACT_INACT_CTL	=	0x27
ADXL345_THRESH_FF		=	0x28
ADXL345_TIME_FF			=	0x29
ADXL345_TAP_AXES		=	0x2A
ADXL345_ACT_TAP_STATUS	=	0x2B
ADXL345_BW_RATE			=	0x2C
ADXL345_POWER_CTL		=	0x2D
ADXL345_INT_ENABLE		=	0x2E
ADXL345_INT_MAP			=	0x2F
ADXL345_INT_SOURCE		=	0x30
ADXL345_DATA_FORMAT		=	0x31
	# Register 0x31 - Data Format - Read/Write
	# D7: SELF_TEST	D6: SPI	D5: INT_INVERT	D4: 0	D3: FULL_RES	D2:Justify	D1-D0: Range
	# D1-D0 = 00: +/-2G	D1-D0 = 01: +/-4G	D1-D0 = 10: +/-8G	D1-D0 = 11: +/-16G
	# Range: 
	#	FULL_RES=1: 3.9 mG/LSP (0.00390625 G/LSP)
	#	+/-2G,10bit mode:  3.9 mG/LSP
	#	+/-4G,10bit mode:  7.8 mG/LSP
	#	+/-8G,10bit mode:  15.6 mG/LSP
	#	+/-16G,10bit mode:  31.2 mG/LSP
ADXL345_DATAX0			=	0x32
ADXL345_DATAX1			=	0x33
ADXL345_DATAY0			=	0x34
ADXL345_DATAY1			=	0x35
ADXL345_DATAZ0			=	0x36
ADXL345_DATAZ1			=	0x37
ADXL345_FIFO_CTL		=	0x38
ADXL345_FIFO_STATUS		=	0x39

ADXL345_SCALE_MULTIPLIER    = 0.00390625	# G/LSP

ADXL345_BW_RATE_3200HZ	= 0x0F
ADXL345_BW_RATE_1600HZ	= 0x0E
ADXL345_BW_RATE_800HZ	= 0x0D
ADXL345_BW_RATE_400HZ	= 0x0C
ADXL345_BW_RATE_200HZ	= 0x0B
ADXL345_BW_RATE_100HZ	= 0x0A # (default)
ADXL345_BW_RATE_50HZ	= 0x09
ADXL345_BW_RATE_25HZ	= 0x08

ADXL345_RANGE_2G		= 0x00 # +/-  2g (default)
ADXL345_RANGE_4G		= 0x01 # +/-  4g
ADXL345_RANGE_8G		= 0x02 # +/-  8g
ADXL345_RANGE_16G		= 0x03 # +/- 16g

ADXL345_MEASURE			= 0x08

#L3G4200D (Gyroscope) constants
L3G4200D_ADDRESS		=	0x69

L3G4200D_WHO_AM_I		=	0x0F
L3G4200D_CTRL_REG1		=	0x20
L3G4200D_CTRL_REG2		=	0x21
L3G4200D_CTRL_REG3		=	0x22
L3G4200D_CTRL_REG4		=	0x23
# Register 0x23 - Control Register 4 - Read/Write
# D7: BDU	D6: BLE	D5-D4: FS0-FS1	D3: 0	D2-D1:ST1-ST0	D0: SIM
# D5-D4 = 00: +/-2G	
# Range D5-D4: 
#	00 +/-250dps : 8.75mdps/LSP
#	01 +/-500dps : 17.50mdps/LSP
#	10 +/-1000dps : 35mdps/LSP
#	11 +/-2000dps : 70mdps/LSP
L3G4200D_CTRL_REG5		=	0x24
L3G4200D_REFERENCE		=	0x25
L3G4200D_OUT_TEMP		=	0x26
L3G4200D_STATUS_REG		=	0x27
L3G4200D_OUT_X_L		=	0x28
L3G4200D_OUT_X_H		=	0x29
L3G4200D_OUT_Y_L		=	0x2A
L3G4200D_OUT_Y_H		=	0x2B
L3G4200D_OUT_Z_L		=	0x2C
L3G4200D_OUT_Z_H		=	0x2D
L3G4200D_FIFO_CTRL_REG	=	0x2E
L3G4200D_FIFO_SRC_REG	=	0x2F
L3G4200D_INT1_CFG		=	0x30
L3G4200D_INT1_SRC		=	0x31
L3G4200D_INT1_TSH_XH	=	0x32
L3G4200D_INT1_TSH_XL	=	0x33
L3G4200D_INT1_TSH_YH	=	0x34
L3G4200D_INT1_TSH_YL	=	0x35
L3G4200D_INT1_TSH_ZH	=	0x36
L3G4200D_INT1_TSH_ZL	=	0x37
L3G4200D_INT1_DURATION	=	0x38

L3G4200D_RANGE_250		= 0x00	# +/-250dps : 8.75mdps/LSP
L3G4200D_RANGE_500		= 0x01	# +/-500dps : 17.50mdps/LSP
L3G4200D_RANGE_1000		= 0x02	# +/-1000dps : 35mdps/LSP
L3G4200D_RANGE_2000		= 0x03	# +/-2000dps : 70mdps/LSP

#HMC5883L (Magnetometer) constants
HMC5883L_ADDRESS		=	0x1E
	
HMC5883L_CRA			=	0x00
HMC5883L_CRB			=	0x01
HMC5883L_MR				=	0x02
HMC5883L_DO_X_H			=	0x03
HMC5883L_DO_X_L			=	0x04
HMC5883L_DO_Z_H			=	0x05
HMC5883L_DO_Z_L			=	0x06
HMC5883L_DO_Y_H			=	0x07
HMC5883L_DO_Y_L			=	0x08
HMC5883L_SR				=	0x09
HMC5883L_IR_A			=	0x0A
HMC5883L_IR_B			=	0x0B
HMC5883L_IR_C			=	0x0C

#BMP180 (Barometer) constants
BMP180_ADDRESS			= 0x77
BMP180_AC1				= 0xAA
BMP180_AC2				= 0xAC
BMP180_AC3				= 0xAE
BMP180_AC4				= 0xB0
BMP180_AC5				= 0xB2
BMP180_AC6				= 0xB4
BMP180_B1				= 0xB6 
BMP180_B2				= 0xB8 
BMP180_MB				= 0xBA 
BMP180_MC				= 0xBC 
BMP180_MD				= 0xBE 


class IMU(object):

	def write_byte(self,adr, value):
		bus.write_byte_data(self.ADDRESS, adr, value)
	
	def read_byte(self,adr):
		return bus.read_byte_data(self.ADDRESS, adr)

	def read_word(self,adr,rf=1):
		# rf=1 Little Endian Format, rf=0 Big Endian Format
		if (rf == 1):
			low = self.read_byte(adr)
			high = self.read_byte(adr+1)
		else:
			high = self.read_byte(adr)
			low = self.read_byte(adr+1)
		val = (high << 8) + low
		return val

	def read_word_2c(self,adr,rf=1):
		val = self.read_word(adr,rf)
		if(val & (1 << 16 - 1)):
			return val - (1<<16)
		else:
			return val

class gy801(object):
	def __init__(self) :
		self.accel = ADXL345()
		self.gyro = L3G4200D()
		self.compass = HMC5883L()
		self.baro = BMP180()


class ADXL345(IMU):
	
	ADDRESS = ADXL345_ADDRESS
	
	def __init__(self) :
		#Class Properties
		self.Xoffset = 0x00
		self.Yoffset = 0x00
		self.Zoffset = 0x00
		self.Xraw = 0.0
		self.Yraw = 0.0
		self.Zraw = 0.0
		self.Xg = 0.0
		self.Yg = 0.0
		self.Zg = 0.0
		self.X = 0.0
		self.Y = 0.0
		self.Z = 0.0
		self.df_value = 0b00001000	# Self test disabled, 4-wire interface
								# Full resolution, Range = +/-2g
		self.Xcalibr = ADXL345_SCALE_MULTIPLIER
		self.Ycalibr = ADXL345_SCALE_MULTIPLIER
		self.Zcalibr = ADXL345_SCALE_MULTIPLIER

		self.write_byte(ADXL345_BW_RATE, ADXL345_BW_RATE_100HZ)	# Normal mode, Output data rate = 100 Hz
		self.write_byte(ADXL345_POWER_CTL, ADXL345_MEASURE)	# Auto Sleep disable
		self.write_byte(ADXL345_DATA_FORMAT, self.df_value)	
	
	# RAW readings in LPS
	def getRawX(self) :
		self.X_raw = self.read_word_2c(ADXL345_DATAX0)
		return self.X_raw

	def getRawY(self) :
		self.Yraw = self.read_word_2c(ADXL345_DATAY0)
		return self.Yraw
	
	def getRawZ(self) :
		self.Zraw = self.read_word_2c(ADXL345_DATAZ0)
		return self.Zraw

	# G related readings in g
	def getXg(self,plf = 1.0) :
		self.Xg = (self.getRawX() * self.Xcalibr + self.Xoffset) * plf + (1.0 - plf) * self.Xg
		return self.Xg

	def getYg(self,plf = 1.0) :
		self.Yg = (self.getRawY() * self.Ycalibr + self.Yoffset) * plf + (1.0 - plf) * self.Yg
		return self.Yg

	def getZg(self,plf = 1.0) :
		self.Zg = (self.getRawZ() * self.Zcalibr + self.Zoffset) * plf + (1.0 - plf) * self.Zg
		return self.Zg
	
	# Absolute reading in m/s2
	def getX(self,plf = 1.0) :
		self.X = self.getXg(plf) * EARTH_GRAVITY_MS2
		return self.X
	
	def getY(self,plf = 1.0) :
		self.Y = self.getYg(plf) * EARTH_GRAVITY_MS2
		return self.Y
	
	def getZ(self,plf = 1.0) :
		self.Z = self.getZg(plf) * EARTH_GRAVITY_MS2
		return self.Z

	def getPitch(self) :
		aX = self.getXg()
		aY = self.getYg()
		aZ = self.getZg()
		self.pitch = atan2(aX,sqrt(aY*aY+aZ*aZ)) * 180.0/pi
		return self.pitch 

	def getRoll(self) :
		aX = self.getXg()
		aY = self.getYg()
		aZ = self.getZg()
		self.roll = atan2(aY,(sqrt(aX*aX+aZ*aZ))) * 180.0/pi
		return self.roll

class L3G4200D(IMU):
	
	ADDRESS = L3G4200D_ADDRESS

	def __init__(self) :
		#Class Properties
		self.Xraw = 0.0
		self.Yraw = 0.0
		self.Zraw = 0.0
		self.X = 0.0
		self.Y = 0.0
		self.Z = 0.0
		self.Xangle = 0.0
		self.Yangle = 0.0
		self.Zangle = 0.0
		self.gain_std = 0.00875	# dps/digit
		self.t0x = None
		self.t0y = None
		self.t0z = None
		
		self.write_byte(L3G4200D_CTRL_REG1, 0x0F)	# 0x0F(15)	Normal mode, X, Y, Z-Axis enabled 0xB0
		self.write_byte(L3G4200D_CTRL_REG4, 0x80)	# 0x30(48)	Block non continous update, Data LSB at lower address
											# FSR 250dps, Self test disabled, 4-wire interface
		#write(L3G4200D_CTRL_REG4, 0x30)	# 0x30(48)	Continous update, Data LSB at lower address
											## FSR 2000dps, Self test disabled, 4-wire interface
		self.setCalibration()

	def setCalibration(self) :
		gyr_r = self.read_byte(L3G4200D_CTRL_REG4)
		
		self.gain = 2 ** ( gyr_r & 48 >> 4) * self.gain_std

	def getRawX(self):
		self.Xraw = self.read_word_2c(L3G4200D_OUT_X_L)
		return self.Xraw

	def getRawY(self):
		self.Yraw = self.read_word_2c(L3G4200D_OUT_Y_L)
		return self.Yraw

	def getRawZ(self):
		self.Zraw = self.read_word_2c(L3G4200D_OUT_Z_L)
		return self.Zraw

	def getX(self,plf = 1.0):
		self.X = ( self.getRawX() * self.gain ) * plf + (1.0 - plf) * self.X
		return self.X

	def getY(self,plf = 1.0):
		self.Y = ( self.getRawY() * self.gain ) * plf + (1.0 - plf) * self.Y
		return self.Y

	def getZ(self,plf = 1.0):
		self.Z = ( self.getRawZ() * self.gain ) * plf + (1.0 - plf) * self.Z
		return self.Z
	
	def getXangle(self,plf = 1.0) :
		if self.t0x is None : self.t0x = time.time()
		t1x = time.time()
		LP = t1x - self.t0x
		self.t0x = t1x
		self.Xangle += self.getX(plf) * LP
		return self.Xangle
	
	def getYangle(self,plf = 1.0) :
		if self.t0y is None : self.t0y = time.time()
		t1y = time.time()
		LP = t1y - self.t0y
		self.t0y = t1y
		self.Yangle += self.getY(plf) * LP
		return self.Yangle
	
	def getZangle(self,plf = 1.0) :
		if self.t0z is None : self.t0z = time.time()
		t1z = time.time()
		LP = t1z - self.t0z
		self.t0z = t1z
		self.Zangle += self.getZ(plf) * LP
		return self.Zangle

class HMC5883L(IMU):
	
	ADDRESS = HMC5883L_ADDRESS

	def __init__(self) :
		#Class Properties
		self.X = None
		self.Y = None
		self.Z = None
		self.angle = None
		self.Xoffset = 0.0
		self.Yoffset = 0.0
		self.Zoffset = 0.0
		self.angle_offset = 0.0
		
		self.scale = 0.92

		self.write_byte(HMC5883L_CRA, 0b01110000)	# Set to 8 samples @ 15Hz
		self.write_byte(HMC5883L_CRB, 0b00100000)	# 1.3 gain LSb / Gauss 1090 (default)
		self.write_byte(HMC5883L_MR, 0b00000000)	# Continuous sampling

	def getX(self):
		self.X = (self.read_word_2c(HMC5883L_DO_X_H) - self.Xoffset) * self.scale
		return self.X

	def getY(self):
		self.Y = (self.read_word_2c(HMC5883L_DO_Y_H) - self.Yoffset) * self.scale
		return self.Y

	def getZ(self):
		self.Z = (self.read_word_2c(HMC5883L_DO_Z_H) - self.Zoffset) * self.scale
		return self.Z
	
	def getAngle(self):
		bearing  = degrees(atan2(self.getY(), self.getX())) + self.angle_offset
		if (bearing < 0):
			bearing += 360
		bearing += self.angle_offset
		if (bearing < 0):
			bearing += 360
		if (bearing > 360):
			bearing -= 360
		self.angle = bearing
		return self.angle

class BMP180(IMU):
	
	ADDRESS = BMP180_ADDRESS
	
	def __init__(self) :
		#Class Properties
		self.tempC = None
		self.tempF = None
		self.press = None
		self.altitude = None
		
		self.oversampling = 3        # 0..3
		
		self._read_calibratio_params()
		
	def _read_calibratio_params(self) :
		self.ac1_val = self.read_word_2c(BMP180_AC1,0)
		self.ac2_val = self.read_word_2c(BMP180_AC2,0)
		self.ac3_val = self.read_word_2c(BMP180_AC3,0)
		self.ac4_val = self.read_word(BMP180_AC4,0)
		self.ac5_val = self.read_word(BMP180_AC5,0)
		self.ac6_val = self.read_word(BMP180_AC6,0)
		self.b1_val = self.read_word_2c(BMP180_B1,0)
		self.b2_val = self.read_word_2c(BMP180_B2,0)
		self.mc_val = self.read_word_2c(BMP180_MC,0)
		self.md_val = self.read_word_2c(BMP180_MD,0)

	def getTempC(self) :

		# print ("Calculating temperature...")
		self.write_byte(0xF4, 0x2E)
		time.sleep(0.005)
		
		ut = self.read_word(0xF6,0)

		x1 = ((ut - self.ac6_val) * self.ac5_val) >> 15
		x2 = (self.mc_val << 11) // (x1 + self.md_val)
		b5 = x1 + x2 
		self.tempC = ((b5 + 8) >> 4) / 10.0
		
		return self.tempC

	def getTempF(self) :
		#print ("Calculating temperature (Fahrenheit)...")
		self.tempF = self.getTempC() * 1.8 + 32

		return self.tempF

	def getPress(self) :

		# print ("Calculating temperature...")
		self.write_byte(0xF4, 0x2E)
		time.sleep(0.005)
		
		ut = self.read_word(0xF6,0)

		x1 = ((ut - self.ac6_val) * self.ac5_val) >> 15
		x2 = (self.mc_val << 11) // (x1 + self.md_val)
		b5 = x1 + x2 

		#print ("Calculating pressure...")
		self.write_byte(0xF4, 0x34 + (self.oversampling << 6))
		time.sleep(0.04)

		msb = self.read_byte(0xF6)
		lsb = self.read_byte(0xF7)
		xsb = self.read_byte(0xF8)
		
		up = ((msb << 16) + (lsb << 8) + xsb) >> (8 - self.oversampling)

		b6 = b5 - 4000
		b62 = b6 * b6 >> 12
		x1 = (self.b2_val * b62) >> 11
		x2 = self.ac2_val * b6 >> 11
		x3 = x1 + x2
		b3 = (((self.ac1_val * 4 + x3) << self.oversampling) + 2) >> 2

		x1 = self.ac3_val * b6 >> 13
		x2 = (self.b1_val * b62) >> 16
		x3 = ((x1 + x2) + 2) >> 2
		b4 = (self.ac4_val * (x3 + 32768)) >> 15
		b7 = (up - b3) * (50000 >> self.oversampling)

		press = (b7 * 2) // b4
		#press = (b7 / b4) * 2

		x1 = (press >> 8) * (press >> 8)
		x1 = (x1 * 3038) >> 16
		x2 = (-7357 * press) >> 16
		self.press = ( press + ((x1 + x2 + 3791) >> 4) ) / 100.0
		
		return self.press

	def getAltitude(self) :
		#	print ("Calculating altitude...")
		self.altitude = 44330 * (1 - ((self.getPress() / STANDARD_PRESSURE) ** 0.1903))
		return self.altitude

if __name__ == "__main__":
	# if run directly we'll just create an instance of the class and output 
	# the current readings
	
	sensors = gy801()
	adxl345 = sensors.accel
	
	print ("\033[1;34;40mADXL345 on address 0x%x:" % (ADXL345_ADDRESS))
	print ("   x = %.3f m/s2" % ( adxl345.getX() ))
	print ("   y = %.3f m/s2" % ( adxl345.getY() ))
	print ("   z = %.3f m/s2" % ( adxl345.getZ() ))
	print ("   x = %.3fG" % ( adxl345.Xg ))
	print ("   y = %.3fG" % ( adxl345.Yg ))
	print ("   z = %.3fG" % ( adxl345.Zg ))
	print ("   x = %.3f" % ( adxl345.Xraw ))
	print ("   y = %.3f" % ( adxl345.Yraw ))
	print ("   z = %.3f" % ( adxl345.Zraw ))
	print ("   pitch = %.3f" % ( adxl345.getPitch() ))
	print ("   roll = %.3f" % ( adxl345.getRoll() ))

	gyro = sensors.gyro
	
	gyro.getXangle()
	gyro.getYangle()
	gyro.getZangle()
    
	print ("\033[1;33;40mL3G4200D on address 0x%x:" % (L3G4200D_ADDRESS))
	print ("   Xangle = %.3f deg" % ( gyro.getXangle() ))
	print ("   Yangle = %.3f deg" % ( gyro.getYangle() ))
	print ("   Zangle = %.3f deg" % ( gyro.getZangle() ))

	compass = sensors.compass
    
	print ("\033[1;32;40mHMC5883L on address 0x%x:" % (HMC5883L_ADDRESS))
	print ("   X = %.3f " % ( compass.getX() ))
	print ("   Y = %.3f " % ( compass.getY() ))
	print ("   Z = %.3f " % ( compass.getZ() ))
	print ("   Angle = %.3f deg" % ( compass.getAngle() ))

	barometer = sensors.baro
	
	tempC = barometer.getTempC()
	tempF = barometer.getTempF()
	press = barometer.getPress()
	altitude = barometer.getAltitude()
   
	print ("\033[1;31;40mBMP180 on address 0x%x:" % (BMP180_ADDRESS))
	print ("   Temp: %f C (%f F)" %(tempC,tempF))
	print ("   Press: %f (hPa)" %(press))
	print ("   Altitude: %f m s.l.m" %(altitude))
	print ("\033[0m")
