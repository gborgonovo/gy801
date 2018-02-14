import smbus
from GY801 import *
import time

bus = smbus.SMBus(1)

def writeACC(register,value):
	bus.write_byte_data(ACC_ADDRESS , register, value)
	return -1

def writeMAG(register,value):
	bus.write_byte_data(MAG_ADDRESS, register, value)
	return -1

def writeGRY(register,value):
	bus.write_byte_data(GYR_ADDRESS, register, value)
	return -1



def readACCx():
	acc_l = bus.read_byte_data(ACC_ADDRESS, ADXL345_DATAX0)
	acc_h = bus.read_byte_data(ACC_ADDRESS, ADXL345_DATAX1)

	acc_combined = ((acc_h & 0x03) * 256) + acc_l
	if acc_combined > 511 :
		acc_combined -= 1024

	return acc_combined


def readACCy():
	acc_l = bus.read_byte_data(ACC_ADDRESS, ADXL345_DATAY0)
	acc_h = bus.read_byte_data(ACC_ADDRESS, ADXL345_DATAY1)

	acc_combined = ((acc_h & 0x03) * 256) + acc_l
	if acc_combined > 511 :
		acc_combined -= 1024

	return acc_combined


def readACCz():
	acc_l = bus.read_byte_data(ACC_ADDRESS, ADXL345_DATAZ0)
	acc_h = bus.read_byte_data(ACC_ADDRESS, ADXL345_DATAZ1)

	acc_combined = ((acc_h & 0x03) * 256) + acc_l
	if acc_combined > 511 :
		acc_combined -= 1024

	return acc_combined


def readMAGx():
	mag_l = bus.read_byte_data(MAG_ADDRESS, HMC5883_DO_X_MSB)
	mag_h = bus.read_byte_data(MAG_ADDRESS, HMC5883_DO_X_LSB)
	mag_combined = mag_l * 256 + mag_h
	if mag_combined > 32767 :
		mag_combined -= 65536
		
	return mag_combined


def readMAGy():
	mag_l = bus.read_byte_data(MAG_ADDRESS, HMC5883_DO_Y_MSB)
	mag_h = bus.read_byte_data(MAG_ADDRESS, HMC5883_DO_Y_LSB)
	mag_combined = mag_l * 256 + mag_h
	if mag_combined > 32767 :
		mag_combined -= 65536
		
	return mag_combined


def readMAGz():
	mag_l = bus.read_byte_data(MAG_ADDRESS, HMC5883_DO_Z_MSB)
	mag_h = bus.read_byte_data(MAG_ADDRESS, HMC5883_DO_Z_LSB)
	mag_combined = mag_l * 256 + mag_h
	if mag_combined > 32767 :
		mag_combined -= 65536
		
	return mag_combined


def readGYRx():
	gyr_l = bus.read_byte_data(GYR_ADDRESS, L3G4200D_OUT_X_L)
	gyr_h = bus.read_byte_data(GYR_ADDRESS, L3G4200D_OUT_X_H)

	gyr_combined = gyr_h * 256 + gyr_l
	if gyr_combined > 32767 :
		gyr_combined -= 65536
	
	return gyr_combined
  

def readGYRy():
	gyr_l = bus.read_byte_data(GYR_ADDRESS, L3G4200D_OUT_Y_L)
	gyr_h = bus.read_byte_data(GYR_ADDRESS, L3G4200D_OUT_Y_H)

	gyr_combined = gyr_h * 256 + gyr_l
	if gyr_combined > 32767 :
		gyr_combined -= 65536
	
	return gyr_combined

def readGYRz():
	gyr_l = bus.read_byte_data(GYR_ADDRESS, L3G4200D_OUT_Z_L)
	gyr_h = bus.read_byte_data(GYR_ADDRESS, L3G4200D_OUT_Z_H)

	gyr_combined = gyr_h * 256 + gyr_l
	if gyr_combined > 32767 :
		gyr_combined -= 65536
	
	return gyr_combined




def initIMU():
	#initialise the accelerometer
	writeACC(ADXL345_BW_RATE, 0x0A) #Normal mode, Output data rate = 100 Hz
	writeACC(ADXL345_POWER_CTL, 0x08)	#Auto Sleep disable
	writeACC(ADXL345_DATA_FORMAT, 0x08)	# Self test disabled, 4-wire interface
										# Full resolution, Range = +/-2g

	#initialise the magnetometer
	writeMAG(HMC5883_CRA, 0x60)	# 0x60(96)	Normal measurement configuration, Data output rate = 0.75 Hz
	writeMAG(HMC5883_MR, 0x00)	# 0x00(00)	Continuous measurement mode

	#initialise the gyroscope
	writeGRY(L3G4200D_CTRL_REG1, 0x0F)	#		0x0F(15)	Normal mode, X, Y, Z-Axis enabled
	writeGRY(L3G4200D_CTRL_REG4, 0x30)	#		0x30(48)	Continous update, Data LSB at lower address
										#		FSR 2000dps, Self test disabled, 4-wire interface



