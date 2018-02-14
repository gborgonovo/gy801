#!/usr/bin/python

from smbus import SMBus
import time
from ctypes import c_short

BMP180_ADDRESS = 0x77
oversampling = 3        # 0..3

bus = SMBus(1);         # 0 for R-Pi Rev. 1, 1 for Rev. 2

# return two bytes from data as a signed 16-bit value
def get_short(data, index):
	return c_short((data[index] << 8) + data[index + 1]).value

# return two bytes from data as an unsigned 16-bit value
def get_ushort(data, index):
	return (data[index] << 8) + data[index + 1]

def writeBMP180(register,value):
	bus.write_byte_data(BMP180_ADDRESS , register, value)
	return -1

def readBMP180(register,nbytes):
	return bus.read_i2c_block_data(BMP180_ADDRESS, register, nbytes)

def getChip() :
	# (chip_id, version)
	return readBMP180(0xD0, 2)

def getTempC() :
	
	# Read whole calibration EEPROM data
	cal = readBMP180(0xAA, 22)

	# Convert byte data to word values
	ac1 = get_short(cal, 0)
	ac2 = get_short(cal, 2)
	ac3 = get_short(cal, 4)
	ac4 = get_ushort(cal, 6)
	ac5 = get_ushort(cal, 8)
	ac6 = get_ushort(cal, 10)
	b1 = get_short(cal, 12)
	b2 = get_short(cal, 14)
	mb = get_short(cal, 16)
	mc = get_short(cal, 18)
	md = get_short(cal, 20)

	# print ("Calculating temperature...")
	writeBMP180(0xF4, 0x2E)
	time.sleep(0.005)

	(msb, lsb) = readBMP180(0xF6, 2)
	ut = (msb << 8) + lsb

	x1 = ((ut - ac6) * ac5) >> 15
	x2 = (mc << 11) // (x1 + md)
	b5 = x1 + x2 
	tempC = ((b5 + 8) >> 4) / 10.0
	
	return tempC

def getTempF() :
	#print ("Calculating temperature (Fahrenheit)...")
	tempF = getTempC() * 1.8 + 32

	return tempF

def getPress() :
	
	# Read whole calibration EEPROM data
	cal = readBMP180(0xAA, 22)

	# Convert byte data to word values
	ac1 = get_short(cal, 0)
	ac2 = get_short(cal, 2)
	ac3 = get_short(cal, 4)
	ac4 = get_ushort(cal, 6)
	ac5 = get_ushort(cal, 8)
	ac6 = get_ushort(cal, 10)
	b1 = get_short(cal, 12)
	b2 = get_short(cal, 14)
	mb = get_short(cal, 16)
	mc = get_short(cal, 18)
	md = get_short(cal, 20)

	writeBMP180(0xF4, 0x2E)
	time.sleep(0.005)
	(msb, lsb) = readBMP180(0xF6, 2)
	ut = (msb << 8) + lsb

	x1 = ((ut - ac6) * ac5) >> 15
	x2 = (mc << 11) // (x1 + md)
	b5 = x1 + x2 

	#print ("Calculating pressure...")
	writeBMP180(0xF4, 0x34 + (oversampling << 6))
	time.sleep(0.04)

	(msb, lsb, xsb) = readBMP180(0xF6, 3)
	up = ((msb << 16) + (lsb << 8) + xsb) >> (8 - oversampling)

	b6 = b5 - 4000
	b62 = b6 * b6 >> 12
	x1 = (b2 * b62) >> 11
	x2 = ac2 * b6 >> 11
	x3 = x1 + x2
	b3 = (((ac1 * 4 + x3) << oversampling) + 2) >> 2

	x1 = ac3 * b6 >> 13
	x2 = (b1 * b62) >> 16
	x3 = ((x1 + x2) + 2) >> 2
	b4 = (ac4 * (x3 + 32768)) >> 15
	b7 = (up - b3) * (50000 >> oversampling)

	press = (b7 * 2) // b4
	#press = (b7 / b4) * 2

	x1 = (press >> 8) * (press >> 8)
	x1 = (x1 * 3038) >> 16
	x2 = (-7357 * press) >> 16
	press = ( press + ((x1 + x2 + 3791) >> 4) ) / 100.0
	
	return press

def getAltitude() :
	#	print ("Calculating altitude...")
	altitude = 44330 * (1 - ((getPress() / 1013.25) ** 0.1903))
	return altitude

print
print ("Temperature:", getTempC(), "C")
print ("Temperature (F):", getTempF(), "F")
print ("Pressure:", getPress(), "hPa")
print ("Altitude:", getAltitude(), "m")
