/*
 * ADXL345mw.cpp - Class file for stripped back access to ADXL345 on Arduino.
 */

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#include "ADXL345mw.h"


namespace adxl345 {

Vector::Vector(int16_t _x, int16_t _y, int16_t _z, float n)
	: x(_x * n), y(_y * n), z(_z * n) {
}

void Vector::lowPassFilter(Vector vector, float alpha) {
	x = vector.x * alpha + (x * (1.0 - alpha));
	y = vector.y * alpha + (y * (1.0 - alpha));
	z = vector.z * alpha + (z * (1.0 - alpha));
}

I2cInterface::I2cInterface(): isValid(false) {
	
}

bool I2cInterface::begin()
{
	Wire.begin();
	// Check ADXL345 REG DEVID
	if (fastRegister8(kRegDevid) != 0xE5) {
		isValid = false;
		return isValid;
	}
	isValid = true;
	// Enable measurement mode (0b00001000)
	writeRegister8(kRegActPowerCtl, 0x08);
	clearSettings();
	return true;
}

/**
 *  Set Range
 */
void I2cInterface::setRange(range_t range)
{
	// Get actual value register
	uint8_t value = readRegister8(kRegDataFormat);
	
	// Update the data rate
	// (&) 0b11110000 (0xF0 - Leave HSB)
	// (|) 0b0000xx?? (range - Set range)
	// (|) 0b00001000 (0x08 - Set Full Res)
	value &= 0xF0;
	value |= range;
	value |= 0x08;
	
	writeRegister8(kRegDataFormat, value);
}

/**
 *  Get Range
 */
range_t I2cInterface::getRange(void)
{
	return (range_t)(readRegister8(kRegDataFormat) & 0x03);
}

/**
 *  Set Data Rate
 */
void I2cInterface::setDataRate(dataRate_t dataRate)
{
	writeRegister8(kRegActBwCtl, dataRate);
}

/**
 *  Get Data Rate
 */
dataRate_t I2cInterface::getDataRate(void)
{
	return (dataRate_t)(readRegister8(kRegActBwCtl) & 0x0F);
}


/**
 *  Read raw values
 */
void I2cInterface::readRaw(int16_t &x, int16_t &y, int16_t &z)
{
	x = readRegister16(kRegDataX0);
	y = readRegister16(kRegDataY0);
	z = readRegister16(kRegDataZ0);
}

/**
 *  Read normalized values
 */
Vector I2cInterface::readNormalize(float gravityFactor)
{
	int16_t x, y, z;
	readRaw(x, y, z);
	return Vector(x, y, z, 0.004f * gravityFactor);
}

/**
 *  Read scaled values
 */
Vector I2cInterface::readScaled(void)
{
	int16_t x, y, z;
	readRaw(x, y, z);
	return Vector(x, y, z, 0.004f);
}

void I2cInterface::clearSettings(void)
{
	setRange(kRange2G);
	setDataRate(kDatarate100HZ);

	writeRegister8(kRegThreshTap, 0x00);
	writeRegister8(kRegDur, 0x00);
	writeRegister8(kRegLatent, 0x00);
	writeRegister8(kRegWindow, 0x00);
	writeRegister8(kRegThreshAct, 0x00);
	writeRegister8(kRegThreshInact, 0x00);
	writeRegister8(kRegTimeInact, 0x00);
	writeRegister8(kRegThreshFreefall, 0x00);
	writeRegister8(kRegTimeFreefall, 0x00);

	uint8_t value;

	value = readRegister8(kRegActInactCtl);
	value &= 0b10001000;
	writeRegister8(kRegActInactCtl, value);

	value = readRegister8(kRegTapAxes);
	value &= 0b11111000;
	writeRegister8(kRegTapAxes, value);
}

/**
 *  Set Tap Threshold (62.5mg / LSB)
 */
void I2cInterface::setTapThreshold(float threshold)
{
	uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
	writeRegister8(kRegThreshTap, scaled);
}

/**
 *  Get Tap Threshold (62.5mg / LSB)
 */
float I2cInterface::getTapThreshold(void)
{
	return readRegister8(kRegThreshTap) * 0.0625f;
}

/**
 *  Set Tap Duration (625us / LSB)
 */
void I2cInterface::setTapDuration(float duration)
{
	uint8_t scaled = constrain(duration / 0.000625f, 0, 255);
	writeRegister8(kRegDur, scaled);
}

/**
 *  Get Tap Duration (625us / LSB)
 */
float I2cInterface::getTapDuration(void)
{
	return readRegister8(kRegDur) * 0.000625f;
}

/**
 *  Set Double Tap Latency (1.25ms / LSB)
 */
void I2cInterface::setDoubleTapLatency(float latency)
{
	uint8_t scaled = constrain(latency / 0.00125f, 0, 255);
	writeRegister8(kRegLatent, scaled);
}

/**
 *  Get Double Tap Latency (1.25ms / LSB)
 */
float I2cInterface::getDoubleTapLatency()
{
	return readRegister8(kRegLatent) * 0.00125f;
}

/**
 *  Set Double Tap Window (1.25ms / LSB)
 */
void I2cInterface::setDoubleTapWindow(float window)
{
	uint8_t scaled = constrain(window / 0.00125f, 0, 255);
	writeRegister8(kRegWindow, scaled);
}

/**
 *  Get Double Tap Window (1.25ms / LSB)
 */
float I2cInterface::getDoubleTapWindow(void)
{
	return readRegister8(kRegWindow) * 0.00125f;
}

/**
 *  Set Activity Threshold (62.5mg / LSB)
 */
void I2cInterface::setActivityThreshold(float threshold)
{
	uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
	writeRegister8(kRegThreshAct, scaled);
}

/**
 *  Get Activity Threshold (65.5mg / LSB)
 */
float I2cInterface::getActivityThreshold(void)
{
	return readRegister8(kRegThreshAct) * 0.0625f;
}

/**
 *  Set Inactivity Threshold (65.5mg / LSB)
 */
void I2cInterface::setInactivityThreshold(float threshold)
{
	uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
	writeRegister8(kRegThreshInact, scaled);
}

/**
 *  Get Incactivity Threshold (65.5mg / LSB)
 */
float I2cInterface::getInactivityThreshold(void)
{
	return readRegister8(kRegThreshInact) * 0.0625f;
}

/**
 *  Set Inactivity Time (s / LSB)
 */
void I2cInterface::setTimeInactivity(uint8_t time)
{
	writeRegister8(kRegTimeInact, time);
}

/**
 *  Get Inactivity Time (s / LSB)
 */
uint8_t I2cInterface::getTimeInactivity(void)
{
	return readRegister8(kRegTimeInact);
}

/**
 *  Set Free Fall Threshold (65.5mg / LSB)
 */
void I2cInterface::setFreeFallThreshold(float threshold)
{
	uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
	writeRegister8(kRegThreshFreefall, scaled);
}

/**
 *  Get Free Fall Threshold (65.5mg / LSB)
 */
float I2cInterface::getFreeFallThreshold(void)
{
	return readRegister8(kRegThreshFreefall) * 0.0625f;
}

/**
 *  Set Free Fall Duratiom (5ms / LSB)
 */
void I2cInterface::setFreeFallDuration(float duration)
{
	uint8_t scaled = constrain(duration / 0.005f, 0, 255);
	writeRegister8(kRegTimeFreefall, scaled);
}

/**
 *  Get Free Fall Duratiom
 */
float I2cInterface::getFreeFallDuration()
{
	return readRegister8(kRegTimeFreefall) * 0.005f;
}

void I2cInterface::setActivityX(bool state)
{
	writeRegisterBit(kRegActInactCtl, 6, state);
}

bool I2cInterface::getActivityX(void)
{
	return readRegisterBit(kRegActInactCtl, 6);
}

void I2cInterface::setActivityY(bool state)
{
	writeRegisterBit(kRegActInactCtl, 5, state);
}

bool I2cInterface::getActivityY(void)
{
	return readRegisterBit(kRegActInactCtl, 5);
}

void I2cInterface::setActivityZ(bool state)
{
	writeRegisterBit(kRegActInactCtl, 4, state);
}

bool I2cInterface::getActivityZ(void)
{
	return readRegisterBit(kRegActInactCtl, 4);
}

void I2cInterface::setActivityXYZ(bool state)
{
	uint8_t value;

	value = readRegister8(kRegActInactCtl);

	if (state) {
		value |= 0b00111000;
	} else {
		value &= 0b11000111;
	}

	writeRegister8(kRegActInactCtl, value);
}


void I2cInterface::setInactivityX(bool state) 
{
	writeRegisterBit(kRegActInactCtl, 2, state);
}

bool I2cInterface::getInactivityX(void)
{
	return readRegisterBit(kRegActInactCtl, 2);
}

void I2cInterface::setInactivityY(bool state)
{
	writeRegisterBit(kRegActInactCtl, 1, state);
}

bool I2cInterface::getInactivityY(void)
{
	return readRegisterBit(kRegActInactCtl, 1);
}

void I2cInterface::setInactivityZ(bool state)
{
	writeRegisterBit(kRegActInactCtl, 0, state);
}

bool I2cInterface::getInactivityZ(void)
{
	return readRegisterBit(kRegActInactCtl, 0);
}

void I2cInterface::setInactivityXYZ(bool state)
{
	uint8_t value = readRegister8(kRegActInactCtl);
	if (state) {
		value |= 0b00000111;
	} else {
		value &= 0b11111000;
	}
	writeRegister8(kRegActInactCtl, value);
}

void I2cInterface::setTapDetectionX(bool state)
{
	writeRegisterBit(kRegTapAxes, 2, state);
}

bool I2cInterface::getTapDetectionX(void)
{
	return readRegisterBit(kRegTapAxes, 2);
}

void I2cInterface::setTapDetectionY(bool state)
{
	writeRegisterBit(kRegTapAxes, 1, state);
}

bool I2cInterface::getTapDetectionY(void)
{
	return readRegisterBit(kRegTapAxes, 1);
}

void I2cInterface::setTapDetectionZ(bool state)
{
	writeRegisterBit(kRegTapAxes, 0, state);
}

bool I2cInterface::getTapDetectionZ(void)
{
	return readRegisterBit(kRegTapAxes, 0);
}

void I2cInterface::setTapDetectionXYZ(bool state)
{
	uint8_t value;
	value = readRegister8(kRegTapAxes);
	if (state) {
		value |= 0b00000111;
	} else {
		value &= 0b11111000;
	}
	writeRegister8(kRegTapAxes, value);
}


void I2cInterface::useInterrupt(int_t interrupt)
{
	if (interrupt == 0) {
		writeRegister8(kRegIntMap, 0x00);
	} else {
		writeRegister8(kRegIntMap, 0xFF);
	}
	writeRegister8(kRegIntEnable, 0xFF);
}

Activites I2cInterface::readActivites(void)
{
	Activites a;

	uint8_t data = readRegister8(kRegIntSource);

	a.isOverrun = (data & kOverrun);
	a.isWatermark = (data & kWatermark);
	a.isFreeFall = (data & kFreeFall);
	a.isInactivity = (data & kInactivity);
	a.isActivity = (data & kActivity);
	a.isDoubleTap = (data & kDoubleTap);
	a.isTap = (data & kSingleTap);
	a.isDataReady = (data & kDataReady);

	data = readRegister8(kRegActTapStatus);

	a.isActivityOnX = (data & (6 << 1));
	a.isActivityOnY = (data & (5 << 1));
	a.isActivityOnZ = (data & (4 << 1));
	a.isTapOnX = (data & (2 << 1));
	a.isTapOnY = (data & (1 << 1));
	a.isTapOnZ = (data & (0 << 1));

	return a;
}

/**
 *  Write byte to register
 */
void I2cInterface::writeRegister8(uint8_t reg, uint8_t value)
{
	Wire.beginTransmission(kAddress);
	#if ARDUINO >= 100
		Wire.write(reg);
		Wire.write(value);
	#else
		Wire.send(reg);
		Wire.send(value);
	#endif
	Wire.endTransmission();
}

/**
 *  Read byte to register
 */
uint8_t I2cInterface::fastRegister8(uint8_t reg)
{
	uint8_t value;
	Wire.beginTransmission(kAddress);
	#if ARDUINO >= 100
		Wire.write(reg);
	#else
		Wire.send(reg);
	#endif
	Wire.endTransmission();

	Wire.requestFrom(kAddress, uint8_t(1));
	#if ARDUINO >= 100
		value = Wire.read();
	#else
		value = Wire.receive();
	#endif
	Wire.endTransmission();

	return value;
}

/**
 *  Read byte from register
 */
uint8_t I2cInterface::readRegister8(uint8_t reg)
{
	uint8_t value;
	Wire.beginTransmission(kAddress);
	#if ARDUINO >= 100
		Wire.write(reg);
	#else
		Wire.send(reg);
	#endif
	Wire.endTransmission();

	Wire.beginTransmission(kAddress);
	Wire.requestFrom(kAddress, uint8_t(1));
	while(!Wire.available()) {};
	#if ARDUINO >= 100
		value = Wire.read();
	#else
		value = Wire.receive();
	#endif
	Wire.endTransmission();

	return value;
}

/**
 *  Read word from register
 */
int16_t I2cInterface::readRegister16(uint8_t reg)
{
	int16_t value;
	Wire.beginTransmission(kAddress);
	#if ARDUINO >= 100
		Wire.write(reg);
	#else
		Wire.send(reg);
	#endif
	Wire.endTransmission();

	Wire.beginTransmission(kAddress);
	Wire.requestFrom(kAddress, uint8_t(2));
	while(!Wire.available()) {};
	#if ARDUINO >= 100
		uint8_t vla = Wire.read();
		uint8_t vha = Wire.read();
	#else
		uint8_t vla = Wire.receive();
		uint8_t vha = Wire.receive();
	#endif
	Wire.endTransmission();

	value = vha << 8 | vla;

	return value;
}

void I2cInterface::writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
	uint8_t value;
	value = readRegister8(reg);
	if (state) {
		value |= (1 << pos);
	} else {
		value &= ~(1 << pos);
	}
	writeRegister8(reg, value);
}

bool I2cInterface::readRegisterBit(uint8_t reg, uint8_t pos)
{
	uint8_t value;
	value = readRegister8(reg);
	return ((value >> pos) & 1);
}

}