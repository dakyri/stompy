/*
 * ADXL345mw.cpp - Class file for stripped back access to ADXL345 on Arduino.
 */
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

Interface::Interface(byte p_address): io(p_address), isValid(false) {
	
}

bool Interface::begin()
{
  io.begin();
	// Check ADXL345 REG DEVID
	if (io.fastRegister8(kRegDevid) != 0xE5) {
		isValid = false;
		return isValid;
	}
	isValid = true;
	// Enable measurement mode (0b00001000)
	io.writeRegister8(kRegActPowerCtl, 0x08);
	clearSettings();
	return true;
}

/**
 *  Set Range
 */
void Interface::setRange(range_t range)
{
	// Get actual value register
	uint8_t value = io.readRegister8(kRegDataFormat);
	
	// Update the data rate
	// (&) 0b11110000 (0xF0 - Leave HSB)
	// (|) 0b0000xx?? (range - Set range)
	// (|) 0b00001000 (0x08 - Set Full Res)
	value &= 0xF0;
	value |= range;
	value |= 0x08;
	
	io.writeRegister8(kRegDataFormat, value);
}

/**
 *  Get Range
 */
range_t Interface::getRange(void)
{
	return (range_t)(io.readRegister8(kRegDataFormat) & 0x03);
}

/**
 *  Set Data Rate
 */
void Interface::setDataRate(dataRate_t dataRate)
{
	io.writeRegister8(kRegActBwCtl, dataRate);
}

/**
 *  Get Data Rate
 */
dataRate_t Interface::getDataRate(void)
{
	return (dataRate_t)(io.readRegister8(kRegActBwCtl) & 0x0F);
}


/**
 *  Read raw values
 */
void Interface::readRaw(int16_t &x, int16_t &y, int16_t &z)
{
	x = io.readRegister16(kRegDataX0);
	y = io.readRegister16(kRegDataY0);
	z = io.readRegister16(kRegDataZ0);
}

/**
 *  Read normalized values
 */
Vector Interface::readNormalize(float gravityFactor)
{
	int16_t x, y, z;
	readRaw(x, y, z);
	return Vector(x, y, z, 0.004f * gravityFactor);
}

/**
 *  Read scaled values
 */
Vector Interface::readScaled(void)
{
	int16_t x, y, z;
	readRaw(x, y, z);
	return Vector(x, y, z, 0.004f);
}

void Interface::clearSettings(void)
{
	setRange(kRange2G);
	setDataRate(kDatarate100HZ);

	io.writeRegister8(kRegThreshTap, 0x00);
	io.writeRegister8(kRegDur, 0x00);
	io.writeRegister8(kRegLatent, 0x00);
	io.writeRegister8(kRegWindow, 0x00);
	io.writeRegister8(kRegThreshAct, 0x00);
	io.writeRegister8(kRegThreshInact, 0x00);
	io.writeRegister8(kRegTimeInact, 0x00);
	io.writeRegister8(kRegThreshFreefall, 0x00);
	io.writeRegister8(kRegTimeFreefall, 0x00);

	uint8_t value;

	value = io.readRegister8(kRegActInactCtl);
	value &= 0b10001000;
	io.writeRegister8(kRegActInactCtl, value);

	value = io.readRegister8(kRegTapAxes);
	value &= 0b11111000;
	io.writeRegister8(kRegTapAxes, value);
}

/**
 *  Set Tap Threshold (62.5mg / LSB)
 */
void Interface::setTapThreshold(float threshold)
{
	uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
	io.writeRegister8(kRegThreshTap, scaled);
}

/**
 *  Get Tap Threshold (62.5mg / LSB)
 */
float Interface::getTapThreshold(void)
{
	return io.readRegister8(kRegThreshTap) * 0.0625f;
}

/**
 *  Set Tap Duration (625us / LSB)
 */
void Interface::setTapDuration(float duration)
{
	uint8_t scaled = constrain(duration / 0.000625f, 0, 255);
	io.writeRegister8(kRegDur, scaled);
}

/**
 *  Get Tap Duration (625us / LSB)
 */
float Interface::getTapDuration(void)
{
	return io.readRegister8(kRegDur) * 0.000625f;
}

/**
 *  Set Double Tap Latency (1.25ms / LSB)
 */
void Interface::setDoubleTapLatency(float latency)
{
	uint8_t scaled = constrain(latency / 0.00125f, 0, 255);
	io.writeRegister8(kRegLatent, scaled);
}

/**
 *  Get Double Tap Latency (1.25ms / LSB)
 */
float Interface::getDoubleTapLatency()
{
	return io.readRegister8(kRegLatent) * 0.00125f;
}

/**
 *  Set Double Tap Window (1.25ms / LSB)
 */
void Interface::setDoubleTapWindow(float window)
{
	uint8_t scaled = constrain(window / 0.00125f, 0, 255);
	io.writeRegister8(kRegWindow, scaled);
}

/**
 *  Get Double Tap Window (1.25ms / LSB)
 */
float Interface::getDoubleTapWindow(void)
{
	return io.readRegister8(kRegWindow) * 0.00125f;
}

/**
 *  Set Activity Threshold (62.5mg / LSB)
 */
void Interface::setActivityThreshold(float threshold)
{
	uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
	io.writeRegister8(kRegThreshAct, scaled);
}

/**
 *  Get Activity Threshold (65.5mg / LSB)
 */
float Interface::getActivityThreshold(void)
{
	return io.readRegister8(kRegThreshAct) * 0.0625f;
}

/**
 *  Set Inactivity Threshold (65.5mg / LSB)
 */
void Interface::setInactivityThreshold(float threshold)
{
	uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
	io.writeRegister8(kRegThreshInact, scaled);
}

/**
 *  Get Incactivity Threshold (65.5mg / LSB)
 */
float Interface::getInactivityThreshold(void)
{
	return io.readRegister8(kRegThreshInact) * 0.0625f;
}

/**
 *  Set Inactivity Time (s / LSB)
 */
void Interface::setTimeInactivity(uint8_t time)
{
	io.writeRegister8(kRegTimeInact, time);
}

/**
 *  Get Inactivity Time (s / LSB)
 */
uint8_t Interface::getTimeInactivity(void)
{
	return io.readRegister8(kRegTimeInact);
}

/**
 *  Set Free Fall Threshold (65.5mg / LSB)
 */
void Interface::setFreeFallThreshold(float threshold)
{
	uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
	io.writeRegister8(kRegThreshFreefall, scaled);
}

/**
 *  Get Free Fall Threshold (65.5mg / LSB)
 */
float Interface::getFreeFallThreshold(void)
{
	return io.readRegister8(kRegThreshFreefall) * 0.0625f;
}

/**
 *  Set Free Fall Duratiom (5ms / LSB)
 */
void Interface::setFreeFallDuration(float duration)
{
	uint8_t scaled = constrain(duration / 0.005f, 0, 255);
	io.writeRegister8(kRegTimeFreefall, scaled);
}

/**
 *  Get Free Fall Duratiom
 */
float Interface::getFreeFallDuration()
{
	return io.readRegister8(kRegTimeFreefall) * 0.005f;
}

void Interface::setActivityX(bool state)
{
	io.writeRegisterBit(kRegActInactCtl, 6, state);
}

bool Interface::getActivityX(void)
{
	return io.readRegisterBit(kRegActInactCtl, 6);
}

void Interface::setActivityY(bool state)
{
	io.writeRegisterBit(kRegActInactCtl, 5, state);
}

bool Interface::getActivityY(void)
{
	return io.readRegisterBit(kRegActInactCtl, 5);
}

void Interface::setActivityZ(bool state)
{
	io.writeRegisterBit(kRegActInactCtl, 4, state);
}

bool Interface::getActivityZ(void)
{
	return io.readRegisterBit(kRegActInactCtl, 4);
}

void Interface::setActivityXYZ(bool state)
{
	uint8_t value;

	value = io.readRegister8(kRegActInactCtl);

	if (state) {
		value |= 0b00111000;
	} else {
		value &= 0b11000111;
	}

	io.writeRegister8(kRegActInactCtl, value);
}


void Interface::setInactivityX(bool state) 
{
	io.writeRegisterBit(kRegActInactCtl, 2, state);
}

bool Interface::getInactivityX(void)
{
	return io.readRegisterBit(kRegActInactCtl, 2);
}

void Interface::setInactivityY(bool state)
{
	io.writeRegisterBit(kRegActInactCtl, 1, state);
}

bool Interface::getInactivityY(void)
{
	return io.readRegisterBit(kRegActInactCtl, 1);
}

void Interface::setInactivityZ(bool state)
{
	io.writeRegisterBit(kRegActInactCtl, 0, state);
}

bool Interface::getInactivityZ(void)
{
	return io.readRegisterBit(kRegActInactCtl, 0);
}

void Interface::setInactivityXYZ(bool state)
{
	uint8_t value = io.readRegister8(kRegActInactCtl);
	if (state) {
		value |= 0b00000111;
	} else {
		value &= 0b11111000;
	}
	io.writeRegister8(kRegActInactCtl, value);
}

void Interface::setTapDetectionX(bool state)
{
	io.writeRegisterBit(kRegTapAxes, 2, state);
}

bool Interface::getTapDetectionX(void)
{
	return io.readRegisterBit(kRegTapAxes, 2);
}

void Interface::setTapDetectionY(bool state)
{
	io.writeRegisterBit(kRegTapAxes, 1, state);
}

bool Interface::getTapDetectionY(void)
{
	return io.readRegisterBit(kRegTapAxes, 1);
}

void Interface::setTapDetectionZ(bool state)
{
	io.writeRegisterBit(kRegTapAxes, 0, state);
}

bool Interface::getTapDetectionZ(void)
{
	return io.readRegisterBit(kRegTapAxes, 0);
}

void Interface::setTapDetectionXYZ(bool state)
{
	uint8_t value;
	value = io.readRegister8(kRegTapAxes);
	if (state) {
		value |= 0b00000111;
	} else {
		value &= 0b11111000;
	}
	io.writeRegister8(kRegTapAxes, value);
}


void Interface::useInterrupt(int_t interrupt)
{
	if (interrupt == 0) {
		io.writeRegister8(kRegIntMap, 0x00);
	} else {
		io.writeRegister8(kRegIntMap, 0xFF);
	}
	io.writeRegister8(kRegIntEnable, 0xFF);
}

Activites Interface::readActivites(void)
{
	Activites a;

	uint8_t data = io.readRegister8(kRegIntSource);

	a.isOverrun = (data & kOverrun);
	a.isWatermark = (data & kWatermark);
	a.isFreeFall = (data & kFreeFall);
	a.isInactivity = (data & kInactivity);
	a.isActivity = (data & kActivity);
	a.isDoubleTap = (data & kDoubleTap);
	a.isTap = (data & kSingleTap);
	a.isDataReady = (data & kDataReady);

	data = io.readRegister8(kRegActTapStatus);

	a.isActivityOnX = (data & (6 << 1));
	a.isActivityOnY = (data & (5 << 1));
	a.isActivityOnZ = (data & (4 << 1));
	a.isTapOnX = (data & (2 << 1));
	a.isTapOnY = (data & (1 << 1));
	a.isTapOnZ = (data & (0 << 1));

	return a;
}

}
