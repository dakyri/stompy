/**
 * ADXL345mw.h - Header file for stripped back access to ADXL345 on Arduino.
 */

#pragma once


#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#include "I2cInterface.h"

namespace adxl345 {

class Vector
{
public:
	Vector(int16_t _x=0, int16_t _y=0, int16_t _z=0, float n=1);
	void lowPassFilter(Vector vector, float alpha = 0.5);
	
	float x;
	float y;
	float z;
};

static const byte kAddress1 = 0x53;
static const byte kAddress2 = 0x1d;
static const byte kRegDevid = 0x00;
static const byte kRegThreshTap = 0x1D; // 1
static const byte kRegOfsX = 0x1E;
static const byte kRegOfsY = 0x1F;
static const byte kRegOfsZ = 0x20;
static const byte kRegDur = 0x21; // 2
static const byte kRegLatent = 0x22; // 3
static const byte kRegWindow = 0x23; // 4
static const byte kRegThreshAct = 0x24; // 5
static const byte kRegThreshInact = 0x25; // 6
static const byte kRegTimeInact = 0x26; // 7
static const byte kRegActInactCtl = 0x27;
static const byte kRegThreshFreefall = 0x28; // 8
static const byte kRegTimeFreefall = 0x29; // 9
static const byte kRegTapAxes = 0x2A;
static const byte kRegActTapStatus = 0x2B;
static const byte kRegActBwCtl = 0x2C;
static const byte kRegActPowerCtl = 0x2D;
static const byte kRegIntEnable = 0x2E;
static const byte kRegIntMap = 0x2F;
static const byte kRegIntSource = 0x30; // A
static const byte kRegDataFormat = 0x31;
static const byte kRegDataX0 = 0x32;
static const byte kRegDataX1 = 0x33;
static const byte kRegDataY0 = 0x34;
static const byte kRegDataY1 = 0x35;
static const byte kRegDataZ0 = 0x36;
static const byte kRegDataZ1 = 0x37;
static const byte kRegFifoCtl = 0x38;
static const byte kRegFifoStatus = 0x39;

static const float kGravitySun = 273.95f;
static const float kGravityEarth = 9.80665f;
static const float kGravityMoon = 1.622f;
static const float kGravityMars = 3.69f;
static const float kGravityNone = 1.00f;

enum dataRate_t
{
	kDatarate3200HZ	= 0b1111,
	kDatarate1600HZ	= 0b1110,
	kDatarate800HZ	= 0b1101,
	kDatarate400HZ	= 0b1100,
	kDatarate200HZ	= 0b1011,
	kDatarate100HZ	= 0b1010,
	kDatarate50HZ	= 0b1001,
	kDatarate25HZ	= 0b1000,
	kDatarate12_5HZ	= 0b0111,
	kDatarate6_25HZ	= 0b0110,
	kDatarate3_13HZ	= 0b0101,
	kDatarate1_56HZ	= 0b0100,
	kDatarate0_78HZ	= 0b0011,
	kDatarate0_39HZ	= 0b0010,
	kDatarate0_20HZ	= 0b0001,
	kDatarate0_10HZ	= 0b0000
};

enum int_t
{
	kInt2 = 0b01,
	kInt1 = 0b00
};

enum activity_t
{
	kDataReady	= 1 << 7,
	kSingleTap	= 1 << 6,
	kDoubleTap	= 1 << 5,
	kActivity	= 1 << 4,
	kInactivity	= 1 << 3,
	kFreeFall	= 1 << 2,
	kWatermark	= 1 << 1,
	kOverrun	= 1 << 0
};

enum range_t
{
	kRange16G	= 0b11,
	kRange8G	= 0b10,
	kRange4G	= 0b01,
	kRange2G	= 0b00
};

struct Activites
{
	bool isOverrun;
	bool isWatermark;
	bool isFreeFall;
	bool isInactivity;
	bool isActivity;
	bool isActivityOnX;
	bool isActivityOnY;
	bool isActivityOnZ;
	bool isDoubleTap;
	bool isTap;
	bool isTapOnX;
	bool isTapOnY;
	bool isTapOnZ;
	bool isDataReady;
};

class I2cInterface: protected ::I2cInterface
{
public:
	I2cInterface(byte p_address=kAddress1);

	bool begin(void);
	void clearSettings(void);

	void readRaw(int16_t &x, int16_t &y, int16_t &z);
	Vector readNormalize(float gravityFactor = kGravityEarth);
	Vector readScaled(void);

	Activites readActivites(void);

	void  setRange(range_t range);
	range_t getRange(void);

	void  setDataRate(dataRate_t dataRate);
	dataRate_t getDataRate(void);

	void setTapThreshold(float threshold);
	float getTapThreshold(void);

	void setTapDuration(float duration);
	float getTapDuration(void);

	void setDoubleTapLatency(float latency);
	float getDoubleTapLatency(void);

	void setDoubleTapWindow(float window);
	float getDoubleTapWindow(void);

	void setActivityThreshold(float threshold);
	float getActivityThreshold(void);

	void setInactivityThreshold(float threshold);
	float getInactivityThreshold(void);

	void setTimeInactivity(uint8_t time);
	uint8_t getTimeInactivity(void);

	void setFreeFallThreshold(float threshold);
	float getFreeFallThreshold(void);

	void setFreeFallDuration(float duration);
	float getFreeFallDuration();

	void setActivityX(bool state);
	bool getActivityX(void);
	void setActivityY(bool state);
	bool getActivityY(void);
	void setActivityZ(bool state);
	bool getActivityZ(void);
	void setActivityXYZ(bool state);

	void setInactivityX(bool state);
	bool getInactivityX(void);
	void setInactivityY(bool state);
	bool getInactivityY(void);
	void setInactivityZ(bool state);
	bool getInactivityZ(void);
	void setInactivityXYZ(bool state);

	void setTapDetectionX(bool state);
	bool getTapDetectionX(void);
	void setTapDetectionY(bool state);
	bool getTapDetectionY(void);
	void setTapDetectionZ(bool state);
	bool getTapDetectionZ(void);
	void setTapDetectionXYZ(bool state);

	void useInterrupt(int_t interrupt);
	
	bool isValid;
};


}
