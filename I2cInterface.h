/**
 * I2cInterface.h - Useful I2c wrapper for Wire
 */

#pragma once

#include "Arduino.h"

class I2cInterface {
public:
	I2cInterface(const byte p_address): address(p_address) {}

	void begin(void);

	void writeRegister8(uint8_t reg, uint8_t value);
	uint8_t readRegister8(uint8_t reg);
	uint8_t fastRegister8(uint8_t reg);
	int16_t readRegister16(uint8_t reg);
	void writeRegisterBit(uint8_t reg, uint8_t pos, bool state);
	bool readRegisterBit(uint8_t reg, uint8_t pos);

	const byte address;
private:
	static bool isWireBegun;
};


