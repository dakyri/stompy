/**
 * I2cInterface.cpp - Useful I2c wrapper for Wire
 */

#include "I2cInterface.h"

#include <Wire.h>

bool I2cInterface::isWireBegun = false;

void I2cInterface::begin()
{
  if (!isWireBegun) {
    Wire.begin();
    isWireBegun = true;
  }
}

/**
 *  Write byte to register
 */
void I2cInterface::writeRegister8(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(address);
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
  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write(reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();

  Wire.requestFrom(address, uint8_t(1));
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
  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write(reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.requestFrom(address, uint8_t(1));
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
  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write(reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.requestFrom(address, uint8_t(2));
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


