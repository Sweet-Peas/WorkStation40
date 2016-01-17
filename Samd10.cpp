/*
 * ----------------------------------------------------------------------------
 *            _____ _           _                   _
 *           | ____| | ___  ___| |_ _ __ ___  _ __ (_) ___
 *           |  _| | |/ _ \/ __| __| '__/ _ \| '_ \| |/ __|
 *           | |___| |  __/ (__| |_| | | (_) | | | | | (__
 *           |_____|_|\___|\___|\__|_|  \___/|_| |_|_|\___|
 *            ____                   _   ____
 *           / ___|_      _____  ___| |_|  _ \ ___  __ _ ___
 *           \___ \ \ /\ / / _ \/ _ \ __| |_) / _ \/ _` / __|
 *            ___) \ V  V /  __/  __/ |_|  __/  __/ (_| \__ \
 *           |____/ \_/\_/ \___|\___|\__|_|   \___|\__,_|___/
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <pontus@sweetpeas.se> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return - Pontus Oldberg
 * ----------------------------------------------------------------------------
 */
#include <Arduino.h>
#include <Wire.h>
#include "Samd10.h"

#define SAMD10_ADDRESS                  ((uint8_t)0x12)

/***************************************************************************
 *
 *  Writes 32-bits to the specified destination register
 *
 **************************************************************************/
void Samd10Class::write32(uint32_t val)
{
  Wire.write((val >> 24) & 0xff);
  Wire.write((val >> 16) & 0xff);
  Wire.write((val >> 8) & 0xff);
  Wire.write(val & 0xff);
}

/***************************************************************************
 *
 * Begin method. This method must be called before using this library
 * either directly if the class is initializing the Wire library or by
 * calling this library's function begin(sda, scl) in which case that
 * function will call this one.
 *
 **************************************************************************/
void Samd10Class::begin(void) { }

#if defined(ARDUINO_ARCH_ESP8266)
/***************************************************************************
 *
 * Convenience method for ESP8266 systems such as the ESP210.
 *
 **************************************************************************/
void Samd10Class::begin(uint8_t sda, uint8_t scl)
{
  Wire.begin(sda, scl);
  begin();
}
#endif

/***************************************************************************
 *
 * Writes a 32 bit word to WorkStation40
 *
 **************************************************************************/
int Samd10Class::writeRegister(uint32_t reg, uint32_t data)
{
#if defined(DEBUG)
  Serial.print(F("Writing to register: "));
  Serial.print(reg, HEX);
  Serial.print(F(", data: "));
  Serial.println(val, HEX);
#endif
  // Make sure it is a 32 bit aligned address
  if (reg & 3) {
    return SAMD10_UNALIGNED_ADDRESS;
  }

  // Write data
  Wire.beginTransmission(SAMD10_ADDRESS);
  write32(reg);
  write32(data);
  Wire.endTransmission();

  return SAMD10_OK;
}


/***************************************************************************
 *
 * Reads a 32 bit register from the WorkStation40
 *
 **************************************************************************/
int Samd10Class::readRegister(uint32_t reg, uint32_t *result)
{
  uint32_t value = 0;
#if defined(DEBUG)
  Serial.print(F("Reading from register: "));
  Serial.print(reg, HEX);
#endif
  // Make sure it is a 32 bit aligned address
  if (reg & 3) {
    return SAMD10_UNALIGNED_ADDRESS;
  }

  Wire.beginTransmission(SAMD10_ADDRESS);
  write32(reg);
  Wire.endTransmission();
  Wire.requestFrom(SAMD10_ADDRESS, (uint8_t)4);
  while (Wire.available() != 4);
  for (int i=0;i<4;i++) {
    value = (value << 8) | Wire.read();
  }
#if defined(DEBUG)
  Serial.print(F(", data: "));
  Serial.println(value, HEX);
#endif
  *result = value;

  return SAMD10_OK;
}

Samd10Class Samd10;

