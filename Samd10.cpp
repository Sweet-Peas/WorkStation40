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
#include "Samd10Defines.h"

#define SAMD10_ADDRESS                  ((uint8_t)0x12)

#define LED_PIN         PIN_PA02
#define LED_MASK        PORT_PA02

static uint8_t pinMap[] = {PIN_PA14, PIN_PA11, PIN_PA10, PIN_PA07,
                             PIN_PA06, PIN_PA05, PIN_PA04, PIN_PA03};

static uint32_t maskMap[] = {PORT_PA14, PORT_PA11, PORT_PA10, PORT_PA07,
                             PORT_PA06, PORT_PA05, PORT_PA04, PORT_PA03};

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

/***************************************************************************
 *
 * Executes code on the Workstation40.
 * This is implemented in the firmware by checking for the special address
 * 0xffffffff (which is normally an illegal address) and then jumping to
 * the specified address.
 *
 **************************************************************************/
int Samd10Class::executeFrom(uint32_t address)
{
#if defined(DEBUG)
  Serial.print(F("Executing code from: "));
  Serial.print(address, HEX);
#endif
  Wire.beginTransmission(SAMD10_ADDRESS);
  write32(0xffffffff);
  write32(address);
  Wire.endTransmission();

  return SAMD10_OK;
}

/***************************************************************************
 *
 * Sets the LED on the workstation board to the wanted state.
 *
 **************************************************************************/
void Samd10Class::setLed(uint8_t state)
{
  // First make sure the port pin is set in output mode
  writeRegister(REG_PORT_DIRSET0, LED_MASK);

  switch(state) {
    case LED_ON:
      writeRegister(REG_PORT_OUTCLR0, LED_MASK);
      break;
    case LED_OFF:
      writeRegister(REG_PORT_OUTSET0, LED_MASK);
      break;
    case LED_TOGGLE:
      writeRegister(REG_PORT_OUTTGL0, LED_MASK);
      break;
    default:
      break;
  }
}

/***************************************************************************
 *
 * Set the wanted pin in the desired mode. This function relies on the
 * standard Arduino pin modes. So you get functions like pull up etc.
 *
 **************************************************************************/
boolean Samd10Class::pinMode(uint8_t pin, uint8_t mode)
{
  // Just make sure the pin is valid
  if (pin >= 7) {
    return false;
  }

  switch(mode) {
    case INPUT:
      writeRegister(REG_PORT_DIRCLR0, maskMap[pin]);
      writeRegister(REG_PORT_OUTCLR0, maskMap[pin]);
      break;
    case INPUT_PULLUP:
      writeRegister(REG_PORT_DIRCLR0, maskMap[pin]);
      writeRegister(REG_PORT_OUTSET0, maskMap[pin]);
      break;
    case OUTPUT:
      writeRegister(REG_PORT_DIRSET0, maskMap[pin]);
      break;
    default:
      break;
  }
  return true;
}

/***************************************************************************
 *
 * Set or clear the wanted digital pin. If the pin is defined as an input
 * this function will enable/disable the pull up pin of the input pin.
 *
 **************************************************************************/
boolean Samd10Class::digitalWrite(uint8_t pin, boolean value)
{
  // Just make sure the pin is valid
  if (pin >= 7) {
    return false;
  }

  if (value) {
    writeRegister(REG_PORT_OUTSET0, maskMap[pin]);
  } else {
    writeRegister(REG_PORT_OUTCLR0, maskMap[pin]);
  }
}
/***************************************************************************
 *
 * Read the value of a digital input pin.
 *
 **************************************************************************/
boolean Samd10Class::digitalRead(uint8_t pin)
{
  uint32_t inp;

  // Just make sure the pin is valid
  if (pin >= 7) {
    return false;
  }

  readRegister(REG_PORT_IN0, &inp);

  return (inp & maskMap[pin] ? true : false);
}

// The global instance
Samd10Class Samd10;

