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
#ifndef __GUARD_SAMD10_H__
#define __GUARD_SAMD10_H__

#include <Arduino.h>
#include <Wire.h>

enum samd10_results {
  SAMD10_OK,
  SAMD10_UNALIGNED_ADDRESS,
};

enum samd10_led_states {
  LED_ON,
  LED_OFF,
  LED_TOGGLE
};

class Samd10Class
{
protected:
 public:
  Samd10Class() { };
  void  begin(void);
#if defined(ARDUINO_ARCH_ESP8266)
  void  begin(uint8_t sda, uint8_t scl);
#endif
  int   writeRegister(uint32_t reg, uint32_t data);
  int   readRegister(uint32_t reg, uint32_t *result);
  int   executeFrom(uint32_t address);
  void  setLed(uint8_t state);
  boolean pinMode(uint8_t pin, uint8_t mode);
  boolean digitalWrite(uint8_t pin, boolean value);
  boolean digitalRead(uint8_t pin);

 private:
  void  write32(uint32_t val);
};

extern Samd10Class Samd10;

#endif //ifndef __GUARD_SAMD10_H__

// EOF
