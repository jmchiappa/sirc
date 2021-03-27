/*
  sirc.h - sirc library v1.0.0 - 2021-03-27
  Copyright (c) 2021 Jean-Marc Chiappa.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

// sirc Protocol decoder Arduino code
#ifndef sirc_h
#define sirc_h

#include "Arduino.h"
 
// Define number of Timer1 ticks (with a prescaler of 1/8)
#define low_min     300
#define low_max     900
#define high_min    900
#define high_max    1500
#define start_min   2200

static void sirc_read();
static void overflow();

class sirc
{
public:
  sirc(uint8_t pin_receiver, TIM_TypeDef *instance);
  void begin();
  boolean codeReceived();
  uint16_t rawCode();
  uint8_t Command();
  uint8_t Address();
  void skipThisCode();
private:
  void update(void);
  TIM_TypeDef *htimer=NULL;
  uint8_t pin;
  uint8_t address;
  uint8_t command;
};

#endif // sirc_h