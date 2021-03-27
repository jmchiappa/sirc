/*
  sirc.cpp - sirc library v1.0.0 - 2021-03-27
  Copyright (c) 2021 Jean-Marc Chiappa.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

// sirc Protocol decoder Arduino code

#include "sirc.h"

enum etat {
  BEGIN_START,
  END_START,
  GET_PULSE
};

/**
  Debug helper
*/

//#define DEBUG

#ifdef DEBUG

  typedef struct elm {
    uint16_t time;
    uint8_t sirc_state;
    uint8_t nb_j;
    uint8_t pinstate;
    char comment[10];
  };  

  elm buffer[1024];
  uint32_t ptrpusher=0;
  uint32_t ptrpuller=0;

  #define Debug_print(a,b)      {Serial.print(a);Serial.print(" : ");Serial.print(b);}
  #define Debug_println(a,b)    {Serial.print(a);Serial.print(" : ");Serial.println(b);}

  inline void pushtrace(uint16_t time,uint8_t state,uint8_t j,uint8_t pin, char *comment){
      buffer[ptrpusher] = {           
        .time = time,                   
        .sirc_state = state,             
        .nb_j = j,                      
        .pinstate = digitalRead(PIN_RECEIVER)
      };
      strcpy(buffer[ptrpusher].comment,comment);
      ptrpusher++;
      ptrpusher = ptrpusher % 1024;     
  }

  inline void poptrace(void) {
    if(ptrpusher!=ptrpuller) {
      Debug_print("time", buffer[ptrpuller].time);
      Debug_print("\t", buffer[ptrpuller].comment);
      Debug_print("\tsirc state", buffer[ptrpuller].sirc_state);
      Debug_print("\tj", buffer[ptrpuller].nb_j);
      Debug_println("\tpin state", buffer[ptrpuller].pinstate);
      ptrpuller++;
      ptrpuller = ptrpuller % 1024;
    }
  }

#else

  #define Debug_print(a,b)      {}
  #define Debug_println(a,b)    {}
  inline void pushtrace(uint16_t time,uint8_t state,uint8_t j,uint8_t pin, char *comment){}
  inline void poptrace()        {}

#endif

/**
 Local variables shared with ISR
*/
  
uint8_t j;
HardwareTimer *tim;
boolean sirc_ok;
uint32_t timer_value;
uint8_t sirc_state=0;
uint32_t sirc_code;
uint8_t pinReceiver;

sirc::sirc(uint8_t pin_receiver, TIM_TypeDef *instance){
  pinReceiver = pin_receiver;
  htimer = instance;
};


void sirc::begin() {
  if(htimer==NULL) {
    htimer = TIM6;
  }
  tim = new HardwareTimer(htimer);
  tim->setPrescaleFactor(8);
  tim->setOverflow(3000,MICROSEC_FORMAT); 
  tim->pause();
  tim->setCount(0);
  tim->attachInterrupt(overflow);
  attachInterrupt(pinReceiver, sirc_read, CHANGE);          // Enable external interrupt (INT0)
}

void sirc_read() {
  if(!sirc_ok){
    // if(sirc_state != BEGIN_START){
      timer_value = tim->getCount(MICROSEC_FORMAT);                         // Store Timer1 value
      tim->setCount(0);
      // pushtrace(timer_value,sirc_state,j,0xFF,"ITEntry");
    // }
  // #if 0
    switch(sirc_state){
     case BEGIN_START :
        tim->setCount(0);
        j = 0;
        pushtrace(tim->getCount(),sirc_state,j,0xFF,"start");
        tim->resume();
        sirc_state = END_START;                               // Next state: end of mid1
        sirc_code=0;
        break;
     case END_START :                                      // End of mid1 ==> check if we're at the beginning of start1 or mid0
      if(timer_value < start_min){         // Invalid interval ==> stop decoding and reset
        sirc_state = 0;                             // Reset decoding process
        tim->pause();
        pushtrace(timer_value,sirc_state,j,0xFF,"Err-Start");
        break;
      }
      pushtrace(timer_value,sirc_state,j,1,"End-Start");
      sirc_state = GET_PULSE;
      break;
     case GET_PULSE :                                      // End of mid0 ==> check if we're at the beginning of start0 or mid1
      if( digitalRead(pinReceiver) ) {
        if((timer_value > high_max) || (timer_value < low_min)){
          sirc_state = 0;                             // Reset decoding process
          tim->pause();
          pushtrace(timer_value,sirc_state,j,0xFF,"ErrTiming");
          break;
        }
        uint8_t state = (timer_value<low_max ? 0 : 1);
        sirc_code |= state<<j;
        pushtrace(timer_value,sirc_code,j,state,"pulse");
        j++;
      }
      break;
    }
  }
}

static void overflow() {
  pushtrace(tim->getCount(MICROSEC_FORMAT),0,0,0xFF,"overflow");  
  sirc_state = 0;                                 // Reset decoding process
  tim->pause();
  if(j==12 || j==15 || j==20) {
    sirc_ok=1;
  }
} 

boolean sirc::codeReceived() {
  return sirc_ok!=0;
}

uint16_t sirc::rawCode() {
  update();
  sirc_ok = 0;
  return sirc_code;
}

uint8_t sirc::Command() {
  update();
  sirc_ok = 0;
  return command;
}

uint8_t sirc::Address() {
  update();
  return address;
};

void sirc::skipThisCode(){
  tim->pause();
  sirc_ok=0;
  sirc_state = 0;
};

void sirc::update() {
  poptrace();
  if(sirc_ok){                                    // If the mcu receives sirc message with successful
    sirc_state = 0;
    tim->pause();
    switch(j) {
      case 12:
        address = (sirc_code >> 7) & 0x1F;            // Next 5 bits are for address
        break;
      case 15:
      case 20:
        address = (sirc_code >> 7) & 0xFF;            // Next 5 bits are for address
        break;
      default:
        Debug_println("error","unknown format");
    }
    command = sirc_code & 0x7F;                   // The 6 LSBits are command bits
  }
}
