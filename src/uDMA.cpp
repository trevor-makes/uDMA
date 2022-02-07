// Copyright (c) 2021 Trevor Makes

#include "uDMA.hpp"
#include "uIO.hpp"

#include <Arduino.h>
#include <stdint.h>

#if !defined(ARDUINO_AVR_MICRO)
#error This sketch is intended for Arduino Micro (ATmega 32u4)
#endif

namespace uDMA {

// PORTB [D7 D6 D5 D4 D3 D2 D1 D0]
using Data = uIO::PortB;

// PORTC [* C - - - - - -]
using Clock = uIO::PinC6;

// PORTD [CS WE - R A3 A2 A1 A0]
using Address0123 = uIO::PortD::Mask<0b00001111>;
using Reset = uIO::PinD4;
using WriteEnable = uIO::PinD6;
using ChipSelect = uIO::PinD7;

// PORTE [- H - - - * - -]
using Halt = uIO::PinE6;

// PORTF [A7 A6 A5 A4 - - A9 A8]
using Address4567 = uIO::PortF::Mask<0b11110000>;
using Address89 = uIO::PortF::Mask<0b00000011>;

// 2114 SRAM requires 70 ns after chip select before reading data
template <typename DATA>
struct DelayRead : DATA {
  static uint8_t read() {
    // 2 NOP cycles at 16 MHz delays 125 ns
    __asm__("nop");
    __asm__("nop");
    return DATA::read();
  }
};

using TmpControl = Control<ChipSelect, WriteEnable>;
using Address = uIO::Port16<uIO::PortJoin<Address0123, Address4567>, Address89>;
using TmpBus = Bus<Address, DelayRead<Data>, TmpControl>;

inline void configure_clock() {
  Clock::config_output(); // DDRC |= bit(6); //< set PC6 (OC3A) as output
  TCCR3B = bit(CS30) | bit(WGM32); //< no prescaling, CTC
  OCR3A = 1; // 4 MHz
}

inline void configure_halt() {
  // Set halt to active low input with pullup
  Halt::config_input_pullups(); // DDRE &= ~HALT_MASK; PORTE |= HALT_MASK;
}

void setup() {
  force_reset(true);
  configure_clock();
  configure_halt();
}

void force_reset(bool enable) {
  Reset::config_output(); // DDRD |= RESET_MASK;
  if (enable) {
    Reset::clear(); // PORTD &= ~RESET_MASK;
  } else {
    Reset::set(); // PORTD |= RESET_MASK;
  }
}

void enable_clock(bool enable) {
  if (enable) {
    TCCR3A = bit(COM3A0); //< toggle OC3A on compare
  } else {
    TCCR3A = bit(COM3A1); //< clear OC3A on compare (hold clock pin low)
  }
}

bool is_halted() {
  return Halt::is_clear(); // !(PINE & HALT_MASK);
}

} // namespace uDMA
