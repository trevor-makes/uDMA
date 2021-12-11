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

// PORTD [CS WE - R A3 A2 A1 A0]
using Address0123 = uIO::PortD::Mask<0x0F>;
const uint8_t RESET_MASK = bit(4); // out, active low
using WriteEnable = uIO::PinD6;
using ChipSelect = uIO::PinD7;

// PORTE
const uint8_t HALT_MASK = bit(6); // in, active low

// PORTF [A7 A6 A5 A4 - - A9 A8]
using Address4567 = uIO::PortF::Mask<0xF0>;
using Address89 = uIO::PortF::Mask<0x03>;

using AddressLSB = uIO::PortSplitter<Address0123, Address4567>;
using Address = uIO::Port16<AddressLSB, Address89>;

inline void configure_clock() {
  DDRC |= bit(6); //< set PC6 (OC3A) as output
  TCCR3B = bit(CS30) | bit(WGM32); //< no prescaling, CTC
  OCR3A = 1; // 4 MHz
}

inline void configure_halt() {
  // Set halt to active low input with pullup
  DDRE &= ~HALT_MASK;
  PORTE |= HALT_MASK;
}

void setup() {
  force_reset(true);
  configure_clock();
  configure_halt();
  disable_dma();
}

inline void force_reset(bool enable) {
  DDRD |= RESET_MASK;
  if (enable) {
    PORTD &= ~RESET_MASK;
  } else {
    PORTD |= RESET_MASK;
  }
}

inline void enable_clock(bool enable) {
  if (enable) {
    TCCR3A = bit(COM3A0); //< toggle OC3A on compare
  } else {
    TCCR3A = bit(COM3A1); //< clear OC3A on compare (hold clock pin low)
  }
}

inline bool is_halted() {
  return !(PINE & HALT_MASK);
}

inline void set_write_enable(bool enable) {
  if (enable) {
    WriteEnable::Write::clear();
  } else {
    WriteEnable::Write::set();
  }
}

inline void set_chip_select(bool enable) {
  if (enable) {
    ChipSelect::Write::clear();
  } else {
    ChipSelect::Write::set();
  }
}

inline void write_address(uint16_t addr) {
  Address::Write::write(addr);
}

void write_data(uint8_t data) {
  set_chip_select(true);
  set_write_enable(true);

  Data::Write::write(data);

  set_write_enable(false);
  set_chip_select(false);
}

uint8_t read_data() {
  set_chip_select(true);

  // Must wait 2 cycles (>70 ns) after chip select before reading data
  __asm__("nop");
  __asm__("nop");
  const uint8_t data = Data::Read::read();

  set_chip_select(false);
  return data;
}

void enable_read() {
  Address::Write::enable_write();
  WriteEnable::Write::enable_write();
  ChipSelect::Write::enable_write();
  Data::Read::disable_pullups();
  Data::Read::enable_read();
}

void enable_write() {
  Address::Write::enable_write();
  WriteEnable::Write::enable_write();
  ChipSelect::Write::enable_write();
  Data::Write::enable_write();
}

void disable_dma() {
  Address::Read::disable_pullups();
  Address::Read::enable_read();
  WriteEnable::Read::enable_pullups();
  WriteEnable::Read::enable_read();
  ChipSelect::Read::enable_pullups();
  ChipSelect::Read::enable_read();
  Data::Read::disable_pullups();
  Data::Read::enable_read();
}

void write_byte(uint16_t addr, uint8_t data) {
  write_address(addr);
  write_data(data);
}

uint8_t read_byte(uint16_t addr) {
  write_address(addr);
  return read_data();
}

void write_string(uint16_t addr, const char* string) {
  for (;;) {
    const uint8_t data = *string++;
    write_address(addr++);
    write_data(data);
    if (data == 0)
      break;
  }
}

void read_string(uint16_t addr, char* string, uint8_t max_len) {
  for (uint8_t i = 0; i < max_len; ++i) {
    write_address(addr + i);
    const uint8_t data = read_data();
    string[i] = data;
    if (data == 0)
      break;
  }
}

} // namespace uDMA
