// https://github.com/trevor-makes/uDMA.git
// Copyright (c) 2021 Trevor Makes

#include "uDMA.hpp"

#include <Arduino.h>
#include <stdint.h>

#if !defined(ARDUINO_AVR_MICRO)
#error This sketch is intended for Arduino Micro (ATmega 32u4)
#endif

namespace uDMA {

// PORTB [D7 D6 D5 D4 D3 D2 D1 D0]
const uint8_t DATA_MASK = 0b11111111; // in/out

// PORTD [CS WE - - A3 A2 A1 A0]
const uint8_t ADDR_0_3_MASK = 0b00001111; // out
const uint8_t RESET_MASK = bit(4); // out, active low
const uint8_t WE_MASK = bit(6); // out, active low
const uint8_t CS_MASK = bit(7); // out, active low

// PORTE
const uint8_t HALT_MASK = bit(6); // in, active low

// PORTF [A7 A6 A5 A4 - - A9 A8]
const uint8_t ADDR_4_7_MASK = 0b11110000; // out
const uint8_t ADDR_8_9_MASK = 0b00000011; // out
const uint8_t ADDR_4_9_MASK = ADDR_4_7_MASK | ADDR_8_9_MASK;

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
    PORTD &= ~WE_MASK; //< set WE low
  } else {
    PORTD |= WE_MASK; //< set WE high
  }
}

inline void set_chip_select(bool enable) {
  if (enable) {
    PORTD &= ~CS_MASK; //< set CS low
  } else {
    PORTD |= CS_MASK; //< set CS high
  }
}

inline void write_address(uint16_t addr) {
  const uint8_t addr_0_3 = addr & 0x0F; //< low 4 bits for port D
  const uint8_t addr_4_7 = addr & 0xF0; //< high 4 bits for port F
  const uint8_t addr_8_9 = addr >> 8; //< 2 bits for port F
  PORTD = (PORTD & ~ADDR_0_3_MASK) | addr_0_3; //< don't touch WE/CS/RESET
  PORTF = addr_4_7 | addr_8_9; //< [7654--98]
}

void write_data(uint8_t data) {
  set_chip_select(true);
  set_write_enable(true);

  PORTB = data;

  set_write_enable(false);
  set_chip_select(false);
}

uint8_t read_data() {
  set_chip_select(true);

  // Must wait 2 cycles (>70 ns) after chip select before reading data
  __asm__("nop");
  __asm__("nop");
  const uint8_t data = PINB;

  set_chip_select(false);
  return data;
}

void enable_read() {
  // Set address and RESET/WE/CS to output
  DDRD = ADDR_0_3_MASK | RESET_MASK | WE_MASK | CS_MASK;
  DDRF = ADDR_4_9_MASK; //< set addr to output
  PORTB = 0; //< disable data pull-ups
  DDRB = 0; //< set data as input
}

void enable_write() {
  // Set address and RESET/WE/CS to output
  DDRD = ADDR_0_3_MASK | RESET_MASK | WE_MASK | CS_MASK;
  DDRF = ADDR_4_9_MASK; //< set addr to output
  DDRB = DATA_MASK; //< set data as output
}

void disable_dma() {
  // enable WE/CS pull-ups, leave addr high-Z
  PORTD = (PORTD & ~ADDR_0_3_MASK) | WE_MASK | CS_MASK;
  DDRD = RESET_MASK; //< set addr/WE/CS to input, RESET to output
  PORTF = 0; //< disable addr pull-ups
  DDRF = 0; //< set addr to input
  PORTB = 0; //< disable data pull-ups
  DDRB = 0; //< set data as input
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
