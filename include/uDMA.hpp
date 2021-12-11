// Copyright (c) 2021 Trevor Makes

#pragma once

#include "uIO.hpp"

#include <stdint.h>

namespace uDMA {

template <typename ADDRESS, typename DATA, typename CHIP_SELECT, typename WRITE_ENABLE>
struct Bus {
  static void enable_write() {
    ADDRESS::Write::enable_write();
    CHIP_SELECT::Write::enable_write();
    WRITE_ENABLE::Write::enable_write();
    DATA::Write::enable_write();
  }

  static void enable_read() {
    ADDRESS::Write::enable_write();
    CHIP_SELECT::Write::enable_write();
    WRITE_ENABLE::Write::enable_write();
    DATA::Read::disable_pullups();
    DATA::Read::enable_read();
  }

  static void disable_bus() {
    ADDRESS::Read::disable_pullups();
    ADDRESS::Read::enable_read();
    CHIP_SELECT::Read::enable_pullups();
    CHIP_SELECT::Read::enable_read();
    WRITE_ENABLE::Read::enable_pullups();
    WRITE_ENABLE::Read::enable_read();
    DATA::Read::disable_pullups();
    DATA::Read::enable_read();
  }

  static void write_data(uint8_t data) {
    // Enable (clear) CS and WE
    CHIP_SELECT::Write::clear();
    WRITE_ENABLE::Write::clear();

    DATA::Write::write(data);

    // Disable (set) CS and WE
    WRITE_ENABLE::Write::set();
    CHIP_SELECT::Write::set();
  }

  static uint8_t read_data() {
    // Enable (clear) CS
    CHIP_SELECT::Write::clear();

    // Must wait 2 cycles (>70 ns) after chip select before reading data
    __asm__("nop");
    __asm__("nop");
    const uint8_t data = DATA::Read::read();

    // Disable (set) CS
    CHIP_SELECT::Write::set();
    return data;
  }

  static void write_byte(uint16_t addr, uint8_t data) {
    ADDRESS::Write::write(addr);
    write_data(data);
  }

  static uint8_t read_byte(uint16_t addr) {
    ADDRESS::Write::write(addr);
    return read_data();
  }

  static void write_string(uint16_t addr, const char* string) {
    for (;;) {
      const uint8_t data = *string++;
      ADDRESS::Write::write(addr++);
      write_data(data);
      if (data == 0)
        break;
    }
  }

  static void read_string(uint16_t addr, char* string, uint8_t max_len) {
    for (uint8_t i = 0; i < max_len; ++i) {
      ADDRESS::Write::write(addr + i);
      const uint8_t data = read_data();
      string[i] = data;
      if (data == 0)
        break;
    }
  }

  // Read bytes to buffer until null terminator or end of buffer
  template <uint8_t L>
  static void read_string(uint16_t addr, char (&buffer)[L]) {
      read_string(addr, buffer, L);
  }
};

// Perform initial configuration
void setup();

// Holds the Z80 in reset if true, otherwise resumes
void force_reset(bool enable);

// Resumes the clock output if true, otherwise pauses
void enable_clock(bool enable);

// Read the HALT signal from the Z80
bool is_halted();

// Grant exclusive access to read from the bus
void enable_read();

// Grant exclusive access to write to the bus
void enable_write();

// Release exclusive access to the bus
void disable_dma();

// Write single byte to memory address
void write_byte(uint16_t addr, uint8_t data);

// Read single byte from memory address
uint8_t read_byte(uint16_t addr);

// Write bytes from string until null terminator
void write_string(uint16_t addr, const char* string);

// Read bytes to string until null terminator or max_len bytes
void read_string(uint16_t addr, char* string, uint8_t max_len);

// Read bytes to buffer until null terminator or end of buffer
template <uint8_t L>
void read_string(uint16_t addr, char (&buffer)[L]) {
    read_string(addr, buffer, L);
}

} // namespace uDMA
