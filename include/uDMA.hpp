// Copyright (c) 2021 Trevor Makes

#pragma once

#include "uIO.hpp"

#include <stdint.h>

namespace uDMA {

template <typename CHIP_SELECT, typename WRITE_ENABLE>
struct Control {
  // Configure control lines for writing to memory
  static void config_write() {
    CHIP_SELECT::config_output();
    WRITE_ENABLE::config_output();
  }

  // Configure control lines for reading from memory
  static void config_read() {
    CHIP_SELECT::config_output();
    WRITE_ENABLE::config_output();
  }

  // Configure control lines for external control
  static void config_external() {
    CHIP_SELECT::config_input_pullups();
    WRITE_ENABLE::config_input_pullups();
  }

  // Set control lines for start of write sequence
  static void begin_write() {
    CHIP_SELECT::clear();
    WRITE_ENABLE::clear();
  }

  // Set control lines for end of write sequence
  static void end_write() {
    CHIP_SELECT::set();
    WRITE_ENABLE::set();
  }

  // Set control lines for start of read sequence
  static void begin_read() {
    CHIP_SELECT::clear();
  }

  // Set control lines for end of read sequence
  static void end_read() {
    CHIP_SELECT::set();
  }
};

template <typename ADDRESS, typename DATA, typename CONTROL>
struct Bus {
  // Configure ports for writing to memory
  static void config_write() {
    ADDRESS::config_output();
    DATA::config_output();
    CONTROL::config_write();
  }

  // Configure ports for reading from memory
  static void config_read() {
    ADDRESS::config_output();
    DATA::config_input();
    CONTROL::config_read();
  }

  // Configure ports for external control
  static void config_external() {
    ADDRESS::config_input();
    DATA::config_input();
    CONTROL::config_external();
  }

  static void write_byte(uint16_t addr, uint8_t data) {
    ADDRESS::write(addr);
    CONTROL::begin_write();
    DATA::write(data);
    CONTROL::end_write();
  }

  static uint8_t read_byte(uint16_t addr) {
    ADDRESS::write(addr);
    CONTROL::begin_read();
    const uint8_t data = DATA::read();
    CONTROL::end_read();
    return data;
  }

  static void write_string(uint16_t addr, const char* string) {
    for (;;) {
      const uint8_t data = *string++;
      write_byte(addr++, data);
      if (data == 0)
        break;
    }
  }

  static void read_string(uint16_t addr, char* string, uint8_t max_len) {
    for (uint8_t i = 0; i < max_len; ++i) {
      const uint8_t data = read_byte(addr + i);
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

} // namespace uDMA
