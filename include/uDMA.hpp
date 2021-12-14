// Copyright (c) 2021 Trevor Makes

#pragma once

#include "uIO.hpp"

#include <stdint.h>

namespace uDMA {

template <typename ADDRESS, typename DATA, typename CHIP_SELECT, typename WRITE_ENABLE>
struct Bus {
  // Configure ports for writing to memory
  static void config_write() {
    ADDRESS::config_output();
    CHIP_SELECT::config_output();
    WRITE_ENABLE::config_output();
    DATA::config_output();
  }

  // Configure ports for reading from memory
  static void config_read() {
    ADDRESS::config_output();
    CHIP_SELECT::config_output();
    WRITE_ENABLE::config_output();
    DATA::config_input();
  }

  // Configure ports for high impedance (allow external device to access memory)
  static void config_high_impedance() {
    ADDRESS::config_input();
    CHIP_SELECT::config_input_pullups();
    WRITE_ENABLE::config_input_pullups();
    DATA::config_input();
  }

  static void enable_chip_select() {
    CHIP_SELECT::clear();
  }

  static void disable_chip_select() {
    CHIP_SELECT::set();
  }

  static void enable_write_enable() {
    WRITE_ENABLE::clear();
  }

  static void disable_write_enable() {
    WRITE_ENABLE::set();
  }

  static void write_data(uint8_t data) {
    enable_chip_select();
    enable_write_enable();

    DATA::write(data);

    disable_write_enable();
    disable_chip_select();
  }

  static uint8_t read_data() {
    enable_chip_select();

    // Must wait 2 cycles (>70 ns) after chip select before reading data
    __asm__("nop");
    __asm__("nop");
    const uint8_t data = DATA::read();

    disable_chip_select();
    return data;
  }

  static void write_byte(uint16_t addr, uint8_t data) {
    ADDRESS::write(addr);
    write_data(data);
  }

  static uint8_t read_byte(uint16_t addr) {
    ADDRESS::write(addr);
    return read_data();
  }

  // TODO test address MSB optimization
  static void write_string(uint16_t addr, const char* string) {
    //if ((addr & 0xFF) != 0)
    //  ADDRESS::MSB::write(addr / 0x100);
    for (;;) {
      ADDRESS::write(addr++);
      //if ((addr & 0xFF) == 0)
      //  ADDRESS::MSB::write(addr / 0x100);
      //ADDRESS::LSB::write(addr++)
      const uint8_t data = *string++;
      write_data(data);
      if (data == 0)
        break;
    }
  }

  // TODO test address MSB optimization
  static void read_string(uint16_t addr, char* string, uint8_t max_len) {
    //if ((addr & 0xFF) != 0)
    //  ADDRESS::MSB::write(addr / 0x100);
    for (uint8_t i = 0; i < max_len; ++i) {
      ADDRESS::write(addr + i);
      //if ((addr + i & 0xFF) == 0)
      //  ADDRESS::MSB::write((addr + i) / 0x100);
      //ADDRESS::LSB::write(addr + i);
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
