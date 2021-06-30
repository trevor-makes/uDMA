// https://github.com/trevor-makes/uANSI.git
// Copyright (c) 2021 Trevor Makes

#pragma once

#include <stdint.h>

namespace uDMA {

// Perform initial configuration
void setup();

// Grant exclusive access to read from the bus
void enable_dma_read();

// Grant exclusive access to write to the bus
void enable_dma_write();

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
