# uDMA

**Deprecated! Merged into [core](https://github.com/trevor-makes/core)**

uDMA is an API for interfacing an Arduino Micro with SRAM over an 8-bit parallel bus. This is intended for an 8-bit homebrew computer where the Arduino is paired with an 8-bit CPU such as a Zilog Z80. In this configuration, the Arduino can be used to load code into SRAM, which can subsequently be run on the CPU.

Presently, the pin configuration is hard-coded (for Arduino Micro) and the address bus is limited to 10-bits, but this may be made generic in the future.

This package is intended to be used with [PlatformIO](https://platformio.org/), but the source files can be manually copied into a project when using the Arduino IDE.

To add uDMA to an existing PlatformIO project, modify the `platformio.ini` configuration file as follows:

```
lib_deps =
    https://github.com/trevor-makes/uDMA.git
```

uDMA is distributed under the [MIT license](LICENSE.txt)

## Usage

uDMA should be initialized by calling `setup()`.

The Arduino's connection to the bus can be set to either read, write, or floating by calling `enable_dma_read()`, `enable_dma_write()`, `disable_dma()`. The floating/disabled state should be used when another device (i.e. a CPU) is actively using the bus.

A single byte of data can be read/written using `read_byte()` and `write_byte()`:

```
enable_dma_write();
write_byte(address, data);

enable_dma_read();
byte data_in = read_byte(address);

disable_dma();
```

The `read_string()` and `write_string()` functions work similarly for null-terminated strings:

```
enable_dma_write();
write_string(address, "Hello World");

enable_dma_read();
char message[64]; // memory to read into; may be any size, but using 64 bytes here
read_string(address, message); // reads up to 64 characters, including null-terminator
Serial.println(message);

disable_dma();
```

## Contributors

[Trevor Makes](mailto:the.trevor.makes@gmail.com)
