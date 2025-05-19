# Raspberry Pi Pico TCLK Protocol Implementation (piClk)

This project implements the TCLK (Tevatron Clock) protocol for the Raspberry Pi Pico using the Programmable Input/Output (PIO) feature. It captures and timestamps input clock signals with a resolution of 50ns and implements the bi-phase (modified Manchester) encoding protocol as described in the TCLK_Paper.pdf.

## Features

- **TCLK Protocol Implementation**: An implementation of the Fermilab Tevatron Clock protocol
  - Uses bi-phase (modified Manchester) coding at 10 MHz
  - Transition at each bit cell boundary
  - Additional transition at mid-cell if bit is 1, no transition if bit is 0
  - Word length is 12 bits:
    * 1 start bit (always 0)
    * 8 data bits
    * 1 parity bit (odd parity)
    * 2 guaranteed '1's after each event
- **PIO-based Implementation**:
  - **clockIN**: Listens to an input clock signal and timestamps transitions with a resolution of 50ns
  - **clockDecode**: Processes the timestamps and decodes data according to the TCLK protocol
  - **clockEncode**: Encodes data according to the TCLK protocol and generates timestamps
  - **clockOUT**: Generates an output clock signal based on the encoded timestamps
- **Docker-based Build Environment**: Encapsulates the entire build environment for easy setup and reproducibility
- **Multi-core Processing**: Uses both cores of the Raspberry Pi Pico for efficient processing

## Project Structure

```
piClk/
├── cmake/                  # CMake modules
│   └── pico_sdk_import.cmake
├── include/                # Header files
│   └── tclk.h
├── pio/                    # PIO programs
│   ├── clockIN.pio
│   ├── clockDecode.pio
│   ├── clockEncode.pio
│   └── clockOUT.pio
├── src/                    # Source files
│   ├── main.cpp
│   └── tclk.c
├── build/                  # Build artifacts (generated)
├── CMakeLists.txt          # CMake configuration
├── Dockerfile              # Docker configuration
└── README.md               # This file
```

## Prerequisites

- Docker
- USB access to Raspberry Pi Pico

## Building with Docker

1. **Build the Docker image**:

   ```bash
   docker build -t piclk-builder .
   ```

2. **Run the Docker container and build the project**:

   ```bash
   docker run --rm -v $(pwd):/app piclk-builder /app/build.sh
   ```

   This will create the build artifacts in the `build` directory, including `piClk.uf2` which can be flashed to the Raspberry Pi Pico.

## Flashing to the Raspberry Pi Pico

### Method 1: Using Docker and picotool

1. Connect your Raspberry Pi Pico to your computer.

2. Run the Docker container with USB device access:

   ```bash
   docker run --rm -v $(pwd):/app --device=/dev/ttyACM0 piclk-builder /app/flash.sh /dev/ttyACM0
   ```

   Replace `/dev/ttyACM0` with the actual device path of your Raspberry Pi Pico.

### Method 2: Manual flashing

1. Connect your Raspberry Pi Pico to your computer while holding the BOOTSEL button.

2. The Pico will appear as a USB mass storage device.

3. Copy the `build/piClk.uf2` file to the Pico:

   ```bash
   cp build/piClk.uf2 /media/YOUR_USERNAME/RPI-RP2/
   ```

   Replace `YOUR_USERNAME` with your actual username.

4. The Pico will automatically reboot and run the program.

## Hardware Setup

1. Connect a clock signal source to GPIO pin 15 (default) of the Raspberry Pi Pico.
2. Connect a clock signal output device to GPIO pin 16 (default) of the Raspberry Pi Pico.
3. Connect the Pico to your computer via USB for power and debugging output.

## TCLK Protocol Details

The TCLK protocol is a 10 MHz serial signal that uses bi-phase (modified Manchester) coding for reliable data transmission:

1. **Bi-phase Encoding**:
   - Transition at each 100ns bit cell boundary (regardless of bit value)
   - Additional transition at mid-cell (after 50ns) if the bit is '1'
   - No mid-cell transition if the bit is '0'

2. **Word Structure** (12 bits total):
   - 1 start bit (always '0')
   - 8 data bits
   - 1 parity bit (odd parity)
   - 2 guaranteed '1's after each event

3. **Default State**: Between clock events, the signal appears as a continuous series of '1's (10 MHz square wave).

4. **Signal Characteristics**:
   - Self-clocking (no separate clock signal needed)
   - When clock event activity is low, the signal appears as a 10 MHz square wave with occasional bursts of bi-phase patterns
   - During high activity, the signal has no repeatable pattern

This implementation follows the protocol as described in the Fermilab paper "Time and Data Distribution Systems at the Fermilab Accelerator" by David G. Beechy and Robert J. Ducar.

## Customization

- To change the input/output pins, modify the `CLOCK_INPUT_PIN` and `CLOCK_OUTPUT_PIN` definitions in `src/main.cpp`.
- To adjust the timestamp resolution or transition threshold, modify the constants in the PIO programs.

## Debugging

The program outputs debug information via USB serial. You can view this output using:

```bash
minicom -D /dev/ttyACM0 -b 115200
```

Or any other serial terminal program.

## License

This project is open source and available under the MIT License.
