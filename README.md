# Raspberry Pi Pico TCLK/MDAT Protocol Implementation (pico-clock)

This project implements the TCLK (Tevatron Clock) and MDAT (Machine Data) protocols for the Raspberry Pi Pico using the Programmable Input/Output (PIO) feature. It captures and timestamps input signals with a resolution of 50ns and implements the bi-phase (modified Manchester) encoding for TCLK and the 28-bit frame structure for MDAT as described in the TCLK_Paper.pdf.

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
- **MDAT Protocol Implementation**: A companion protocol for machine data transmission
  - 28-bit frame length
  - Self-clocking at 10 MBit/s
  - Frame is always 2.75 microseconds in length
  - Parity ensures consistent frame length
- **PIO-based Implementation**:
  - **TCLK Components**:
    * **tclkIN**: Listens to an input clock signal and timestamps transitions with a resolution of 50ns
    * **tclkDecode**: Processes the timestamps and decodes data according to the TCLK protocol
    * **tclkEncode**: Encodes data according to the TCLK protocol and generates timestamps
    * **tclkOUT**: Generates an output clock signal based on the encoded timestamps
  - **MDAT Components**:
    * **mdatIN**: Captures input signal transitions and generates timestamps
    * **mdatDecode**: Processes timestamps and decodes data according to the MDAT protocol
    * **mdatEncode**: Encodes data according to the MDAT protocol and generates timestamps
    * **mdatOUT**: Generates output signals based on the encoded timestamps
- **Docker-based Build Environment**: Encapsulates the entire build environment for easy setup and reproducibility
- **Multi-core Processing**: Uses both cores of the Raspberry Pi Pico for efficient processing

## Project Structure

```
pico-clock/
├── cmake/                  # CMake modules
│   └── pico_sdk_import.cmake
├── include/                # Header files
│   ├── tclk.h              # TCLK protocol header
│   └── mdat.h              # MDAT protocol header
├── pio/                    # PIO programs
│   ├── tclkIN.pio          # TCLK input program
│   ├── tclkDecode.pio      # TCLK decoding program
│   ├── tclkEncode.pio      # TCLK encoding program
│   ├── tclkOUT.pio         # TCLK output program
│   ├── mdatIN.pio          # MDAT input program
│   ├── mdatDecode.pio      # MDAT decoding program
│   ├── mdatEncode.pio      # MDAT encoding program
│   └── mdatOUT.pio         # MDAT output program
├── scripts/                # Build and flash scripts
│   ├── build.sh
│   └── flash.sh
├── src/                    # Source files
│   ├── main.cpp            # Main application
│   ├── tclk.c              # TCLK implementation
│   └── mdat.c              # MDAT implementation
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
   docker build -t pico-clock-builder .
   ```

2. **Run the Docker container and build the project**:

   ```bash
   docker run --rm -v $(pwd):/app/project pico-clock-builder /app/scripts/build.sh
   ```

   This will create the build artifacts in the `build` directory, including `pico-clock.uf2` which can be flashed to the Raspberry Pi Pico.

## Flashing to the Raspberry Pi Pico

### Method 1: Using Docker and picotool

1. Connect your Raspberry Pi Pico to your computer.

2. Run the Docker container with USB device access:

   ```bash
   docker run --rm -v $(pwd):/app --device=/dev/ttyACM0 pico-clock-builder /app/scripts/flash.sh /dev/ttyACM0
   ```

   Replace `/dev/ttyACM0` with the actual device path of your Raspberry Pi Pico.

### Method 2: Manual flashing

1. Connect your Raspberry Pi Pico to your computer while holding the BOOTSEL button.

2. The Pico will appear as a USB mass storage device.

3. Copy the `build/pico-clock.uf2` file to the Pico:

   ```bash
   cp build/pico-clock.uf2 /media/YOUR_USERNAME/RPI-RP2/
   ```

   Replace `YOUR_USERNAME` with your actual username.

4. The Pico will automatically reboot and run the program.

## Hardware Setup

### TCLK Connections
1. Connect a TCLK signal source to GPIO pin 16 (default) of the Raspberry Pi Pico.
2. Connect a TCLK signal output device to GPIO pin 17 (default) of the Raspberry Pi Pico.

### MDAT Connections
1. Connect an MDAT signal source to GPIO pin 25 (default) of the Raspberry Pi Pico.
2. Connect an MDAT signal output device to GPIO pin 24 (default) of the Raspberry Pi Pico.

### General Setup
1. Connect the Pico to your computer via USB for power and debugging output.
2. For testing, you can jumper the output pins to the corresponding input pins to create a loopback.

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

## MDAT Protocol Details

The MDAT (Machine Data) protocol is a companion to the TCLK protocol, used for transmitting machine status data:

1. **Frame Structure**:
   - 28-bit frame length
   - Self-clocking at 10 MBit/s
   - Frame is always 2.75 microseconds in length
   - Parity ensures consistent frame length

2. **Implementation**:
   - **mdatIN**: Captures input signal transitions and generates timestamps
   - **mdatDecode**: Processes timestamps and decodes data according to the MDAT protocol
   - **mdatEncode**: Encodes data according to the MDAT protocol and generates timestamps
   - **mdatOUT**: Generates output signals based on the encoded timestamps

3. **Integration with TCLK**:
   - MDAT and TCLK work together to provide a complete timing and data distribution system
   - TCLK provides timing events while MDAT provides machine status data
   - Both protocols can be used independently or together depending on requirements

## Customization

### Pin Configuration
- To change the TCLK input/output pins, modify the `TCLK_INPUT_PIN` and `TCLK_OUTPUT_PIN` definitions in `src/main.cpp`.
- To change the MDAT input/output pins, modify the `MDAT_INPUT_PIN` and `MDAT_OUTPUT_PIN` definitions in `src/main.cpp`.

### Protocol Parameters
- To adjust the TCLK timestamp resolution or transition threshold, modify the constants in the TCLK PIO programs.
- To adjust the MDAT frame parameters, modify the constants in the MDAT PIO programs.

## Debugging

The program outputs debug information via USB serial. You can view this output using:

```bash
minicom -D /dev/ttyACM0 -b 115200
```

Or any other serial terminal program.

## License

This project is open source and available under the MIT License.
