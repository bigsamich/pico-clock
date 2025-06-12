FROM ubuntu:22.04

# Avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    build-essential \
    python3 \
    git \
    pkg-config \
    libusb-1.0-0-dev \
    usbutils \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set up working directory
WORKDIR /app

# Clone Pico SDK and examples
RUN git clone --depth 1 https://github.com/raspberrypi/pico-sdk.git \
    && cd pico-sdk \
    && git submodule update --init \
    && cd ..

# Set environment variables
ENV PICO_SDK_PATH=/app/pico-sdk

# Install picotool for flashing
RUN git clone --depth 1 https://github.com/raspberrypi/picotool.git \
    && cd picotool \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install \
    && cd ../..

# Set up working directory
WORKDIR /app
COPY scripts /app/scripts

# Create build directory
RUN mkdir -p /app/build

# Default command to keep the container running
CMD ["tail", "-f", "/dev/null"] 
