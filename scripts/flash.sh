#!/bin/bash
if [ -z "$1" ]; then
  echo "Usage: $0 <device>
Example: $0 /dev/ttyACM0"
  exit 1
fi

DEVICE=$1

cd /app/build
picotool load -f pico-clock.uf2 -t uf2 -d $DEVICE
