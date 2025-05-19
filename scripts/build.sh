#!/bin/bash
cd /app/build
cmake ..
make -j$(nproc)
