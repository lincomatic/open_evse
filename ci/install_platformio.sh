#!/bin/bash
set -o xtrace

pip install -U platformio
pio upgrade
pio --version
