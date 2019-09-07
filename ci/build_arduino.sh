#!/bin/bash
set -o xtrace

arduino-cli compile -b arduino:avr:openevse -v firmware/open_evse
