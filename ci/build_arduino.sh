#!/bin/bash
set -o xtrace

arduino-cli compile -b arduino:avr:openevse firmware/open_evse
