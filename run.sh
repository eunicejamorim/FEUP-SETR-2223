#!/bin/bash
arduino-cli compile -b arduino:avr:uno --clean $1
arduino-cli upload -b arduino:avr:uno -p /dev/ttyACM0 $1
arduino-cli monitor -p /dev/ttyACM0