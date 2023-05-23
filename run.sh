#!/bin/bash
arduino-cli compile -b arduino:avr:uno --clean $2
arduino-cli upload -b arduino:avr:uno -p $1 $2
arduino-cli monitor -p $1