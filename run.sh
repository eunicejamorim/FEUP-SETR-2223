#!/bin/bash
arduino-cli compile -b arduino:avr:micro --clean $2
arduino-cli upload -b arduino:avr:micro -p $1 $2
#arduino-cli monitor -p $1