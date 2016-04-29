#!/bin/bash

rm modules/PX4Firmware/src/lib/ecl -rf
rm modules/uavcan/dsdl -rf


git submodule update



