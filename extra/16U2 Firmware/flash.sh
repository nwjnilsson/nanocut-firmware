#!/bin/bash
sudo dfu-programmer atmega16u2 erase
sudo dfu-programmer atmega16u2 flash Arduino-usbserial-UNOR3.hex
sudo dfu-programmer atmega16u2 reset