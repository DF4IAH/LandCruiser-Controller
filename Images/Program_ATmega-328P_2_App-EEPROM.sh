#!/bin/bash

avrdude -pm328p -cdragon_pp -u -Ueeprom:w:../LandCruiser-Controller_SW/LandCruiser-Controller_SW/Release/LandCruiser-Controller_SW.eep:a

