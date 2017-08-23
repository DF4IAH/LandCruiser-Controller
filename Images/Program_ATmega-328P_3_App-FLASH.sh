#!/bin/bash

avrdude -pm328p -cdragon_pp -u -Uflash:w:../LandCruiser-Controller_SW/LandCruiser-Controller_SW/Release/LandCruiser-Controller_SW.hex:a

