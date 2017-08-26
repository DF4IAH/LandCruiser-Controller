#!/bin/bash

avrdude -pm328p -cdragon_pp -u -Ulfuse:w:0xe2:m -Uhfuse:w:0xb7:m -Uefuse:w:0xfd:m

