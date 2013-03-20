DIPIO
=====

DIPIO Cypress FX2 Source Code (including uVision project)

USB Mini or DIPIO
=====

This project is actually the USB programmer for the DIPIO (which I'm currently calling the USB Mini).
However, the DIPIO is a more unique name and will more likely be recognized, so this git repo is the DIPIO repo.
The DIPIO itself is simply a MachXO2 FPGA and doesn't contain custom firmware (it is simply programmed by the firmware in this repo).
Check out the DIPIO on http://www.mdsinstruments.com for more information on the product (website may still be under construction).

What This Contains
=====

This repo contains the uVision project and all associated source files to build the DIPIO USB programmer .hex and .iic files.
You'll need to install the Cypress development tools (http://www.cypress.com/?rID=14319) to create the .iic file and load the firmware.
The source itself uses a very simple opcode then data protocol via endpoint 1.
The opcodes cover both I2C and SPI programming, as well as some information on version, etc.

VID/PID Pair
=====
This project currently uses a Cypress VID (0x04b4) and random PID, but will hopefully be updated to use an open source VID/PID pair.