## Synopsis

SmallyMouse2 is the AT90USB1287 firmware source code and KiCAD schematic/PCB design for the SmallyMouse2 project.

## Motivation

SmallyMouse2 is a project that creates a USB mouse adaptor for retro computers that use quadrature mouse input including the Acorn BBC Micro, Acorn Master series, Commodore Amiga, Atari ST, busmouse compatible computers and many more.  SmallyMouse2 provides both a generic mouse output header (for attaching mouse to retro computer cables) and an IDC connector suitable for use with Acorn 8-bit user-ports.  SmallyMouse2 also features a configurable quadrature rate limiter that prevents VIA overrun when in use with slower 8-bit machines.

SmallyMouse2 supports both JTAG and USB bootloader programming.  The AT90USB1287 is pre-programmed by Atmel with the (FLIP) DFU bootloader; this bootloader is recognised by Atmel Studio as a programming device and can be used to flash the firmware to the board.

## Installation

Note: This is an Atmel Studio 7 project that can be loaded and compiled by the IDE

Please see http://www.waitingforfriday.com/?p=827 for detailed documentation about SmallyMouse2

## Author

SmallyMouse2 is written and maintained by Simon Inns.

## License (Software)

    SmallyMouse2 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SmallyMouse2 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SmallyMouse2. If not, see <http://www.gnu.org/licenses/>.

## License (Hardware)

Both the schematic and the PCB design of SmallyMouse2 (i.e. the KiCAD project and files) are covered by a Creative Commons license; as with the software you are welcome (and encouraged!) to extend, re-spin and otherwise use and modify the design as allowed by the license.  However; under the terms of the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) license you are required to release your design (or redesign) under the same license.  For details of the licensing requirements please see <https://creativecommons.org/licenses/by-sa/4.0/>
