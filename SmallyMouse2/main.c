/************************************************************************
	main.c

	Main functions
    SmallyMouse2 - USB to quadrature mouse converter
    Copyright (C) 2017 Simon Inns

	This file is part of SmallyMouse2.

    SmallyMouse is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	Email: simon.inns@gmail.com

************************************************************************/

// System includes
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <stdio.h>
#include <stdlib.h>

#include <util/delay.h>

// Note: SmallyMouse2 makes extensive use of the LUFA libraries:
//
// LUFA Library
// Copyright (C) Dean Camera, 2015.
// dean [at] fourwalledcubicle [dot] com
// www.lufa-lib.org

// Include LUFA libraries
#include <LUFA/Drivers/Misc/TerminalCodes.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/Peripheral/Serial.h>
#include <LUFA/Platform/Platform.h>

#include "ConfigDescriptor.h"
#include "main.h"

// Limit the fastest output rate of the quadrature
// If it runs too fast the VIA in the host cannot
// perform the timing and the mouse goes a bit haywire
// (especially with 8-bit machines)
#define SLOW_RATELIMIT 60
#define NORMAL_RATELIMIT 20

// A rate limit of 40 means the maximum interrupt speed
// is 40 * 16uS = 640uS.  There are 4 interrupts per 
// position update (due to the quadrature output) so this
// represents a frequency of about 1.5 KHz maximum

// Testing with the AMX SuperROM (SupRom3p41) on a BBC Micro
// showed that a rate limit of 40 prevented overrunning the 
// 6522 (with a lag limit of 256)

// Testing with the same ROM on a BBC Master Turbo (i.e. with
// a 65C102 co-pro) showed that a rate limit of 60 with a 
// lag limit of 128 worked well.

// The LAGLIMIT limits the amount of position updates that
// can 'backlog' due to the RATELIMIT.  If this limit is too
// high then a lot of fast movement will cause the mouse
// pointer to keep moving after the USB mouse has stopped
// causing a 'lag' effect which will confuse the user.
#define SLOW_LAGLIMIT 128
#define NORMAL_LAGLIMIT 128

// The following ISR is called at 600 Hz and performs one half of the 
// quadrature output per call (meaning the mouse is updated at 300 Hz
// towards the retro computer)

// The following globals are used to indicate the difference between
// the USB Mouse movement and the quadrature mouse movement (so that the 
// interrupt towards the host causes the movement to 'catch up' over
// time).
volatile int8_t mouseDirectionX = 0;		// X direction (0 = decrement, 1 = increment)
volatile int8_t mouseEncoderPhaseX = 0;		// Quadrature phase

volatile int8_t mouseDirectionY = 0;		// Y direction (0 = decrement, 1 = increment)
volatile int8_t mouseEncoderPhaseY = 0;		// Quadrature phase

volatile int16_t mouseDistanceX = 0;		// Distance left for mouse to move
volatile int16_t mouseDistanceY = 0;		// Distance left for mouse to move

volatile int8_t updateCounterX = 0;
volatile int8_t updateCounterY = 0;

// The following globals are used to indicate the configuration 
//
// 01: Switch state:			off = normal (active high), on = inverted (active low)
// 02: X direction:				off = normal, on = inverted
// 03: Y direction:				off = normal, on = inverted
// 04: Quadrature output speed: off = normal, on = slow (for 8-bit machines)

volatile int8_t switchState = 0;
volatile int8_t xDirection = 0;
volatile int8_t yDirection = 0;
volatile int8_t outputSpeed = 0;

// Interrupt Service Routine based on Timer0 for mouse X movement quadrature output
ISR(TIMER0_COMPA_vect)
{
	// Process X output
	if (xDirection == 0)
	{
		if (mouseEncoderPhaseX == 0) X1_PORT |=  X1;	// Set X1 to 1
		if (mouseEncoderPhaseX == 1) X2_PORT |=  X2;	// Set X2 to 1
		if (mouseEncoderPhaseX == 2) X1_PORT &= ~X1;	// Set X1 to 0
		if (mouseEncoderPhaseX == 3) X2_PORT &= ~X2;	// Set X2 to 0
	}
	else
	{
		if (mouseEncoderPhaseX == 0) X2_PORT |=  X2;	// Set X2 to 1
		if (mouseEncoderPhaseX == 1) X1_PORT |=  X1;	// Set X1 to 1
		if (mouseEncoderPhaseX == 2) X2_PORT &= ~X2;	// Set X2 to 0
		if (mouseEncoderPhaseX == 3) X1_PORT &= ~X1;	// Set X1 to 0
	}

	// Only change phase if the mouse is still moving
	if (mouseDistanceX != 0)
	{
		if (mouseDirectionX == 0) mouseEncoderPhaseX--; else mouseEncoderPhaseX++;
		mouseDistanceX--;
	}

	// Range check the phase
	if ((mouseDirectionX == 1) && (mouseEncoderPhaseX > 3)) mouseEncoderPhaseX = 0;
	if ((mouseDirectionX == 0) && (mouseEncoderPhaseX < 0)) mouseEncoderPhaseX = 3;
}

// Interrupt Service Routine based on Timer2 for mouse Y movement quadrature output
ISR(TIMER2_COMPA_vect)
{
	// Process Y output
	if (yDirection == 0)
	{
		if (mouseEncoderPhaseY == 3) Y1_PORT &= ~Y1;	// Set Y1 to 0
		if (mouseEncoderPhaseY == 2) Y2_PORT &= ~Y2;	// Set Y2 to 0
		if (mouseEncoderPhaseY == 1) Y1_PORT |=  Y1;	// Set Y1 to 1
		if (mouseEncoderPhaseY == 0) Y2_PORT |=  Y2;	// Set Y2 to 1
	}
	else
	{
		if (mouseEncoderPhaseY == 3) Y2_PORT &= ~Y2;	// Set Y2 to 0
		if (mouseEncoderPhaseY == 2) Y1_PORT &= ~Y1;	// Set Y1 to 0
		if (mouseEncoderPhaseY == 1) Y2_PORT |=  Y2;	// Set Y2 to 1
		if (mouseEncoderPhaseY == 0) Y1_PORT |=  Y1;	// Set Y1 to 1
	}

	// Only change phase if the mouse is still moving
	if (mouseDistanceY != 0)
	{
		if (mouseDirectionY == 0) mouseEncoderPhaseY--; else mouseEncoderPhaseY++;
		mouseDistanceY--;
	}

	// Range check the phase
	if ((mouseDirectionY == 1) && (mouseEncoderPhaseY > 3)) mouseEncoderPhaseY = 0;
	if ((mouseDirectionY == 0) && (mouseEncoderPhaseY < 0)) mouseEncoderPhaseY = 3;
}

// Main function
int main(void)
{
	// Initialise the SmallyMouse hardware
	initialiseHardware();
	
	// The frequency of the reports from the USB mouse is about 120
	// reports per second.  Each report can cause up to 127 position movements
	// of the quadrature wheel.
	//
	// We are receive a USB update approximately every 8000uS
	//
	// Prescale is 256 = 16,000,000 / 256 = 62,500 ticks per second
	// 62,500 ticks per second is one interrupt every 16uS

	// Configure Timer0 to interrupt (8-bit timer)
	OCR0A = 0;
	TCCR0A = (1 << WGM01); // CTC
	TCCR0B = 4; //  4 = /256 prescale
	TIMSK0 = (1 << OCIE0A); // Timer0 interrupt enabled
	
	// Configure Timer2 to interrupt (8-bit timer)
	OCR2A = 0;
	TCCR2A = (1 << WGM21);
	TCCR2B = 5; // 5 = /256 prescale
	TIMSK2 = (1 << OCIE2A); // Timer2 interrupt enabled

	// Enable interrupts (required for USB support)
	sei();

	// Set initial mouse button state (active low logic)
	LB_PORT |= LB; // Pin = 1 (inactive)
	MB_PORT |= MB; // Pin = 1 (inactive)
	RB_PORT |= RB; // Pin = 1 (inactive)
	
	// Main processing loop
	while(1)
	{
		// Check configuration
		checkConfiguration();
		
		// Perform any pending mouse actions
		processMouse();

		// Process the USB host interface
		USB_USBTask();
	}
}

// Initialise the SmallyMouse2 hardware
void initialiseHardware(void)
{
	// Disable the watchdog timer (if set in fuses)
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	// Disable the clock divider (if set in fuses)
	clock_prescale_set(clock_div_1);

	// Set the quadrature output pins to output
	X1_DDR |= X1; // Output
	X2_DDR |= X2; // Output
	Y1_DDR |= Y1; // Output
	Y2_DDR |= Y2; // Output
	LB_DDR |= LB; // Output
	MB_DDR |= MB; // Output
	RB_DDR |= RB; // Output
	
	// Set all output pins equal to zero
	X1_PORT &= ~X1; // Pin = 0
	X2_PORT &= ~X2; // Pin = 0
	Y1_PORT &= ~Y1; // Pin = 0
	Y2_PORT &= ~Y2; // Pin = 0
	LB_PORT &= ~LB; // Pin = 0
	MB_PORT &= ~MB; // Pin = 0
	RB_PORT &= ~RB; // Pin = 0

	// Initialise the rate limit setting switch
	RATESW_DDR &= ~RATESW; // Input
	RATESW_PORT |= RATESW; // Turn on weak pull-up
	
	// Initialise the expansion (Ian) header
	E0_DDR |= ~E0; // Output
	E1_DDR |= ~E1; // Output
	E2_DDR |= ~E2; // Output
	E3_DDR |= ~E3; // Output
	E4_DDR |= ~E4; // Output
	E5_DDR |= ~E5; // Output
	E6_DDR |= ~E6; // Output
	E7_DDR |= ~E7; // Output
	
	E0_PORT &= ~E0; // Pin = 0
	E1_PORT &= ~E1; // Pin = 0
	E2_PORT &= ~E2; // Pin = 0
	E3_PORT &= ~E3; // Pin = 0
	E4_PORT &= ~E4; // Pin = 0
	E5_PORT &= ~E5; // Pin = 0
	E6_PORT &= ~E6; // Pin = 0
	E7_PORT &= ~E7; // Pin = 0
	
	// Initialise the LUFA USB stack
	USB_Init();
	
	// By default, SmallyMouse2 will output USB debug events on the 
	// AVR's UART port.  You can monitor the debug console by connecting
	// a serial to USB adapter.  Only the Tx (D3 and 0V pins are required).
	
	// Initialise the serial UART - 9600 baud 8N1
	Serial_Init(9600, false);

	// Create a serial debug stream on stdio
	// Note: The serial UART is available on the 
	// expansion (Ian) header
	Serial_CreateStream(NULL);

	// Output some header information to the serial console
	puts_P(PSTR(ESC_FG_YELLOW "SmallyMouse2 - Serial debug console\r\n" ESC_FG_WHITE));
	puts_P(PSTR(ESC_FG_YELLOW "(c)2017 Simon Inns\r\n" ESC_FG_WHITE));
	puts_P(PSTR(ESC_FG_YELLOW "http://www.waitingforfriday.com\r\n" ESC_FG_WHITE));
}

// Check the status of the configuration
void checkConfiguration(void)
{
	// Test the rate limit switch position
	if ((RATESW_PIN & RATESW) != 0) outputSpeed = 0;
	else outputSpeed = 1;
}

// Read the mouse USB report and process the information
void processMouse(void)
{
	USB_MouseReport_Data_t MouseReport;
	int16_t updateSpeed;
		
	// Only process the mouse if a mouse is attached to the USB port
	if (USB_HostState != HOST_STATE_Configured)	return;

	// Select mouse data pipe
	Pipe_SelectPipe(MOUSE_DATA_IN_PIPE);

	// Unfreeze mouse data pipe
	Pipe_Unfreeze();

	// Has a packet arrived from the USB device?
	if (!(Pipe_IsINReceived()))
	{
		// Nothing received, exit...
		Pipe_Freeze();
		return;
	}

	// Ensure pipe is in the correct state before reading
	if (Pipe_IsReadWriteAllowed())
	{
		// Read in mouse report data
		Pipe_Read_Stream_LE(&MouseReport, sizeof(MouseReport), NULL);
		
		// The USB mouse report contains 3 variables: button, X and Y
		//
		// Button is an 8-bit flag containing up to 5 button states:
		//   (LSB)1 = Left button
		//        2 = Right button
		//        4 = Middle button
		//
		// Note: Buttons are active low (i.e. 0 = pressed)
		//
		// X and Y indicate the number of steps the mouse moved since the
		// last report was sent.
		//
		// +X = Mouse going right
		// -X = Mouse going left
		// +Y = Mouse going down
		// -Y = Mouse going up
		//
		// X and Y have a range of -127 to +127
		
		// If the mouse movement changes direction then disregard any remaining
		// motion.
		if (MouseReport.X > 0 && mouseDirectionX == 0) mouseDistanceX = 0;
		if (MouseReport.X < 0 && mouseDirectionX == 1) mouseDistanceX = 0;
		if (MouseReport.Y > 0 && mouseDirectionY == 0) mouseDistanceY = 0;
		if (MouseReport.Y < 0 && mouseDirectionY == 1) mouseDistanceY = 0;
		
		// Process mouse X movement -------------------------------------------
		if (MouseReport.X > 0)
		{
			// Set the mouse direction to incrementing
			mouseDirectionX = 1;
		
			// Set the mouse distance to move
			mouseDistanceX += MouseReport.X; // (0-127)
			
			// Limit the amount of positional lag allowed
			if (outputSpeed == 0)
			{
				if (mouseDistanceX > NORMAL_LAGLIMIT) mouseDistanceX = NORMAL_LAGLIMIT;
			}
			else
			{
				if (mouseDistanceX > SLOW_LAGLIMIT) mouseDistanceX = SLOW_LAGLIMIT;
			}
			
			// We have to send mouseDistanceX updates in 7040uS maximum
			// interrupt speed is 16uS per interrupt
			updateSpeed = (7040 / mouseDistanceX) / 16;
			
			// Limit the slowest output rate to 8-bits for the timer
			if (updateSpeed > 255) updateSpeed = 255;
			
			// Limit the fastest output rate
			if (outputSpeed == 0)
			{
				if (updateSpeed < NORMAL_RATELIMIT) updateSpeed = NORMAL_RATELIMIT;
			}
			else
			{
				if (updateSpeed < SLOW_RATELIMIT) updateSpeed = SLOW_RATELIMIT;
			}			
			
			// Set the interrupt speed
			OCR0A = (uint8_t)updateSpeed;
			TCCR0B = 4; // /16 prescale
		}
		
		if (MouseReport.X < 0)
		{
			// Set the mouse direction to decrementing
			mouseDirectionX = 0;
		
			// Set the mouse distance to move
			mouseDistanceX += abs(MouseReport.X); // (0-127)
			
			// Limit the amount of positional lag allowed
			if (outputSpeed == 0)
			{
				if (mouseDistanceX > NORMAL_LAGLIMIT) mouseDistanceX = NORMAL_LAGLIMIT;
			}
			else
			{
				if (mouseDistanceX > SLOW_LAGLIMIT) mouseDistanceX = SLOW_LAGLIMIT;
			}			
			
			// We have to send mouseDistanceX updates in 7040uS maximum
			// interrupt speed is 16uS per interrupt
			updateSpeed = (7040 / mouseDistanceX) / 16;
			
			// Limit the slowest output rate to 8-bits for the timer
			if (updateSpeed > 255) updateSpeed = 255;
			
			// Limit the fastest output rate
			if (outputSpeed == 0)
			{
				if (updateSpeed < NORMAL_RATELIMIT) updateSpeed = NORMAL_RATELIMIT;
			}
			else
			{
				if (updateSpeed < SLOW_RATELIMIT) updateSpeed = SLOW_RATELIMIT;
			}
			
			// Set the interrupt speed
			OCR0A = (uint8_t)updateSpeed;
			TCCR0B = 4; // /16 prescale
		}
		
		// Process mouse Y ----------------------------------------------------
		if (MouseReport.Y > 0)
		{
			// Set the mouse direction to incrementing
			mouseDirectionY = 1;
			
			// Set the mouse distance to move
			mouseDistanceY += MouseReport.Y; // (1-127)
			
			// Limit the amount of positional lag allowed
			if (outputSpeed == 0)
			{
				if (mouseDistanceY > NORMAL_LAGLIMIT) mouseDistanceY = NORMAL_LAGLIMIT;
			}
			else
			{
				if (mouseDistanceY > SLOW_LAGLIMIT) mouseDistanceY = SLOW_LAGLIMIT;
			}
			
			// We have to send mouseDistanceY updates in 7040uS maximum
			// interrupt speed is 16uS per interrupt
			updateSpeed = (7040 / mouseDistanceY) / 16;
			
			// Limit the slowest output rate to 8-bits for the timer
			if (updateSpeed > 255) updateSpeed = 255;
			
			// Limit the fastest output rate
			if (outputSpeed == 0)
			{
				if (updateSpeed < NORMAL_RATELIMIT) updateSpeed = NORMAL_RATELIMIT;
			}
			else
			{
				if (updateSpeed < SLOW_RATELIMIT) updateSpeed = SLOW_RATELIMIT;
			}
			
			// Set the interrupt speed
			OCR2A = (uint8_t)updateSpeed;
			TCCR2B = 5; // /16 prescale
		}
		
		if (MouseReport.Y < 0)
		{
			// Set the mouse direction to decrementing
			mouseDirectionY = 0;
			
			// Set the mouse distance to move
			mouseDistanceY += abs(MouseReport.Y); // (-127 - -1)
			
			// Limit the amount of positional lag allowed
			if (outputSpeed == 0)
			{
				if (mouseDistanceY > NORMAL_LAGLIMIT) mouseDistanceY = NORMAL_LAGLIMIT;
			}
			else
			{
				if (mouseDistanceY > SLOW_LAGLIMIT) mouseDistanceY = SLOW_LAGLIMIT;
			}
			
			// We have to send mouseDistanceY updates in 7040uS maximum
			// interrupt speed is 16uS per interrupt
			updateSpeed = (7040 / mouseDistanceY) / 16;
			
			// Limit the slowest output rate to 8-bits for the timer
			if (updateSpeed > 255) updateSpeed = 255;
			
			// Limit the fastest output rate
			if (outputSpeed == 0)
			{
				if (updateSpeed < NORMAL_RATELIMIT) updateSpeed = NORMAL_RATELIMIT;
			}
			else
			{
				if (updateSpeed < SLOW_RATELIMIT) updateSpeed = SLOW_RATELIMIT;
			}
			
			// Set the interrupt speed
			OCR2A = (uint8_t)updateSpeed;
			TCCR2B = 5; // /16 prescale
		}
		
		// Process mouse buttons ----------------------------------------------

		if (switchState == 0) // Non-inverted switches
		{
			// Check for left mouse button
			if ((MouseReport.Button & 0x01) == 0) LB_PORT |= LB;
			else LB_PORT &= ~LB;
			
			// Check for middle mouse button
			if ((MouseReport.Button & 0x04) == 0) MB_PORT |= MB;
			else MB_PORT &= ~MB;
			
			// Check for right mouse button
			if ((MouseReport.Button & 0x02) == 0) RB_PORT |= RB;
			else RB_PORT &= ~RB;
		}
		else // Inverted switches
		{
			// Check for left mouse button
			if ((MouseReport.Button & 0x01) == 0) LB_PORT &= ~LB;
			else LB_PORT |= LB;
			
			// Check for middle mouse button
			if ((MouseReport.Button & 0x04) == 0) MB_PORT &= ~MB;
			else MB_PORT |= MB;
			
			// Check for right mouse button
			if ((MouseReport.Button & 0x02) == 0) RB_PORT &= ~RB;
			else RB_PORT |= RB;
		}
	}

	// Clear the IN endpoint, ready for next data packet
	Pipe_ClearIN();

	// Refreeze mouse data pipe
	Pipe_Freeze();
}

// LUFA event handlers ------------------------------------------------------------------------------------------------

// Event handler for the USB_DeviceAttached event. This indicates that a device has been attached to the host, and
// starts the library USB task to begin the enumeration and USB management process.
void EVENT_USB_Host_DeviceAttached(void)
{
	puts_P(PSTR(ESC_FG_GREEN "Device Attached.\r\n" ESC_FG_WHITE));
}

// Event handler for the USB_DeviceUnattached event. This indicates that a device has been removed from the host, and
// stops the library USB task management process.
void EVENT_USB_Host_DeviceUnattached(void)
{
	puts_P(PSTR(ESC_FG_GREEN "Device Unattached.\r\n" ESC_FG_WHITE));
}

// Event handler for the USB_DeviceEnumerationComplete event. This indicates that a device has been successfully
// enumerated by the host and is now ready to be used by the application.
void EVENT_USB_Host_DeviceEnumerationComplete(void)
{
	puts_P(PSTR("Getting Config Data.\r\n"));

	uint8_t ErrorCode;

	/* Get and process the configuration descriptor data */
	if ((ErrorCode = ProcessConfigurationDescriptor()) != SuccessfulConfigRead)
	{
		if (ErrorCode == ControlError)
		{
		  puts_P(PSTR(ESC_FG_RED "Control Error (Get Configuration).\r\n"));
		}
		else
		{
		  puts_P(PSTR(ESC_FG_RED "Invalid Device.\r\n"));
		}

		printf_P(PSTR(" -- Error Code: %d\r\n" ESC_FG_WHITE), ErrorCode);

		return;
	}

	/* Set the device configuration to the first configuration (rarely do devices use multiple configurations) */
	if ((ErrorCode = USB_Host_SetDeviceConfiguration(1)) != HOST_SENDCONTROL_Successful)
	{
		printf_P(PSTR(ESC_FG_RED "Control Error (Set Configuration).\r\n"
		                         " -- Error Code: %d\r\n" ESC_FG_WHITE), ErrorCode);

		return;
	}

	/* HID class request to set the mouse protocol to the Boot Protocol */
	USB_ControlRequest = (USB_Request_Header_t)
		{
			.bmRequestType = (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE),
			.bRequest      = HID_REQ_SetProtocol,
			.wValue        = 0,
			.wIndex        = 0,
			.wLength       = 0,
		};

	/* Select the control pipe for the request transfer */
	Pipe_SelectPipe(PIPE_CONTROLPIPE);

	/* Send the request, display error and wait for device detach if request fails */
	if ((ErrorCode = USB_Host_SendControlRequest(NULL)) != HOST_SENDCONTROL_Successful)
	{
		printf_P(PSTR(ESC_FG_RED "Control Error (Set Protocol).\r\n"
								 " -- Error Code: %d\r\n" ESC_FG_WHITE), ErrorCode);

		USB_Host_SetDeviceConfiguration(0);
		return;
	}

	puts_P(PSTR("Mouse Enumerated.\r\n"));
}

// Event handler for the USB_HostError event. This indicates that a hardware error occurred while in host mode.
void EVENT_USB_Host_HostError(const uint8_t ErrorCode)
{
	USB_Disable();

	printf_P(PSTR(ESC_FG_RED "Host Mode Error\r\n"
	                       " -- Error Code %d\r\n" ESC_FG_WHITE), ErrorCode);

	while(1);
}

// Event handler for the USB_DeviceEnumerationFailed event. This indicates that a problem occurred while
// enumerating an attached USB device.
void EVENT_USB_Host_DeviceEnumerationFailed(const uint8_t ErrorCode,
                                            const uint8_t SubErrorCode)
{
	printf_P(PSTR(ESC_FG_RED "Dev Enum Error\r\n"
	                         " -- Error Code %d\r\n"
	                         " -- Sub Error Code %d\r\n"
	                         " -- In State %d\r\n" ESC_FG_WHITE), ErrorCode, SubErrorCode, USB_HostState);
}


