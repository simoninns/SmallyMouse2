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

// Configuration ------------------------------------------------------------------------------------------------------

// Quadrature output limiting
//
// This setting limits the maximum frequency of the quadrature output towards the retro computer.
// If the rate of output is too high the retro computer cannot 'count' the input accurately and
// this causes spurious mouse movement.
//
// There are two possible limits based on the setting of the 'Slow' configuration jumper.  If the 
// jumper is present, the software will use the setting of "SLOW_RATELIMIT" otherwise the setting
// of "FAST_RATELIMIT" is used.  The slow rate limit is recommended for older 8-bit machines, for
// 16 and 32 bit machines the fast rate limit should be used.
//
// This setting is in Hertz (maximum allowed output rate is 15625 Hz - do not exceed this):
#define SLOW_RATELIMIT 1400
#define FAST_RATELIMIT 3000

// Movement lag limiting
//
// Since the output rate is limited, it's possible for the USB movement input to overrun the 
// quadrature output speed.  The software will 'buffer' any overrun and continue to output
// the indicated movement once the USB input slows down or stops.  This prevents movement
// being lost.  However, this buffering causes the quadrature mouse movement to lag behind
// the actual mouse movement.  Too much lag will cause the mouse to visibly keep moving after
// the user has stopped moving the physical mouse (and is confusing to the user).  Therefore
// the lag limit setting limits the number of movements that can be buffered (extra movement
// over the lag limit is silently dropped).
//
// Note that the effect of the lag limit is dependent on the maximum allowed output rate.  The
// amount of lag will be higher for lower rate output as each buffered movement takes longer
// to send.  For example, at a rate limit of 1400 a lag limit of 700 represents half a second.
//
// The setting is the number of buffered movements that are allowed:
#define SLOW_LAGLIMIT 50
#define FAST_LAGLIMIT 100

// Interrupt Service Routines for quadrature output -------------------------------------------------------------------

// The following globals are used by the interrupt service routines to track the mouse
// movement.  The mouseDirection indicates which direction the mouse is moving in and
// the mouseEncoderPhase tracks the current phase of the quadrature output.
//
// The mouseDistance variable tracks the current distance the mouse has left to move
// (this is incremented by the USB mouse reports and decremented by the ISRs as they
// output the quadrature to the retro host).
volatile int8_t mouseDirectionX = 0;		// X direction (0 = decrement, 1 = increment)
volatile int8_t mouseEncoderPhaseX = 0;		// X Quadrature phase (0-3)

volatile int8_t mouseDirectionY = 0;		// Y direction (0 = decrement, 1 = increment)
volatile int8_t mouseEncoderPhaseY = 0;		// Y Quadrature phase (0-3)

volatile int16_t mouseDistanceX = 0;		// Distance left for mouse to move
volatile int16_t mouseDistanceY = 0;		// Distance left for mouse to move

// Interrupt Service Routine based on Timer0 for mouse X movement quadrature output
ISR(TIMER0_COMPA_vect)
{
	// Process X output
	if (mouseEncoderPhaseX == 0) X1_PORT |=  X1;	// Set X1 to 1
	if (mouseEncoderPhaseX == 1) X2_PORT |=  X2;	// Set X2 to 1
	if (mouseEncoderPhaseX == 2) X1_PORT &= ~X1;	// Set X1 to 0
	if (mouseEncoderPhaseX == 3) X2_PORT &= ~X2;	// Set X2 to 0

	// Only change phase if the mouse is still moving
	if (mouseDistanceX > 0) {
		if (mouseDirectionX == 0) mouseEncoderPhaseX--; else mouseEncoderPhaseX++;
		
		// Decrement the distance left to move
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
	if (mouseEncoderPhaseY == 3) Y1_PORT &= ~Y1;	// Set Y1 to 0
	if (mouseEncoderPhaseY == 2) Y2_PORT &= ~Y2;	// Set Y2 to 0
	if (mouseEncoderPhaseY == 1) Y1_PORT |=  Y1;	// Set Y1 to 1
	if (mouseEncoderPhaseY == 0) Y2_PORT |=  Y2;	// Set Y2 to 1

	// Only change phase if the mouse is still moving
	if (mouseDistanceY > 0) {
		if (mouseDirectionY == 0) mouseEncoderPhaseY--; else mouseEncoderPhaseY++;
		
		// Decrement the distance left to move
		mouseDistanceY--;
	}

	// Range check the phase
	if ((mouseDirectionY == 1) && (mouseEncoderPhaseY > 3)) mouseEncoderPhaseY = 0;
	if ((mouseDirectionY == 0) && (mouseEncoderPhaseY < 0)) mouseEncoderPhaseY = 3;
}

// Main function
int main(void)
{
	// Initialise the SmallyMouse2 hardware
	initialiseHardware();
	
	// Initialise the ISR timers
	initialiseTimers();

	// Enable interrupts (required for USB support and quadrature output ISRs)
	sei();
	
	// Main processing loop
	while(1) {
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
	
	// Set quadrature output pins to zero
	X1_PORT &= ~X1; // Pin = 0
	X2_PORT &= ~X2; // Pin = 0
	Y1_PORT &= ~Y1; // Pin = 0
	Y2_PORT &= ~Y2; // Pin = 0
	
	// Set mouse button output pins to off
	LB_PORT &= ~LB; // Pin = 0 (off)
	MB_PORT &= ~MB; // Pin = 0 (off)
	RB_PORT &= ~RB; // Pin = 0 (off)

	// Set the rate limit configuration header to input
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

	// Output some debug header information to the serial console
	puts_P(PSTR(ESC_FG_YELLOW "SmallyMouse2 - Serial debug console\r\n" ESC_FG_WHITE));
	puts_P(PSTR(ESC_FG_YELLOW "(c)2017 Simon Inns\r\n" ESC_FG_WHITE));
	puts_P(PSTR(ESC_FG_YELLOW "http://www.waitingforfriday.com\r\n" ESC_FG_WHITE));
}

// Initialise the ISR timers
void initialiseTimers(void)
{
	// The frequency of the reports from the USB mouse is about 120 Hz (i.e. 120
	// reports per second).  Each report can indicate up to 127 position movements
	// in each direction (X & Y)
	
	// If we can receive 127 movements per report at a rate of 120 Hz
	// then the maximum movement per second is 120 Hz * 127 movements =
	// 15,240 movements per second (i.e. the quadrature output has to 
	// be at least 15,240 Hz to keep up)
	
	// The setting of the timers controls the maximum interrupt speed and 
	// therefore the maximum possible quadrature rate output.
	// Note that there are 4 interrupts per mouse movement (one for each
	// quadrature phase), so the maximum output speed is 4 times slower
	// than the maximum interrupt speed.
	
	// Timer0 and Timer2 are 8-bit timers.  Therefore the slowest interrupt
	// speed possible is 256 times the tick period and the fastest is a single
	// tick period.
	
	// Timer prescale is /256 and the AVR clock speed is 16,000,000 Hz
	// 16,000,000 Hz / 256 prescale = 62,500 ticks per second
	//
	// 1,000,000 uS per second / 62,500 ticks per second = 16 uS per tick
	//
	// 62,500 Hz / 4 (quadrature output) = 15,625 Hz (the maximum quadrature
	// output rate)
	//
	// Therefore 15,625 Hz / 256 = 61 Hz (the minimum quadrature output rate)
	
	// Note that 'rate' here is a little misleading for the minimum output.
	// Since there are 4 interrupts required for the output the minimum rate
	// has little effect.

	// Configure Timer0 to interrupt (8-bit timer)
	OCR0A = 0; // In CTC mode OCR0A = TOP
	TCCR0A = (1 << WGM01); // Clear timer on compare match (CTC) mode
	TCCR0B = 4; //  CS02/CS01/CS00 = binary 100 = /256 prescale
	TIMSK0 = (1 << OCIE0A); // Timer0 interrupt enabled
		
	// Configure Timer2 to interrupt (8-bit timer)
	OCR2A = 0; // In CTC mode OCR2A = TOP
	TCCR2A = (1 << WGM21); // Clear timer on compare match (CTC) mode
	TCCR2B = 6; // CS22/CS21/CS20 = binary 110 = /256 prescale
	TIMSK2 = (1 << OCIE2A); // Timer2 interrupt enabled
}

// Check the setting of the rate limiter configuration switch
// Returns 1 if a jumper is present (on) or 0 if a jumper is not present (off)
uint8_t getRateLimiterState(void)
{
	// Test the rate limiter configuration switch position
	if ((RATESW_PIN & RATESW) != 0) return 0; // Rate limiter switch is off
	
	// Rate limiter switch is on
	return 1;
}

// Read the mouse USB report and process the information
void processMouse(void)
{
	USB_MouseReport_Data_t MouseReport;
	uint8_t totalMouseDistance = 0;
	
	uint16_t timerTopValue;
	uint16_t maxRate;
	
	uint8_t rateLimiterSwitch;
		
	// Only process the mouse if a mouse is attached to the USB port
	if (USB_HostState != HOST_STATE_Configured)	return;

	// Get the rate limiter configuration switch state
	rateLimiterSwitch = getRateLimiterState();

	// Select mouse data pipe
	Pipe_SelectPipe(MOUSE_DATA_IN_PIPE);

	// Unfreeze mouse data pipe
	Pipe_Unfreeze();

	// Has a packet arrived from the USB device?
	if (!(Pipe_IsINReceived())) {
		// Nothing received, exit...
		Pipe_Freeze();
		return;
	}

	// Ensure pipe is in the correct state before reading
	if (Pipe_IsReadWriteAllowed()) {
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
		// movement units in the previous direction.
		if (MouseReport.X > 0 && mouseDirectionX == 0) mouseDistanceX = 0;
		if (MouseReport.X < 0 && mouseDirectionX == 1) mouseDistanceX = 0;
		if (MouseReport.Y > 0 && mouseDirectionY == 0) mouseDistanceY = 0;
		if (MouseReport.Y < 0 && mouseDirectionY == 1) mouseDistanceY = 0;
		
		// Process mouse X movement -------------------------------------------
		if (MouseReport.X != 0) {
			if (MouseReport.X > 0) {
				// Set the mouse direction to incrementing
				mouseDirectionX = 1;
				mouseDistanceX += MouseReport.X;
			} else {
				// Set the mouse direction to decrementing
				mouseDirectionX = 0;
				mouseDistanceX += abs(MouseReport.X);
			}
			
			// Apply the lag limit
			if (rateLimiterSwitch == 0) {
				if (mouseDistanceX > (127 + FAST_LAGLIMIT)) mouseDistanceX = 127;
			} else {
				if (mouseDistanceX > (127 + SLOW_LAGLIMIT)) mouseDistanceX = 127;
			}
			
			// mouseDistanceX can exceed 127 due to lag caused by processing delays.  Here we ensure
			// that the quadrature is output is relative to the mouseDistanceX value unless 
			// mouseDistanceX is > 127 - in that case we always output at full speed.  This calculation
			// is based on each received report.
			if (mouseDistanceX > 127) totalMouseDistance = 127; else totalMouseDistance = mouseDistanceX;
			
			// Calculate the required timer TOP value based on the amount of mouse movement queued for output
			timerTopValue = 255 - ((totalMouseDistance * 2) + 1);
			
			// Calculate the minimum allowed value of timerTopValue:
			//
			// The RATELIMIT is the maximum hertz of the quadrature output
			//
			// We have 62,500 timer ticks per second (1,000,000 / 16 uS)
			// We need 4 timer ticks per quadrature output, 62,500 / 4 = 15,625
			// 15,625 / RATELIMIT gives the setting of TOP at the maximum allowed rate
			//
			// For example:
			// RATELIMIT = 256Hz therefore 15,625 / 256 = 61
			// RATELIMIT = 512Hz therefore 15,625 / 512 = 31
			// 
			// Since the range of TOP is 0-255, we also need to subtract 1 from the result
			if (rateLimiterSwitch == 0) maxRate = (15625 / FAST_RATELIMIT) - 1;
			else maxRate = (15625 / SLOW_RATELIMIT) - 1;
						
			// If the maximum output rate is exceeded, limit the rate to the maximum
			if (timerTopValue < maxRate) timerTopValue = maxRate;
			
			// Range check the timer TOP value
			if (timerTopValue > 255) timerTopValue = 255;
			
			// Set the interrupt speed (timer TOP)
			OCR0A = (uint8_t)timerTopValue;
		}
		
		// Process mouse Y ----------------------------------------------------
		if (MouseReport.Y != 0) {
			if (MouseReport.Y > 0) {
				// Set the mouse direction to incrementing
				mouseDirectionY = 1;
				mouseDistanceY += MouseReport.Y;
				
			} else {
				// Set the mouse direction to decrementing
				mouseDirectionY = 0;
				mouseDistanceY += abs(MouseReport.Y);
			}
			
			// Apply the lag limit
			if (rateLimiterSwitch == 0) {
				if (mouseDistanceY > (127 + FAST_LAGLIMIT)) mouseDistanceY = 127;
				} else {
				if (mouseDistanceY > (127 + SLOW_LAGLIMIT)) mouseDistanceY = 127;
			}
			
			// mouseDistanceY can exceed 127 due to lag caused by processing delays.  Here we ensure
			// that the quadrature is output is relative to the mouseDistanceX value unless
			// mouseDistanceY is > 127 - in that case we always output at full speed.  This calculation
			// is based on each received report.
			if (mouseDistanceY > 127) totalMouseDistance = 127; else totalMouseDistance = mouseDistanceY;
			
			// Calculate the required timer TOP value based on the amount of mouse movement queued for output
			timerTopValue = 255 - ((totalMouseDistance * 2) + 1);
			
			// Calculate the minimum allowed value of timerTopValue (see X calculation for notes)
			if (rateLimiterSwitch == 0) maxRate = (15625 / FAST_RATELIMIT) - 1;
			else maxRate = (15625 / SLOW_RATELIMIT) - 1;
			
			// If the maximum output rate is exceeded, limit the rate to the maximum
			if (timerTopValue < maxRate) timerTopValue = maxRate;
			
			// Range check the timer TOP value
			if (timerTopValue > 255) timerTopValue = 255;
			
			// Set the interrupt speed
			OCR2A = (uint8_t)timerTopValue;
		}
		
		// Process mouse buttons ----------------------------------------------
		
		// Check for left mouse button
		if ((MouseReport.Button & 0x01) == 0) LB_PORT |= LB; // Button on
		else LB_PORT &= ~LB; // Button off
			
		// Check for middle mouse button
		if ((MouseReport.Button & 0x04) == 0) MB_PORT |= MB; // Button on
		else MB_PORT &= ~MB; // Button off
			
		// Check for right mouse button
		if ((MouseReport.Button & 0x02) == 0) RB_PORT |= RB; // Button on
		else RB_PORT &= ~RB; // Button off
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
	puts_P(PSTR(ESC_FG_GREEN "USB Device attached\r\n" ESC_FG_WHITE));
}

// Event handler for the USB_DeviceUnattached event. This indicates that a device has been removed from the host, and
// stops the library USB task management process.
void EVENT_USB_Host_DeviceUnattached(void)
{
	puts_P(PSTR(ESC_FG_GREEN "USB Device detached\r\n" ESC_FG_WHITE));
}

// Event handler for the USB_DeviceEnumerationComplete event. This indicates that a device has been successfully
// enumerated by the host and is now ready to be used by the application.
void EVENT_USB_Host_DeviceEnumerationComplete(void)
{
	puts_P(PSTR("Getting configuration data from device...\r\n"));

	uint8_t ErrorCode;

	/* Get and process the configuration descriptor data */
	if ((ErrorCode = ProcessConfigurationDescriptor()) != SuccessfulConfigRead) {
		if (ErrorCode == ControlError) {
			puts_P(PSTR(ESC_FG_RED "Control Error (Get configuration)!\r\n"));
		} else {
			puts_P(PSTR(ESC_FG_RED "Invalid Device!\r\n"));
		}

		printf_P(PSTR(" -- Error Code: %d\r\n" ESC_FG_WHITE), ErrorCode);
		return;
	}

	// Set the device configuration to the first configuration (rarely do devices use multiple configurations)
	if ((ErrorCode = USB_Host_SetDeviceConfiguration(1)) != HOST_SENDCONTROL_Successful) {
		printf_P(PSTR(ESC_FG_RED "Control Error (Set configuration)!\r\n"
		                         " -- Error Code: %d\r\n" ESC_FG_WHITE), ErrorCode);
		return;
	}
	
	// HID class request to set the mouse protocol to the Boot Protocol
	USB_ControlRequest = (USB_Request_Header_t) {
			.bmRequestType = (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE),
			.bRequest      = HID_REQ_SetProtocol,
			.wValue        = 0,
			.wIndex        = 0,
			.wLength       = 0,
	};

	// Select the control pipe for the request transfer
	Pipe_SelectPipe(PIPE_CONTROLPIPE);

	// Send the request, display error and wait for device detach if request fails
	if ((ErrorCode = USB_Host_SendControlRequest(NULL)) != HOST_SENDCONTROL_Successful) {
		printf_P(PSTR(ESC_FG_RED "Control Error (Set protocol)!\r\n"
								 " -- Error Code: %d\r\n" ESC_FG_WHITE), ErrorCode);

		USB_Host_SetDeviceConfiguration(0);
		return;
	}

	puts_P(PSTR("USB Mouse enumeration successful\r\n"));
}

// Event handler for the USB_HostError event. This indicates that a hardware error occurred while in host mode.
void EVENT_USB_Host_HostError(const uint8_t ErrorCode)
{
	USB_Disable();

	printf_P(PSTR(ESC_FG_RED "Host Mode Error!\r\n"
	                       " -- Error Code %d\r\n" ESC_FG_WHITE), ErrorCode);

	while(1);
}

// Event handler for the USB_DeviceEnumerationFailed event. This indicates that a problem occurred while
// enumerating an attached USB device.
void EVENT_USB_Host_DeviceEnumerationFailed(const uint8_t ErrorCode,
                                            const uint8_t SubErrorCode)
{
	printf_P(PSTR(ESC_FG_RED "Device Enumeration Error!\r\n"
	                         " -- Error Code %d\r\n"
	                         " -- Sub Error Code %d\r\n"
	                         " -- In State %d\r\n" ESC_FG_WHITE), ErrorCode, SubErrorCode, USB_HostState);
}


