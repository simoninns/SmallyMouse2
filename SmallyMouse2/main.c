/************************************************************************
	main.c

	Main functions
    SmallyMouse2 - USB to quadrature mouse converter
    Copyright (C) 2017-2020 Simon Inns

	This file is part of SmallyMouse2.

    SmallyMouse2 is free software: you can redistribute it and/or modify
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

// Important notice:  If you port this firmware to another hardware design
// it *will be* a derivative of the original and therefore you must pay attention
// to the CC hardware licensing and release your hardware design as per the share-
// alike license.  Keep it open! ...and yes, that includes you Commodore chaps.

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

#define MOUSEX	0
#define MOUSEY	1

// Configuration ------------------------------------------------------------------------------------------------------

// Quadrature output frequency limit
//
// This setting limits the maximum frequency of the quadrature output towards the retro computer.
// If the rate of output is too high the retro computer cannot 'count' the input accurately and
// this causes spurious mouse movement.
//
// With no rate limit the quadrature output speed will reach a maximum of 3,906.25 Hz
// For 8-bit machines it is recommended that the speed doesn't exceed 500 Hz.  This limit is
// only applied if the 'slow' configuration jumper is shorted (i.e. on)
#define Q_RATELIMIT 500

// Quadrature output buffer limit
//
// Since the slow rate limit will prevent the quadrature output keeping up with the USB movement
// report input, the quadrature output will lag behind (i.e. the quadrature mouse will continue
// to move after the USB mouse has stopped).  This setting limits the maximum number of buffered
// movements to the quadrature output.  If the buffer reaches this value further USB movements
// will be discarded
#define Q_BUFFERLIMIT 300

// DPI Divider
//
// Some USB mice have very high DPI which causes the quadrature rate to be too high (making the
// mouse move too fast).  If the DPISW header is shorted the following constant will be used to 
// divide the DPI rate to slow things down.  2 or 3 are reasonable values.
#define DPI_DIVIDER 2

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

volatile uint8_t xTimerTop = 0;				// X axis timer TOP value
volatile uint8_t yTimerTop = 0;				// Y axis timer TOP value

// Interrupt Service Routine based on Timer0 for mouse X movement quadrature output
ISR(TIMER0_COMPA_vect)
{
	// Process X output
	if (mouseDistanceX > 0) {
		// Set the output pins according to the current phase
		if (mouseEncoderPhaseX == 0) X1_PORT |=  X1;	// Set X1 to 1
		if (mouseEncoderPhaseX == 1) X2_PORT |=  X2;	// Set X2 to 1
		if (mouseEncoderPhaseX == 2) X1_PORT &= ~X1;	// Set X1 to 0
		if (mouseEncoderPhaseX == 3) X2_PORT &= ~X2;	// Set X2 to 0

		// Change phase
		if (mouseDirectionX == 0) mouseEncoderPhaseX--; else mouseEncoderPhaseX++;
		
		// Decrement the distance left to move
		mouseDistanceX--;

		// Range check the phase
		if ((mouseDirectionX == 1) && (mouseEncoderPhaseX > 3)) mouseEncoderPhaseX = 0;
		if ((mouseDirectionX == 0) && (mouseEncoderPhaseX < 0)) mouseEncoderPhaseX = 3;
	}
	
	// Set the timer top value for the next interrupt
	OCR0A = xTimerTop;
}

// Interrupt Service Routine based on Timer2 for mouse Y movement quadrature output
ISR(TIMER2_COMPA_vect)
{
	// Process Y output
	if (mouseDistanceY > 0) {
		// Set the output pins according to the current phase
		if (mouseEncoderPhaseY == 3) Y1_PORT &= ~Y1;	// Set Y1 to 0
		if (mouseEncoderPhaseY == 2) Y2_PORT &= ~Y2;	// Set Y2 to 0
		if (mouseEncoderPhaseY == 1) Y1_PORT |=  Y1;	// Set Y1 to 1
		if (mouseEncoderPhaseY == 0) Y2_PORT |=  Y2;	// Set Y2 to 1

		// Change phase
		if (mouseDirectionY == 0) mouseEncoderPhaseY--; else mouseEncoderPhaseY++;
		
		// Decrement the distance left to move
		mouseDistanceY--;

		// Range check the phase
		if ((mouseDirectionY == 1) && (mouseEncoderPhaseY > 3)) mouseEncoderPhaseY = 0;
		if ((mouseDirectionY == 0) && (mouseEncoderPhaseY < 0)) mouseEncoderPhaseY = 3;
	}
	
	// Set the timer top value for the next interrupt
	OCR2A = yTimerTop;
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
	
	// Set mouse button output pins to on
	// Note: Mouse buttons are inverted, so this sets them to 'off'
	//       from the host's perspective
	LB_PORT |= LB; // Pin = 1 (on)
	MB_PORT |= MB; // Pin = 1 (on)
	RB_PORT |= RB; // Pin = 1 (on)

	// Set the rate limit configuration header to input
	RATESW_DDR &= ~RATESW; // Input
	RATESW_PORT |= RATESW; // Turn on weak pull-up
	
	// Configure E7 on the expansion header to act as
	// DPISW (since it is easily jumpered to 0V)	
	DPISW_DDR &= ~DPISW; // Input
	DPISW_PORT |= DPISW; // Turn on weak pull-up
	
	// Initialise the expansion (Ian) header
	E0_DDR |= ~E0; // Output
	E1_DDR |= ~E1; // Output
	E2_DDR |= ~E2; // Output
	E3_DDR |= ~E3; // Output
	E4_DDR |= ~E4; // Output
	E5_DDR |= ~E5; // Output
	E6_DDR |= ~E6; // Output
	
	E0_PORT &= ~E0; // Pin = 0
	E1_PORT &= ~E1; // Pin = 0
	E2_PORT &= ~E2; // Pin = 0
	E3_PORT &= ~E3; // Pin = 0
	E4_PORT &= ~E4; // Pin = 0
	E5_PORT &= ~E5; // Pin = 0
	E6_PORT &= ~E6; // Pin = 0
	
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
	puts_P(PSTR(ESC_FG_YELLOW "(c)2017-2020 Simon Inns\r\n" ESC_FG_WHITE));
	puts_P(PSTR(ESC_FG_YELLOW "http://www.waitingforfriday.com\r\n" ESC_FG_WHITE));
	
	// Now report the status of the various configuration switches
	if ((RATESW_PIN & RATESW) == 0) puts_P(PSTR("Rate limit switch is ON\r\n"));
	else puts_P(PSTR("Rate limit switch is OFF\r\n"));
	if ((DPISW_PIN & DPISW) == 0) puts_P(PSTR("DPI divide switch is ON\r\n"));
	else puts_P(PSTR("DPI divide switch is OFF\r\n"));
}

// Initialise the ISR timers
void initialiseTimers(void)
{
	// The frequency of the reports from the USB mouse is about 100-125 Hz (i.e.
	// 100-125 reports per second).  Each report can indicate up to 127 position
	// movements in each direction (X & Y)
	
	// If we can receive 127 movements per report at a rate of 125 Hz
	// then the maximum movement per second is 100-125 Hz * 127 movements =
	// 12,700 - 15,875 movements per second (i.e. the quadrature output has 
	// to be 12,700 - 15,875 Hz to keep up)
	
	// The setting of the timers controls the maximum interrupt speed and 
	// therefore the maximum possible quadrature rate output.
	// 
	// Each movement unit reported by the USB device causes a single phase
	// change in the quadrature output (which means there is a 4:1 ratio
	// between the USB movement units and the quadrature movement output)
	
	// Timer0 and Timer2 are 8-bit timers.  Therefore the slowest interrupt
	// speed possible is 256 times the tick period and the fastest is a single
	// tick period.
	
	// Timer prescale is /1024 and the AVR clock speed is 16,000,000 Hz
	// 16,000,000 Hz / 1024 prescale = 15,625 ticks per second
	//
	// 1,000,000 uS per second / 15,625 ticks per second = 64 uS per tick
	//
	// 15,625 ticks (with a 4:1 ratio) means the maximum quadrature output
	// will be 3,906.25 Hz.

	// Configure Timer0 to interrupt (8-bit timer)
	OCR0A = 0; // In CTC mode OCR0A = TOP
	TCCR0A = (1 << WGM01); // Clear timer on compare match (CTC) mode
	TCCR0B = 5; //  CS02/CS01/CS00 = binary 101 = /1024 prescale
	TIMSK0 = (1 << OCIE0A); // Timer0 interrupt enabled
		
	// Configure Timer2 to interrupt (8-bit timer)
	OCR2A = 0; // In CTC mode OCR2A = TOP
	TCCR2A = (1 << WGM21); // Clear timer on compare match (CTC) mode
	TCCR2B = 7; // CS22/CS21/CS20 = binary 111 = /1024 prescale
	TIMSK2 = (1 << OCIE2A); // Timer2 interrupt enabled
}

// Read the mouse USB report and process the information
void processMouse(void)
{
	USB_MouseReport_Data_t MouseReport;
	bool limitRate = false;
	bool dpiDivide = false;
		
	// Only process the mouse if a mouse is attached to the USB port
	if (USB_HostState != HOST_STATE_Configured)	return;
	
	// Get the state of the rate limiting (slow) header
	if ((RATESW_PIN & RATESW) == 0) limitRate = true;
	
	// Get the state of the DPI divider header
	if ((DPISW_PIN & DPISW) == 0) dpiDivide = true;
	
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
		// Set USB report processing activity on expansion port pin D0
		// Note: This can be used to analyze the rate at which USB mouse
		// reports are received by measuring the rate using an oscilloscope
		// or frequency counter on D0
		E0_PORT |= E0; // Pin = 1
		
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
		if (MouseReport.X != 0) xTimerTop = processMouseMovement(MouseReport.X, MOUSEX, limitRate, dpiDivide);
		if (MouseReport.Y != 0) yTimerTop = processMouseMovement(MouseReport.Y, MOUSEY, limitRate, dpiDivide);
		
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
		
		// Clear USB report processing activity on expansion port pin D0
		E0_PORT &= ~E0; // Pin = 0
	}

	// Clear the IN endpoint, ready for next data packet
	Pipe_ClearIN();

	// Refreeze mouse data pipe
	Pipe_Freeze();
}

// Process the mouse movement units from the USB report
uint8_t processMouseMovement(int8_t movementUnits, uint8_t axis, bool limitRate, bool dpiDivide)
{
	uint16_t timerTopValue = 0;
	
	// Set the mouse movement direction and record the movement units
	if (movementUnits > 0) {
		// Moving in the positive direction
		
		// Apply DPI limiting if required
		if (dpiDivide) {
			movementUnits /= DPI_DIVIDER;
			if (movementUnits < 1) movementUnits = 1;
		}
		
		// Set the mouse direction to incrementing
		if (axis == MOUSEX) mouseDirectionX = 1; else mouseDirectionY = 1;
		
		// Add the movement units to the quadrature output buffer
		if (axis == MOUSEX) mouseDistanceX += movementUnits;
		else mouseDistanceY += movementUnits;
	} else {
		// Moving in the negative direction
		
		// Apply DPI limiting if required
		if (dpiDivide) {
			movementUnits /= DPI_DIVIDER;
			if (movementUnits > -1) movementUnits = -1;
		}
		
		// Set the mouse direction to decrementing
		if (axis == MOUSEX) mouseDirectionX = 0; else mouseDirectionY = 0;
		
		// Add the movement units to the quadrature output buffer
		if (axis == MOUSEX) mouseDistanceX += abs(movementUnits);
		else mouseDistanceY += abs(movementUnits);
	}
	
	// Apply the quadrature output buffer limit
	if (axis == MOUSEX) {
		if (mouseDistanceX > Q_BUFFERLIMIT) mouseDistanceX = Q_BUFFERLIMIT;
	} else {
		if (mouseDistanceY > Q_BUFFERLIMIT) mouseDistanceY = Q_BUFFERLIMIT;
	}
	
	// Get the current value of the quadrature output buffer
	if (axis == MOUSEX) timerTopValue = mouseDistanceX;
	else timerTopValue = mouseDistanceY;
	
	// Range check the quadrature output buffer
	if (timerTopValue > 127) timerTopValue = 127;
	
	// Since the USB reports arrive at 100-125 Hz (even if there is only
	// a small amount of movement, we have to output the quadrature
	// at minimum rate to keep up with the reports (otherwise it creates
	// a slow lag).  If we assume 100 Hz of reports then the following
	// is true:
	//
	// 127 movements = 12,700 interrupts/sec
	// 100 movements = 10,000 interrupts/sec
	//  50 movements =  5,000 interrupts/sec
	//  10 movements =  1,000 interrupts/sec
	//   1 movement  =    100 interrupts/sec
	//
	// Timer speed is 15,625 ticks per second = 64 uS per tick
	//
	// Required timer TOP values (0 is fastest so all results are x-1):
	// 1,000,000 / 12,700 = 78.74 / 64 uS = 1.2 - 1
	// 1,000,000 / 10,000 = 100 / 64 uS = 1.56 - 1
	// 1,000,000 / 5,000 = 200 / 64 uS = 3.125 - 1
	// 1,000,000 / 1,000 = 1000 uS / 64 uS = 15.63 - 1
	// 1,000,000 / 100 = 10000 uS / 64 uS = 156.25 - 1
	//
	// So:
	//   timerTopValue = 10000 / timerTopValue; // i.e. 1,000,000 / (timerTopValue * 100)
	//   timerTopValue = timerTopValue / 64;
	//   timerTopValue = timerTopValue - 1;
	
	timerTopValue = ((10000 / timerTopValue) / 64) - 1;
	
	// If the 'Slow' configuration jumper is shorted; apply the quadrature rate limit
	if (limitRate) {
		// Rate limit is on
		
		// Rate limit is provided in hertz
		// Each timer tick is 64 uS
		//
		// Convert hertz into period in uS
		// 1500 Hz = 1,000,000 / 1500 = 666.67 uS
		// 
		// Convert period into timer ticks (* 4 due to quadrature)
		// 666.67 us / (64 * 4) = 2.6 ticks
		//
		// Timer TOP is 0-255, so subtract 1
		// 10.42 ticks - 1 = 9.42 ticks
		
		uint32_t rateLimit = ((1000000 / Q_RATELIMIT) / 256) - 1;
		
		// If the timerTopValue is less than the rate limit, we output
		// at the maximum allowed rate.  This will cause addition lag that
		// is handled by the quadrature output buffer limit above.
		if (timerTopValue < (uint16_t)rateLimit) timerTopValue = (uint16_t)rateLimit;
	}
	
	// Return the timer TOP value
	return (uint8_t)timerTopValue;
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


