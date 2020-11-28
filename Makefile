CC=avr-gcc
OBJCOPY=avr-objcopy
OBJS=SmallyMouse2/main.o SmallyMouse2/ConfigDescriptor.o SmallyMouse2/src/LUFA/LUFA/Drivers/USB/Core/USBTask.o SmallyMouse2/src/LUFA/LUFA/Drivers/USB/Core/AVR8/USBController_AVR8.o SmallyMouse2/src/LUFA/LUFA/Drivers/USB/Core/AVR8/USBInterrupt_AVR8.o SmallyMouse2/src/LUFA/LUFA/Drivers/USB/Core/HostStandardReq.o SmallyMouse2/src/LUFA/LUFA/Drivers/USB/Core/ConfigDescriptors.o SmallyMouse2/src/LUFA/LUFA/Drivers/USB/Core/AVR8/Pipe_AVR8.o SmallyMouse2/src/LUFA/LUFA/Drivers/USB/Core/AVR8/PipeStream_AVR8.o SmallyMouse2/src/LUFA/LUFA/Drivers/USB/Core/AVR8/Host_AVR8.o SmallyMouse2/src/LUFA/LUFA/Drivers/Peripheral/AVR8/Serial_AVR8.o SmallyMouse2/src/LUFA/LUFA/Drivers/USB/Core/AVR8/Device_AVR8.o SmallyMouse2/src/LUFA/LUFA/Drivers/USB/Core/AVR8/Endpoint_AVR8.o SmallyMouse2/src/LUFA/LUFA/Drivers/USB/Core/Events.o

CFLAGS=-Os -DUSE_LUFA_CONFIG_HEADER -DARCH=ARCH_AVR8 -DBOARD=BOARD_NONE -DF_CPU=16000000UL -DF_USB=16000000UL -D__AVR_AT90USB1287__ -mmcu=at90usb1287 -ISmallyMouse2/src/ -ISmallyMouse2/src/config/ -ISmallyMouse2/src/LUFA/
PORT=/dev/ttyACM0

smallymouse.hex: smallymouse.elf
	${OBJCOPY} -j .text -j .data -O ihex smallymouse.elf smallymouse.hex

smallymouse.elf: ${OBJS}
	${CC} -mmcu=at90usb1287 -o smallymouse.elf ${OBJS}

install: smallymouse.hex
  dfu-programmer at90usb1287 erase --force
	dfu-programmer at90usb1287 flash smallymouse.hex
