
PORT ?= /dev/ttyACM0
BOARD ?= arduino:avr:mega

.PHONY: clean flash run

C_SRC = $(wildcard *.ino) $(wildcard *.h)

build: $(C_SRC)
	arduino-cli compile -p $(PORT) -b $(BOARD) -v

flash: build
	arduino-cli upload -p $(PORT) -b $(BOARD) -v

run: flash
	rosrun rosserial_arduino serial_node.py _port:=$(PORT)

clean:
	rm -rf build
