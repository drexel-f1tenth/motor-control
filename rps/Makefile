
PORT ?= /dev/ttyACM0
# BOARD ?= arduino:avr:mega
BOARD ?= SparkFun:avr:promicro:cpu=16MHzatmega32U4
ADDITIONAL_URLS ?= --additional-urls "https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json"

.PHONY: clean deps flash run

deps:
	arduino-cli core update-index $(ADDITIONAL_URLS)
	arduino-cli core install SparkFun:avr $(ADDITIONAL_URLS)

C_SRC = $(wildcard *.ino) $(wildcard *.h)
build: $(C_SRC)
	arduino-cli compile -p $(PORT) -b $(BOARD) -v --warnings more

flash: build
	arduino-cli upload -p $(PORT) -b $(BOARD) -v

run: flash
	rosrun rosserial_arduino serial_node.py _port:=$(PORT)

clean:
	rm -rf build
