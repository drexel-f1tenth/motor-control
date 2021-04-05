PORT ?= /dev/ttyACM0
BOARD ?= arduino:avr:mega
# BOARD ?= SparkFun:avr:promicro:cpu=16MHzatmega32U4
ADDITIONAL_URLS ?= --additional-urls "https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json"
DEPS_LIBS ?= \
	"SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library" \
	servo

.PHONY: clean flash run

all: run

deps:
	arduino-cli core update-index $(ADDITIONAL_URLS)
	arduino-cli core install SparkFun:avr $(ADDITIONAL_URLS)
	arduino-cli lib install $(DEPS_LIBS)
	rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries

C_SRC = $(shell find mcu -name '*.ino' -o -name '*.h')
build: $(C_SRC)
	arduino-cli compile -b $(BOARD) mcu \
		--warnings all --libraries ros_lib --output-dir build \
		--build-properties build.extra_flags="-std=c++17"

flash: build
	arduino-cli upload -p $(PORT) -b $(BOARD) \
		-v --input-dir build

run: flash
	rosrun rosserial_arduino serial_node.py _port:=$(PORT)

clean:
	rm -rf build
