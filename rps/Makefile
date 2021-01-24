
PORT := /dev/ttyACM0
BOARD := arduino:avr:mega

.PHONY: clean flash run

build: rps.ino
	arduino-cli compile -p $(PORT) -b $(BOARD) -v

flash: build
	arduino-cli upload -p $(PORT) -b $(BOARD) -v

clean:
	rm -rf build
