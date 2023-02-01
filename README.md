# i2c-dummy-device on ESP32-C3

## What is this?

This simulates an I2C device via bit-banging.

It ACKs on all 7-bit addresses and returns the data `b"012345678901234567890123456789"` for every register.

The code is intentionally not fancy - it's not even using interrupts.

When using it with e.g. an SSD1306 display driver you *need* to disable the console output. (`VERBOSE`) - also always run in release mode.

SDA is on GPIO 1, SCL is on GPIO 2.

Additionally it can simulate clock-stetching by changing `CLOCK_STRETCH_DELAY_US`

If it doesn't work you might need to add external pull-up resistors.

This is really just for testing purposes - it might crash or act weird in many ways.
