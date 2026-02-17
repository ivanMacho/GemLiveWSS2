# GemLiveWSS2

This repository contains the source code for the GemLiveWSS2 project, a small
application targeting the ESP32 microcontroller. The firmware demonstrates how
to connect to a WebSocket server and take advantage of the Gemini API using an
API key. It is written using PlatformIO and is intended to run on an ESP32-based
board (ESP32-S3 in the current configuration).

## Features

- WebSocket client functionality
- Integration with Gemini API via HTTP requests
- Configurable pin assignments for peripherals (you may need to adjust them
  depending on your hardware setup)

## Getting Started

### Requirements

- ESP32 development board (ESP32-S3 recommended)
- PlatformIO installed in VS Code or another compatible environment
- A valid Gemini API key

### Configuration

Before building and uploading the firmware, you need to create a header file
with your Gemini API key. In the `include` directory, create a file named
`secrets.h` with the following contents:

```cpp
#ifndef SECRETS_H
#define SECRETS_H

const char* GEMINI_API_KEY = "YOUR_API_KEY";

#endif
```

Replace `YOUR_API_KEY` with your actual Gemini API key. Without this, the
program will not be able to authenticate with the Gemini service.

### Pin Assignments

The code is written for an ESP32 device, but pin assignments for things like
displays, sensors, or buttons may vary between boards. You should review the
`src/main.cpp` file and modify any `#define` or variable values related to GPIO
pins to match your hardware.

### Building and Uploading

Use PlatformIO to build and upload the firmware:

```sh
platformio run --environment esp32-s3-xh-s3e --target upload
```

or use the PlatformIO extension in VS Code to select the appropriate
environment and upload.

## License

This project is provided as-is with no warranty. Feel free to use and modify
it for your own projects.

---

_Notes:_

- Designed for ESP32; pin changes might be necessary depending on your board.
- Ensure `include/secrets.h` is created with your Gemini API key as described
  above.