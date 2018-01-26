# WVSU-NASA-2018
This project contains code for a Raspberry Pi and an external microcontroller. The Raspberry Pi will process images captured from a Michelson Interferometer, communicate with the microcontroller, and transmit data over an telemetry line with its UART port. The microcontroller code will monitor and manage geiger circuits and an IMU. The first flight will be late 2018 as part of NASA's RockSat-X.

The `interferometer` folder can be built with CMake, and the `microcontroller` folder can be built with PlatformIO, but the source should be compatible with the Arduino IDE if you prefer to use it. Just copy and paste it into the Arduino IDE.
