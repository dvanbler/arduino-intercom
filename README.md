# An Arduino UNO R4 Wifi Intercom

During normal operation, the device plays audio streamed to it over the network. While a button remains pressed, it switches to a mode that streams mic audio back to the last ip address that streamed audio to it.

## Stock Wifi Throughput Limitations

By default, the serial link between the ESP32 and the R4 is hard-coded to 115200 baud and cannot be changed in code. This allows for a maximum wifi throughput of approximately 9 KB/s. In practice, this means network audio streaming is limited to 8 kHz, with 8 bits per sample.

In order to stream 32 khz 8-bit audio, it is necessary to modify both the ESP32 firmware, and the Arduino Renesas WiFiS3 library. Both sides need to have the serial link baud rate configured to the same value, in this case 921600.

A test of maximum network throughput produced these reults:

| Baud      | Wifi Receive B/s | Wifi Send B/s |
|-----------|------------------|---------------|
| 115,200   | 9,169            | 8,689         |
| 921,600   | 46,027           | 53,383        |

Setting the baud rate to 921,600 allows for streaming 32 kHz 8-bit audio.

See the forked [uno-r4-wifi-usb-bridge](https://github.com/arduino/uno-r4-wifi-usb-bridge/commit/2fa3b34fadbd676eb7b6deea5ee43953ade71df3) and [ArduinoCore-renesas](https://github.com/arduino/ArduinoCore-renesas/commit/d704f0d362f324f21992a7fbc5ddee20b230cc98) repositories for the necessary changes.
