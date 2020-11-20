# hackintosh-g5-cooler

We ripped a defect Apple PowerMac G5 apart, keeping only the case and rear fans. We use a water tank, an aquarium pump and aquarium components to drip water down some foam material. We let the fans blow over the moist foam and expect a decent cooling effect from that. Air flow passes through the G5 case.

We use an ESP8266 microcontroller to control fan and pump speed, to read environmental data from a temperature/humidity sensor and to control start button and LED(s).

We use the WiFi features of the ESP8266 to provide a simple HTTP server to set fan/pump speed and to display sensor values. Further integration with smart home projects is planned.
