## Project Goal

...

## Apple G5 Case

...

## Apple G5 Fans

The Apple G5 fans (at least the rear ones that we care about) are different from typical PC fans: they are voltage driven instead of PWM driven. This means fan speed is controlled by a voltage level between around 2.5V to 12.5V at the fan speed pin, rather than providing a pulsed 12V signal.

The fans have 4 wires each (Vcc, GND, speed, sense). The two rear fans share a 6pin connector, where the Vcc and GND pins are used by both fans. They are brushless DC fans and probably have some internal circuitry (we did not open them), so a fixed Vcc voltage needs to be provided. That would probably be 12V, we have used 15V Vcc without issues. It did not work with only 5V on the fans Vcc pins.

Fan speed is controlled by variable voltage on the fans speed pins. Fans start to spinup at >2.5V and can be driven to >12V for full speed. Fan speed increases linear with speed voltage. The fan speed connection also needs to provide enough current. We tried to use a voltage divider trimmer pot, but the fans would not move in most pot positions because the resistor was limiting the current on the speed pin. We have not measured the current on the rear fans, but a rough guess would be that at least 100mA need to be provided on the fan speed pin. We use a LM317 regulator to provide the required voltage and current to the fan speed pin.

We have not used the fans rotation speed sense pins yet.

## Microcontroller

#### Design Rev. 1 (Arduino, no WiFi, no OTA)

Initially, we designed the system to use standard Arduino boards (Arduino Uno, Nano or Pro Mini with ATmega328 processor). It might even work with limited functionality on the smaller ATTiny boards, but these boards do not have enough GPIO pins for our purposes, so extra port multiplexer components would have to be added. We did not test this yet. It should work with minor adjustments on bigger Arduino boards providing more ports (such as Arduino Mega). Untested because we don't have these boards.

Most Arduino boards do not have builtin WiFi. One way to add WiFi support to an Arduino would be to connect it to a small ESP8266-01 board. The Arduino can connect to a WiFi network by treating the ESP as a modem and set `AT` commands over a software serial line. This provides WiFI IP connectivity, but we need more work to implement an HTTP server. And it seems that *over-the-air* software updates are not possible. We did not follow that route.

We use PWM output from the MCU to drive two LM317 regulators to control the fan speed and water pump power. The MCU PWM signal gets converted into a (more or less) constant voltage, with gets amplified to the required level using an LM358 OpAmp. We have also experimented with digital potentiometers (MCP4162) to drive the LM317 output voltages, but these devices are rather expensive compared to the PWM/lowpass/OpAmp circuit. They will also need 5 pins for SPI. We did not follow that route further.

#### Design Rev. 2 (ESP8266, WiFi, OTA, Sensors, Front Panel)

After some redesign we wanted to support WiFi, so the system could interface to home automation systems or be controlled by a mobile app. The obvious choice was using a board from the ESP8266 family, which include WiFi hardware. We used an ESP-12 board (Wemos D1, similar boards such as NodeMCU should work as well). The small ESP-01 boards might work with very limited functionality, as they need further port multiplexer components. We did not test this yet.

With the Rev2 design we chose to add two environment sensors (temperature/humidity) near both airflow inlet and outlet. Which requires 2 more pins (for DHT22 sensors) or using the I2C bus (for AM2320 sensors), which needs 2 pins as well. We also added a water tank level sensor that outputs certain voltages depending on the water tank level, which needs an analog input to measure that voltage. ESP8266 MCUs have *exactly one* analog input pin.

We have experimented with voltage dividers to read and report voltage levels (5V MCU power rail, 15V rail, fan voltage, pump voltage). Each of these requires another analog input, so in order to use that we need an analog port multiplexer. A cheap 74HC4051 provides 8 analog input ports, but needs 4 MCU pins to read them. The more expensive ADS1115 units provide 8 analog ports over I2C, which might already be used for the AM2320 sensors, so no further pins are required.

In Rev2 we also support the Mac G5 front panel (on/off pushbutton with LED, additional 2-color status LED). For the on/off pushbutton to work, we need to provide a power latching circuit ourselves - it was implemented on the Mac mainboard which we removed.

Even with the ESP8266-12, we run out of ports (esp. analog ports), and need external port multiplexer devices.

#### Design Rev. 3 (ESP32, WiFi/BluetoothLE, OTA, Sensors, Front Panel)

[work in progress]

### Functionality provided by Microcontroller

The software running on the MCU shall provide the following functionality:

* control rear fans speed (voltage)
  * using either PWM or digital potentiometer
  * read/report fan voltage
* control water pump (voltage)
  * using either PWM or digital potentiometer
  * read/report pump voltage
* read/report/warn on water tank low/high
* read/report temperature and humidity
  * DHT22 sensor (alternatives: AM2320, SI7021, BME280)
  * support 2 sensors on airflow inlet (rear) / airflow outlet (front)
* keep G5 front panel
  * power button and power LED
  * USB as power supply for external devices (no USB data)
  * place additional LED or bicolor LED in audio connector
* provide a simple HTTP server to report status and change fan and water pump speed
* for debugging, support serial console output and commands
* support OTA (over-the-air) software updates

### MCU Pin Requirements

* PWM_FAN - fan speed (PWM)
* PWM_PUMP - pump speed (PWM)
* Digital1 - rear env sensor data (OneWire, for DHT22 only)
* Digital2 - front env sensor data (OneWire, for DHT22 only)
* Digital3 - software poweroff (TTL)
* <strike>Digital4 - front pushbutton LED (TTL)</strike> latchup circuit, no MCU pin needed
* Digital5 - front status green LED (TTL)  // if bicolor status LED used
* Digital6 - front status red LED (TTL)  // if bicolor status LED used
* Digital7 - 4051 MUX addr  // if >1 analog port used
* Digital8 - 4051 MUX addr  // if >1 analog port used
* Digital9 - 4051 MUX addr  // if >1 analog port used
* SPI_FAN - fan speed (SPI)  // if MCP4162 digipot used
* SPI_PUMP - pump speed (SPI)  // if MCP4162 digipot used
* SPI_MOSI - SPI bus  // if MCP4162 digipot used
* SPI_MISO - SPI bus  // if MCP4162 digipot used
* SPI_CLK - SPI bus  // if MCP4162 digipot used
* Analog1 - water tank level
* Analog2 - fan voltage
* Analog3 - pump voltage
* Analog4 - Dcc 15V rail voltage
* Analog5 - MCU 5V rail voltage

## Circuit

![Circuit](./Apple_Fan_PWM_Control.png)

### Power Supply

The Apple fans that we want to control are voltage controlled, fan speed can be between "none" (< 2.5V) and "max" (~ 12.5V). We will use an LM317 voltage regulator, which has a drop-out voltage of 2.5V. So in order to provide full 12V to the fans, the system needs to run at 15V Vcc.

We use an old notebook power supply providing 15V DC 4 Amp, which is more than enough for the project.

### MCU Power Supply

We use a LM7805 fixed voltage regulator to provide 5V Vcc to the microcontroller. Arduinos need 5V, while most ESP boards can be powered by 5V, even though they use 3.3V internally.

We planned to use a low-power 78L05 for this, as we did not expect the (Arduino) MCU to draw more than 100mA. However, the decision to use an ESP8266 MCU requires a more powerful voltage regulator, as these devices draw around 150mA in normal operation and are specified with a peak current draw of 450mA. We could have used an AMS1117 3.3 regulator that we had in the drawer, but we want a 5V power supply (to deliver 5V to the front panel USB connector).
Capacitors C1, C2 use recommended values from the datasheet on both Vin/Vout sides.

### Fan Power Supply

We drive the fan voltage (and thus fan speed) using a LM317 adjustable voltage regulator. The Vin capacitor is actually C1 from the MCU power supply, while the Vout capacitor C5 uses a value recommended in the datasheet. Resistor R2 should be 240 Ohm 1% tolerance according to the LM317 datasheet. We did not have that value in our toolbox and used 270 Ohm instead (there are sample circuits on the Internet that even use 330 or 470 Ohm, which might work but possibly break the spec of the chip).

D1 and D2 are optional protection diodes that protect the LM317 against short-circuits on both Vin and Vout sides.

### Fan Voltage Control by PWM

#### RC Lowpass

R1=100k and C3=100n build a lowpass to turn the PWM signal from the MCU into a (more or less) constant voltage. In reality, there remains a ripple voltage on the lowpass output that gets amplified and *might* lead to noticeable oscillations of fan speed.

The cut-off frequency of the lowpass is calculated as (1 / (2pi RC)). With the specified values, the cut-off frequency is around 16 Hz, which leaves some ripple voltage on the output when the MCU drives the PWM signal at its default PWM frequency (Arduino: 490 Hz, ESP8266: 1 kHz). Ripple can be reduced by using higher PWM frequencies than the default.

Ripple could be reduced further by lowering the cut-off frequency. Which means increasing R and/or C. We don't want to use elkos here, so C3 won't go much higher. We are hesitating to go higher with R1 as well. We could build a 2nd order RC lowpass by simply adding another resistor and capacitor, using the same R and C values.

Probably not worth the effort, as it can be improved in software by increasing the PWM frequency to >30 kHz, so the RC lowpass will be more efficient.

Ripple on the fan speed control voltage turned out to be irrelevant for the brushless DC fans we are using. There were no noticable oscillations of fan rotation.

#### OpAmp

The LM358 OpAmp is needed to amplify the PWM voltage after the RC lowpass to voltage levels required to drive the LM317.

With 15V Vcc and LM317 2.5V dropoff, we can get 12.5V Vout max. To get this Vout, we need to supply 11.25V to the LM317 Vadj pin. This should correspond to 100% PWM.

An Arduino powered from USB provides ~4.7V DC after the RC lowpass at 100% PWM. An ESP8266 uses 3.3V internally and provides ~3.2V after the RC lowpass at 100% PWM. A non-inverting OpAmp is used to amplify these voltage levels to 11.25V LM317 Vadj.

Amplification is controlled by the values of the (R3+R4)/R5 voltage divider. In our experiments, we used a fix value for R4 in series with a 5k trimmer pot to find the sweet spot.

Resistor values for Arduino: R5=10k, R4=15k, R3=0-5k; sweet spot R3+R4=18k.

Resistor values for ESP: R5=18k, R4=47k, R3=0-5k; sweet spot R3+R4=Xk.

### Fan Voltage Control by Digital Potentiometer

An alternative way to control fan voltage is to use a digital potentiometer rather than PWM to drive LM317 Vadj. The output voltage Vfans is controlled by the voltage divider R2/R3, which resembles example applications in the LM317 datasheet. This a has a few advantages:
* the circuit will be simpler with less component parts
* ripple is not an issue as there is no pulsed PWM signal
* not using PWM puts less strain on the MCU

It also has a major disadvantage: it is way more expensive. We payed around 4.50€ for the MCP4162, while the OpAmp, resitors and capacitors needed for PWM cost around 0.50€ altogether. So this is merely an academic exercise. Also, since the MCP4162 uses SPI, at least 4 more lines need to be connected.

One important thing to check is that the current through the digipot does not exceed the maximum value specified in the digipot's datasheet, otherwise the unit will get destroyed. A MCP4162 digipot allows a maximum current of 2.5mA through its "resistor" pins. According to LM317 datasheet, the maximum current flowing out of the ADJ pin to GND through the digipot will be <0.1mA, so we should be safe.

### Fan Voltage Measurement

Voltages can be measured with a MCU using analog input pin in combination with a voltage divider. Input voltage on analog pins must not exceed certain limits, so the voltage divider "scales" down the output voltage to be measured to acceptable values.

Details vary between Arduino and ESP8266 boards:

Arduinos have 8 analog input pins that can accept max 5V. For max 12.5V output voltage, good resistor values would be R6=33k / R7=22k. We chose R7=18k for some safety margin, so we can measure up to 14.1V without risk of damaging the Arduino analog input ports.

ESP8266-12 MCUs have only one analog input pin that can accept max 1V, while the small ESP8266-01 MCUs have no analog input pin at all. Various boards use the ESP8266-12 MCU and have an integrated 220k/100k voltage divider that would allow up to 3.2V on the boards analog input. For these boards, only a single resistor needs to provided, which adds to the internal voltage divider. With a 1M resistor we can measure up to 13.2V  without risk of damaging the ESP analog input port. It would be possible to add more analog inputs by using a 74HC405x port multiplexer.

### Water Pump Power Supply and Water Pump Voltage Measurement

We have a brushless DC water pump that can be driven with 3.5V to 9V power supply. We use another instance of the same circuit as above for the water pump power supply. Rather recalculating the (R3+R4)/R5 voltage divider to limit the maximum output voltage to 9V, we use the same resistor values as for 12.5V output and will limit max output voltage in software.

### Water Tank Level Sensor

We use an XKC-25-V sensor to measure the water tank liquid level. The sensor accepts 5-24 Vcc and outputs a voltage between 0 and Vccmax depending on the liquid level. In the software, no drivers are required, it only needs to measure sensor output voltage on an analog MCU pin. In electronics, only a resistor is needed to limit the output voltage to the maximum allowed voltage on the analog input pin (3.3V on ESP, 5V on Arduino).

The resistor value depends on the sensor's Vccmax and the MCU type. We have both 5V MCU rail voltage and 15V power supply voltage. We chose to use 15V because 5V would be on the lower end of the sensor's specification. For max 15V sensor output voltage and an ESP board, we chose a resistor value of 1.22M (1M in series with 220k) in front of the MCU analog input pin.

### Environment Sensors

The environment sensors are not shown in the schematics because they do not need electronic components (only Vcc, GND and connection to a MCU digital pin or I2C).

We considered DHT11, DHT22, AM2320, SI7021 and BME280 environmental sensors. We choose to use DHT22 sensors for the following reasons:
- we want the sensor to have a cover shield. SI7021 and BME280 have no shield.
- the sensor should be mounted easily. Only DHT22 and AM2320B sensors have a plastic cover with a mounting hole

DHT22 sensors are connected by 3 lines (signal, 5V Vcc and GND). They need a dedicated MCU pin for each sensor.
AM2320 sensors are connected by 4 lines (SDA, SCL, 5V Vcc and GND). They use the I2C bus and do not use a dedicated MCU pin for each sensor. They are slightly more precise than DHT22, are better mountable, but are also more expensive than DHT22. Only one AM2320 sensor can be attached to an I2C bus as they use a fixed I2C address.

## Software

### Supported Boards

The Arduino sketch is designed to support Arduino boards with ATmega328 CPU (Uno, Nano, Mini Pro) and ESP8266 boards (ESP8266-12, Wemos D1 mini, NodeMCU or similar). We are working to support ESP32 boards. Further boards might be added.

### Components and Configuration

The software is designed to be modular. Configuration settings are defined by C preprocessor defines in the config section. Various components can be deconfigured by commenting out `#define WITH_*` lines. Config settings not starting with `WITH_` can be left at the defaults, they get ignored if their corresponding `WITH_` settings are commented out.

The code is heavily #ifdef'd in order to keep to resulting code small by not compiling deconfigured stuff.

#### PWM Usage

Controlling DC devices by PWM is a core concept of Arduino-like MCUs. By default, rather low PWM frequencies are used (Arduino: 490Hz, ESP8266: 1kHz). Higher PWM frequencies provide finer resolution at the cost of more MCU workload (interrupts). Higher PWM frequencies also reduce ripple voltage after the RC lowpass. Both of these improvements are not really needed just to drive fans by PWM.

However, using PWM with the default low PWM frequencies may affect other devices and libraries used by the MCU. We noticed a DHT22 sensor was occasionally reporting invalid values in combination with a low PWM frequency - possibly because the DHT22 driver uses timing related code internally. The same issue may be relevant when using bus technologies such as SPI or I2C.

We use an increased PWM frequency of >30kHz by default. This can be disabled for debugging by commenting out `#define WITH_HIGH_PWMFREQ`.

#### Configuration `#define` Options

##### WITH_REAR_FANS and WITH_REAR_FANS_VOLTAGE

`WITH_REAR_FANS` allows to control the rear fans from the MCU, which is the main project goal anyway. Might be disabled for testing purposes. `WITH_REAR_FANS_VOLTAGE` allows to measure fan voltage and report it by HTTP or serial console if configured.

##### WITH_WATER_PUMP and WITH_WATER_PUMP_VOLTAGE

`WITH_WATER_PUMP` allows to control water pump power from the MCU. Might be disabled for testing purposes. `WITH_WATER_PUMP_VOLTAGE` allows to measure water pump voltage and report it by HTTP or serial console if configured. Note that ESP8266 provide only one analog input, so either fans or pump voltage can be monitored.

##### WITH_FRONT_ENV_SENSOR and WITH_REAR_ENV_SENSOR

These define that environmental sensors are mounted on the front and the back of the unit. An environmental sensor reads and reports at least temperature und humidity data. Currently supported sensors are DHT22, possibly supported sensors might be AM2320, SI7021 or BMEx80. If configured, the sensors will report data by HTTP or serial console if these channels are enabled.

##### WITH_BICOLOR_STATUS_LED

This is used to support a bicolor status LED (green/red) that is mounted on the Apple G5 frontpanel in place of the audio jack. 

##### WITH_SERIAL and WITH_SERIAL_COMMANDS

`WITH_SERIAL` is useful for debugging with the MCU attached to an IDE. It will print startup messages and periodical sensor value data to the serial console. In production use without a permanently connected serial console, it should be commented out, because it will claim runtime memory for string operations that are never consumed. Similarily, it should be disabled if WITH_OTA is used (because there is no serial console on OTA).
`WITH_SERIAL_COMMANDS` allows to actively inject commands from the serial console, rather than just passively reporting status. It requires `WITH_SERIAL` and is ignored otherwise. Supported commands:
- `status` - show current values
- `contrep on|off` - toggle continuous value reporting (default on)
- `fan n` - set fan speed to n percent (0 < n <= 100)
- `pump n` - set pump speed to n percent (0 < n <= 100)

##### WITH_FS

`WITH_FS` defines whether a flash file system should be supported. Not available for Arduino hardware (ESP8266/ESP32 only). Required for `WITH_HTTPSRV`.

##### WITH_ESP8266_WIFI

`WITH_ESP8266_WIFI` will be defined automatically if WiFi capable hardware is detected. Comment out to explicitly disable WiFi. Requires `WITH_FS`.

##### WITH_HTTPSRV

`WITH_HTTPSRV` controls whether the device provides an HTTP server. It is ignored if `WITH_ESP8266_WIFI` is not defined. Comment out to explicitly disable HTTP server.

##### WITH_OTA

`WITH_OTA` supports over-the-air updates of the code running on the MCU. It is ignored if `WITH_ESP8266_WIFI` is not defined. Comment out to explicitly disable OTA updates. When using `WITH_OTA`, you should disable `WITH_SERIAL`, as there is no usable serial line.

##### WITH_HIGH_PWMFREQ

This increases the PWM frequency from 490 Hz to 31372.55 Hz (Arduino), or from 1 kHz to 32 kHz (ESP8266). This is recommended when using PWM for rear fans or pump voltage control, because it reduces the possibility of interference with other sensors or libraries (we noticed a DHT22 reporting bad values when low PWM frequencies were used).

## References

* G5 (dis-)assembly and pinouts: https://www.applerepairmanuals.com/the_manuals_are_in_here/PowerMac_G5.pdf
* G5 pinouts: https://kettek.net/articles/powermac-g5-to-atx-pinouts
* G5 fan wiring: https://www.insanelymac.com/forum/topic/86729-wiring-for-g5-fans/
* Helpful for G5 fan pinouts (rest is misleading): https://www.rellimmot.com/how-to/Mac-Fan-Pinouts/
* Details on G5 fans: https://www.tonymacx86.com/threads/info-on-g5-fans.69889/
* More details on G5 fans (posts #12 and #16): https://www.tonymacx86.com/threads/macpro-fan-pinout.122584/page-2
* G5 front panel pinout (post #3): https://gradivis.com/projects/topic/powermac-g5-resource-page

* Latching circuits (esp. Fig 6): http://www.mosaic-industries.com/embedded-systems/microcontroller-projects/electronic-circuits/push-button-switch-turn-on/latching-toggle-power-switch
* Latching circuits: http://www.technoblogy.com/show?VOO

* PWM to DC voltage: https://www.edn.com/control-an-lm317t-with-a-pwm-signal/
* PWM to DC voltage: https://electronics.stackexchange.com/questions/17852/how-does-this-power-supply-circuit-work-mcu-lm317
* PWM to DC voltage (post #14): https://forum.arduino.cc/index.php?topic=459003.0
* Details on LM317 (german): https://praktische-elektronik.dr-k.de/Bauelemente/Be-LM317.html

* Arduino Nano pinouts; https://www.circuitstoday.com/arduino-nano-tutorial-pinout-schematics
* ESP8266 pinouts and pin usage: https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
* Wemos D1 mini pinouts and schematics: https://escapequotes.net/esp8266-wemos-d1-mini-pins-and-diagram/
* Pinouts and details about various ESP8266 boards (german): http://stefanfrings.de/esp8266/

* Nice voltage divider calculator: https://ohmslawcalculator.com/voltage-divider-calculator

* Voltage measurement: https://startingelectronics.org/articles/arduino/measuring-voltage-with-arduino/
* MCP4162: https://tronixstuff.com/category/mcp4162/
* MCP4162: https://github.com/phanrahan/arduino/tree/master/MCP4162

* HC4051 Analog Mux: https://hackaday.com/2017/05/17/a-few-of-our-favorite-chips-4051-analog-mux/
* HC4051 Analog Mux: https://internetofhomethings.com/homethings/?p=530
* HC4051 Analog Mux: http://sankios.imediabank.com/74hc4051
