#define SW_VERSION "1.2"

//---------------------------------------------------------------------------------------------------------------------
// constants (to get the symbols defined, value does not really matter)
//---------------------------------------------------------------------------------------------------------------------
// constants for type of LM317 voltage control
#define PWM_CONTROL 0
#define MCP4162_CONTROL 1
// constants for component hardware
#define DHT22_SENSOR_ 0
#define AM2320_SENSOR_ 1
#define HC4051_MUX_ 2
#define PCF8591_MUX_ 3
#define ADS1115_MUX_ 4

// constants for unknown data values
#define TEMPERATURE_UNKNOWN -99
#define HUMIDITY_UNKNONW -99

//---------------------------------------------------------------------------------------------------------------------
// include config from separate file
//---------------------------------------------------------------------------------------------------------------------
#include "config.h"

//---------------------------------------------------------------------------------------------------------------------
// board definitions (supported: Arduino, ESP8266, ESP32)
//---------------------------------------------------------------------------------------------------------------------
#ifdef HW_ARDUINO
  #define HW_NAME "Arduino"
  #define HW_CPUFREQ 16
  #define ANALOG_WRITE_RANGE 256
  // for memory reporting, from https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
  #ifdef __arm__
  // should use uinstd.h to define sbrk but Due causes a conflict
  extern "C" char* sbrk(int incr);
  #else  // __ARM__
  extern char *__brkval;
  #endif  // __arm__
#endif  // HW_ARDUINO

#ifdef HW_ESP8266
  #define HW_NAME "ESP8266"
  //#define ANALOG_WRITE_RANGE 1024
  #define ANALOG_WRITE_RANGE 256
  #include <ESP.h>
#endif

#ifdef HW_ESP32
  #define HW_NAME "ESP32"
#endif

//---------------------------------------------------------------------------------------------------------------------
// global section - global vars and included definitions/functions for components
//---------------------------------------------------------------------------------------------------------------------

long freeHeapMem               = 0;
long heapFragmentation         = 0;

#ifdef WITH_SERIAL
long    lastReportedOnSerial   = 0;
boolean continuous_reporting   = true;
#endif

#ifdef WITH_VOLTAGE_MEASURE
long  lastReadVoltages         = 0;
float railVoltage              = 0.0;
float mcuRailVoltage           = 0.0;
#ifdef WITH_REAR_FANS
float rearFansVoltage          = 0.0;
#endif
#ifdef WITH_WATER_PUMP
float pumpVoltage              = 0.0;
#endif
#endif

#ifdef WITH_REAR_FANS
long  rearFansSpeedPercent     = 0;
long  rearFansPwmValue         = 0;
#endif

#ifdef WITH_WATER_PUMP
long  pumpSpeedPercent         = 0;
long  pumpPwmValue             = 0;
#endif

#ifdef WITH_FRONT_ENV_SENSOR
float frontEnvSensorTemperature = 0;
float frontEnvSensorHumidity    = 0;
long  lastReadFrontEnvSensor    = 0;
#if FRONT_ENV_SENSOR_TYPE == DHT22_SENSOR_
  #include <dhtnew.h>
  DHTNEW frontDHT22Sensor(DIGITAL_IN1_PIN);
#endif
#if FRONT_ENV_SENSOR_TYPE == AM2320_SENSOR_
  #define FRONT_ENV_SENSOR_I2C
  #include <AM2320.h>
  AM2320 frontAM2320Sensor;
#endif
#endif

#ifdef WITH_REAR_ENV_SENSOR
float rearEnvSensorTemperature = 0.0;
float rearEnvSensorHumidity    = 0.0;
long  lastReadRearEnvSensor    = 0;
#if REAR_ENV_SENSOR_TYPE == DHT22_SENSOR_
  #include <dhtnew.h>
  DHTNEW rearDHT22Sensor(DIGITAL_IN2_PIN);
#endif
#if REAR_ENV_SENSOR_TYPE == AM2320_SENSOR_
  #define REAR_ENV_SENSOR_I2C
  #include <AM2320.h>
  AM2320 rearAM2320Sensor;
#endif
#endif

#ifdef WITH_ESP8266_WIFI
#include <ESP8266WiFi.h>
IPAddress ip;
#ifdef WITH_OTA
#include <ArduinoOTA.h>
boolean ota_enable = true;
#endif
#ifdef WITH_ESP8266_HTTPSRV
#include <ESP8266WebServer.h>
#include <uri/UriBraces.h>
//#include <uri/UriRegex.h>
ESP8266WebServer webserver(HTTPSRV_PORT);
#endif
#endif  // WITH_ESP8266_WIFI

#if (defined(WITH_REAR_FANS) && REAR_FANS_CONTROL == PWM_CONTROL) || (defined(WITH_WATER_PUMP) && WATER_PUMP_CONTROL == PWM_CONTROL)
#define USE_PWM
#endif

#if (defined(WITH_REAR_FANS) && REAR_FANS_CONTROL == MCP4162_CONTROL) || (defined(WITH_WATER_PUMP) && WATER_PUMP_CONTROL == MCP4162_CONTROL)
#include "SPI.h"
#define SPI_WRITE_RANGE 256
#define USE_SPI
#endif

#if defined(FRONT_ENV_SENSOR_I2C) || defined(REAR_ENV_SENSOR_I2C) || defined(ANALOG_MUX_I2C)
#include <Wire.h>
#define USE_I2C
#endif

#if defined(ANALOG_MUX_TYPE) && ANALOG_MUX_TYPE == HC4051_MUX_
const byte muxOut = MUX4051_OUT;         // usually A0
// the multiplexer address select lines (A/B/C)
const byte muxAddressA = MUX4051_ADDR_A; // low-order bit
const byte muxAddressB = MUX4051_ADDR_B;
const byte muxAddressC = MUX4051_ADDR_C; // high-order bit
#endif

#if defined(ANALOG_MUX_TYPE) && ANALOG_MUX_TYPE == ADS1115_MUX_
#include <ADS1115_WE.h>
#define I2C_ADDRESS 0x48
ADS1115_WE adc(I2C_ADDRESS);
#endif

#ifdef WITH_BICOLOR_STATUS_LED
#define STATUS_GREEN_LED_PIN DIGITAL_OUT1_PIN
#define STATUS_GREEN_LED_PIN DIGITAL_OUT2_PIN
#endif

#ifdef WITH_SERIAL_COMMANDS
#include "SerialCommands.h"
char serial_command_buffer_[16];
//// handler callback functions must be defined here, not in the functions section hear the end
void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print(F("Unrecognized command ["));
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println(F("]"));
}
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\n", " ");

void cmd_get_status(SerialCommands* sender)
{
  serial_report_values();
}
SerialCommand cmd_get_status_("status", cmd_get_status);

void cmd_set_contrep(SerialCommands* sender)
{
  char* contrep = sender->Next();
  if (contrep == NULL)
  {
    // report setting
    sender->GetSerial()->print(F("Continuous reporting: "));
    sender->GetSerial()->println(continuous_reporting);
  }
  else if (strcmp(contrep, "on") == 0)
  {
    // enable continuous reporting
    continuous_reporting = true;
  }
  else if (strcmp(contrep, "off") == 0)
  {
    // disable continuous reporting
    continuous_reporting = false;
  }
  else
  {
    sender->GetSerial()->println(F("ERROR invalid contrep "));
  }
}
SerialCommand cmd_set_contrep_("contrep", cmd_set_contrep);

#ifdef WITH_OTA
void cmd_set_ota(SerialCommands* sender)
{
  char* ota = sender->Next();
  if (ota == NULL)
  {
    // report setting
    sender->GetSerial()->print(F("OTA: "));
    sender->GetSerial()->println(ota_enable);
  }
  else if (strcmp(ota, "on") == 0)
  {
    // enable OTA
    ota_enable = true;
  }
  else if (strcmp(ota, "off") == 0)
  {
    // disable OTA
    ota_enable = false;
  }
  else
  {
    sender->GetSerial()->println(F("ERROR invalid ota "));
  }
}
SerialCommand cmd_set_ota_("ota", cmd_set_ota);
#endif

#ifdef WITH_REAR_FANS
void cmd_set_fan(SerialCommands* sender)
{
  char* fanspeed = sender->Next();
  if (fanspeed == NULL)
  {
    sender->GetSerial()->println(F("ERROR no fan speed "));
    return;
  }
  setRearFansSpeedPercent(atoi(fanspeed));
}
SerialCommand cmd_set_fan_("fan", cmd_set_fan);
#endif

#ifdef WITH_WATER_PUMP
void cmd_set_pump(SerialCommands* sender)
{
  char* pumpspeed = sender->Next();
  if (pumpspeed == NULL)
  {
    sender->GetSerial()->println(F("ERROR no pump speed "));
    return;
  }
  setPumpSpeedPercent(atoi(pumpspeed));
}
SerialCommand cmd_set_pump_("pump", cmd_set_pump);
#endif
#endif  // WITH_SERIAL_COMMANDS

//-------------------------------------------------------------------------------------------
// setup()
//-------------------------------------------------------------------------------------------
void setup ()
{
#ifdef WITH_SERIAL
    init_serial();
#endif

#if defined(USE_PWM) && defined(WITH_HIGH_PWMFREQ)
    init_pwm();
#endif

#ifdef USE_I2C
    init_i2c();
#endif

#ifdef USE_SPI
    init_spi();
#endif

#if defined(ANALOG_MUX_TYPE) && ANALOG_MUX_TYPE == HC4051_MUX_
    init_analog_mux();
#endif

#ifdef WITH_REAR_FANS
    init_rear_fans();
#endif

#ifdef WITH_WATER_PUMP
    init_water_pump();
#endif

#ifdef WITH_FRONT_ENV_SENSOR
    init_front_env_sensor();
#endif

#ifdef WITH_REAR_ENV_SENSOR
    init_rear_env_sensor();
#endif

#ifdef WITH_BICOLOR_STATUS_LED
    pinMode(STATUS_GREEN_LED_PIN, OUTPUT);
    pinMode(STATUS_RED_LED_PIN, OUTPUT);
#endif

#ifdef WITH_ESP8266_WIFI
    init_esp8266_wifi();
#endif

#if defined(WITH_SERIAL) && defined(WITH_SERIAL_COMMANDS)
    serial_commands_.SetDefaultHandler(cmd_unrecognized);
    serial_commands_.AddCommand(&cmd_get_status_);
    serial_commands_.AddCommand(&cmd_set_contrep_);
#ifdef WITH_OTA
    serial_commands_.AddCommand(&cmd_set_ota_);
#endif
#ifdef WITH_REAR_FANS
    serial_commands_.AddCommand(&cmd_set_fan_);
#endif
#ifdef WITH_WATER_PUMP
    serial_commands_.AddCommand(&cmd_set_pump_);
#endif
#endif
} // end setup()


//-------------------------------------------------------------------------------------------
// loop()
//-------------------------------------------------------------------------------------------
void loop()
{
    freeHeapMem = getFreeHeapMem();
#ifndef HW_ARDUINO
    heapFragmentation = getMemoryFragmentation();
#endif

#ifdef WITH_OTA
    if (ota_enable) {
      // check for over-the-air updates
      ArduinoOTA.handle();
    }
#endif

#if defined(ANALOG_MUX_TYPE) && ANALOG_MUX_TYPE == HC4051_MUX_
    // read analog mux
    if ((millis() - lastReadVoltages) > VOLTAGE_READ_INTERVAL) {
        readAnalogMux();
    }
#endif

#ifdef WITH_FRONT_ENV_SENSOR
    // read front environment sensor
    if ((millis() - lastReadFrontEnvSensor) > FRONT_ENV_SENSOR_READ_INTERVAL) {
  #if FRONT_ENV_SENSOR_TYPE == DHT22_SENSOR_
        read_front_dht22();
  #endif
  #if FRONT_ENV_SENSOR_TYPE == AM2320_SENSOR_
        read_front_am2320();
  #endif
    }
#endif  // WITH_FRONT_ENV_SENSOR

#ifdef WITH_REAR_ENV_SENSOR
    // read rear environment sensor
    if ((millis() - lastReadRearEnvSensor) > REAR_ENV_SENSOR_READ_INTERVAL) {
  #if REAR_ENV_SENSOR_TYPE == DHT22_SENSOR_
      read_rear_dht22();
  #endif
  #if REAR_ENV_SENSOR_TYPE == AM2320_SENSOR_
      read_rear_am2320();
  #endif
    }
#endif  // WITH_REAR_ENV_SENSOR

#if defined(WITH_REAR_FANS) && defined(WITH_REAR_FANS_VOLTAGE)
    // read rear fans voltage
    if ((millis() - lastReadRearFansVoltage) > REAR_FANS_VOLTAGE_READ_INTERVAL) {
        read_rear_fans_voltage();
    }
#endif

#if defined(WITH_WATER_PUMP) && defined(WITH_WATER_PUMP_VOLTAGE)
    // read water pump voltage
    if ((millis() - lastReadPumpVoltage) > WATER_PUMP_VOLTAGE_READ_INTERVAL) {
        read_water_pump_voltage();
    }
#endif

#ifdef WITH_ESP8266_HTTPSRV
    // handle HTTP client
    webserver.handleClient();
#endif

#ifdef WITH_SERIAL
    if (continuous_reporting && (millis() - lastReportedOnSerial) > SERIAL_REPORT_INTERVAL) {
        serial_report_values();
    }

#ifdef WITH_SERIAL_COMMANDS
    serial_commands_.ReadSerial();
#endif

#endif  // WITH_SERIAL

    delay(LOOP_DELAY);
} // end loop()


//-------------------------------------------------------------------------------------------
// functions
//-------------------------------------------------------------------------------------------

//-------------- I2C
#ifdef USE_I2C
void init_i2c() {
    consolePrintln("Initialize I2C...");
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
}
#endif  // USE_I2C


//-------------- SPI
#ifdef USE_SPI
void init_spi() {
    consolePrintln("Initialize SPI...");
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);  // needed for MCP4162
}
void writeSPIvalue(int line, int value) {
    consolePrintln("Write %u to SPI line %u", value, line);
    digitalWrite(line, LOW);
    SPI.transfer( (value & 0x100) ? 1 : 0 );
    SPI.transfer( value & 0xff ); // send value (0~255)
    digitalWrite(line, HIGH);
}
#endif  // USE_SPI


//-------------- PWM (high frequeny setup)
#ifdef USE_PWM
#ifdef WITH_HIGH_PWMFREQ
void init_pwm() {
  #ifdef HW_ARDUINO
    set_pwm_freq_arduino();
  #endif
  #ifdef HW_ESP8266
    analogWriteRange(ANALOG_WRITE_RANGE - 1);
    analogWriteFreq(PWMFREQ_ESP);
  #endif
  #ifdef HW_ESP32
  #endif
}
#ifdef HW_ARDUINO
/*
 * boilerplate code to set PWM frequency on Arduino
 */
void set_pwm_freq_arduino()
{
  consolePrintln("Initialize PWM frequency...");
// For Arduino Uno, Nano, YourDuino RoboRED, Mini Driver, Lilly Pad and any other board using ATmega 8, 168 or 328
//---------------------------------------------- Set PWM frequency for D5 & D6 -------------------------------
//NOTE: Changing this timer 0 affects millis() and delay!
//TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
//TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
  TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (The DEFAULT)
//TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
//TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz
//---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
//TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
//---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------
  TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
//TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
//TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
//TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
}
#endif  // HW_ARDUINO
#endif  // WITH_HIGH_PWMFREQ
#endif  // USE_PWM

//-------------- ANALOG_MUX
#if defined(ANALOG_MUX_TYPE) && ANALOG_MUX_TYPE == HC4051_MUX_
void init_analog_mux()
{
    pinMode (muxOut, OUTPUT);
    pinMode (muxAddressA, OUTPUT);
    pinMode (muxAddressB, OUTPUT);
    pinMode (muxAddressC, OUTPUT);
}
void readAnalogMux()
{
#ifdef WITH_VOLTAGE_MEASURE
    railVoltage     = readVoltage(MUX_PIN_RAIL_VOLTAGE);
    mcuRailVoltage  = readVoltage(MUX_PIN_MCURAIL_VOLTAGE);
#ifdef WITH_REAR_FANS
    rearFansVoltage = readVoltage(MUX_PIN_FANS_VOLTAGE);
#endif
#ifdef WITH_WATER_PUMP
    pumpVoltage     = readVoltage(MUX_PIN_PUMP_VOLTAGE);
#endif
#endif  // WITH_VOLTAGE_MEASURE
    // other analog sensors below
    lastReadVoltages = millis();
}
#ifdef WITH_VOLTAGE_MEASURE
float readVoltage (const byte which)
{
    // select correct MUX channel
    digitalWrite (muxAddressA, (which & 1) ? HIGH : LOW);  // low-order bit
    digitalWrite (muxAddressB, (which & 2) ? HIGH : LOW);
    digitalWrite (muxAddressC, (which & 4) ? HIGH : LOW);  // high-order bit
    // read selected sensor: take a number of analog samples and add them up
    int sum       = 0;      // sum of samples taken
    int count     = 0;      // current sample number
    while (count < VOLTAGE_NUM_SAMPLES) {
        sum += analogRead(muxOut);
        count++;
        delay(10);
    }
    float avrg = sum / count;
    // MISSING: convert to voltage value
    consolePrintln("Raw analog sensor %s, val %.2f", which, avrg);
    return avrg;
}
#endif  // WITH_VOLTAGE_MEASURE
#endif  // ANALOG_MUX_TYPE == HC4051_MUX_


//-------------- REAR_FANS
/* functions for rear fans */
#ifdef WITH_REAR_FANS
void init_rear_fans() {
  int val = 0;
  #if REAR_FANS_CONTROL == PWM_CONTROL
  int pin = PWM_CONTROL1_PIN;
  #endif
  #if REAR_FANS_CONTROL == MCP4162_CONTROL
  int pin = SPI_SELECT1_PIN;
  #endif
    consolePrintln("Initialize rear fans to 0");
    pinMode(pin, OUTPUT);
    setRearFansSpeedRaw(pin, val);
}
void setRearFansSpeedRaw(int pin, long value) {
  #if REAR_FANS_CONTROL == PWM_CONTROL
    analogWrite(pin, value);
  #endif
  #if REAR_FANS_CONTROL == MCP4162_CONTROL
    writeSPIvalue(pin, value);
  #endif
}
void setRearFansSpeedPercent(long speed) {
    if (speed < 0) {
      rearFansSpeedPercent = 0;
    } else if (speed >= 100) {
      rearFansSpeedPercent = 100;
    } else {
      rearFansSpeedPercent = speed;
    }
    consolePrintln("Set rear fans speed%% to %u", rearFansSpeedPercent);
  #if REAR_FANS_CONTROL == PWM_CONTROL
    rearFansPwmValue = (rearFansSpeedPercent / 100.0) * (ANALOG_WRITE_RANGE - 1);
    consolePrintln("Set rear fans PWM to %u", rearFansPwmValue);
    analogWrite(PWM_CONTROL1_PIN, rearFansPwmValue);
  #endif
  #if REAR_FANS_CONTROL == MCP4162_CONTROL
    rearFansSPIValue = (rearFansSpeedPercent / 100.0) * (SPI_WRITE_RANGE - 1);
    writeSPIvalue(SPI_SELECT1_PIN, rearFansSPIValue);
  #endif
}
#endif  // WITH_REAR_FANS


//-------------- WATER_PUMP
#ifdef WITH_WATER_PUMP
/* functions for water pump */
void init_water_pump() {
  int val = 0;
  #if WATER_PUMP_CONTROL == PWM_CONTROL
  int pin = PWM_CONTROL2_PIN;
  #endif
  #if WATER_PUMP_CONTROL == MCP4162_CONTROL
  int pin = SPI_SELECT2_PIN;
  #endif
    consolePrintln("Initialize water pump to 0");
    pinMode(pin, OUTPUT);
    setWaterPumpSpeedRaw(pin, val);
}
void setWaterPumpSpeedRaw(int pin, long value) {
  #if WATER_PUMP_CONTROL == PWM_CONTROL
    analogWrite(pin, value);
  #endif
  #if WATER_PUMP_CONTROL == MCP4162_CONTROL
    writeSPIvalue(pin, value);
  #endif
}
void setPumpSpeedPercent(long speed) {
    if (speed < 0) {
      pumpSpeedPercent = 0;
    } else if (speed >= 100) {
      pumpSpeedPercent = 100;
    } else {
      pumpSpeedPercent = speed;
    }
    consolePrintln("Set pump speed%% to %u", pumpSpeedPercent);
  #if WATER_PUMP_CONTROL == PWM_CONTROL
    pumpPwmValue = (pumpSpeedPercent / 100.0) * (ANALOG_WRITE_RANGE - 1);
    consolePrintln("Set pump PWM to ", pumpPwmValue);
    analogWrite(PWM_CONTROL2_PIN, pumpPwmValue);
  #endif
  #if WATER_PUMP_CONTROL == MCP4162_CONTROL
    // unimplemented
  #endif
}
#endif  // WITH_WATER_PUMP


//-------------- FRONT_ENV_SENSOR
#ifdef WITH_FRONT_ENV_SENSOR
#if FRONT_ENV_SENSOR_TYPE == DHT22_SENSOR_
/* functions for front DHT22 env sensor */
void init_front_env_sensor() {
    consolePrintln("Initialize front DHT22 sensor...");
    // maybe adjust DHT22 sensor offsets (example)
    //frontDHT22Sensor.setHumOffset(10);
    //frontDHT22Sensor.setTempOffset(-3.5);
}
void read_front_dht22() {
    frontDHT22Sensor.read();
    frontEnvSensorHumidity    = frontDHT22Sensor.getHumidity();
    frontEnvSensorTemperature = frontDHT22Sensor.getTemperature();
    lastReadFrontEnvSensor    = frontDHT22Sensor.lastRead();
}
#endif  // DHT22_SENSOR_
#if FRONT_ENV_SENSOR_TYPE == AM2320_SENSOR_
void init_front_env_sensor() {
  consolePrintlnA("Initialize front AM2320 sensor...");
  // nothing to do
}
void read_front_am2320() {
  switch(frontAM2320Sensor.Read()) {
    case 2:
      consolePrintln("Front AM2320 CRC failed");
      frontEnvSensorHumidity    = HUMIDITY_UNKNONW;
      frontEnvSensorTemperature = TEMPERATURE_UNKNOWN;
      break;
    case 1:
      consolePrintln("Front AM2320 sensor offline");
      frontEnvSensorHumidity    = HUMIDITY_UNKNONW;
      frontEnvSensorTemperature = TEMPERATURE_UNKNOWN;
      break;
    case 0:
      frontEnvSensorHumidity    = frontAM2320Sensor.h;
      frontEnvSensorTemperature = frontAM2320Sensor.t;
      break;
    default:
      frontEnvSensorHumidity    = HUMIDITY_UNKNONW;
      frontEnvSensorTemperature = TEMPERATURE_UNKNOWN;
  }
    lastReadFrontEnvSensor    = millis();
}
#endif  // AM2320_SENSOR_
#endif  // WITH_FRONT_ENV_SENSOR


//-------------- REAR_ENV_SENSOR
#ifdef WITH_REAR_ENV_SENSOR
#if REAR_ENV_SENSOR_TYPE == DHT22_SENSOR_
/* functions for rear DHT22 env sensor */
void init_rear_env_sensor() {
    consolePrintln("Initialize rear DHT22 sensor...");
    // maybe adjust DHT22 sensor offsets (example)
    //rearDHT22Sensor.setHumOffset(10);
    //rearDHT22Sensor.setTempOffset(-3.5);
}
void read_rear_dht22() {
    rearDHT22Sensor.read();
    rearEnvSensorHumidity    = rearDHT22Sensor.getHumidity();
    rearEnvSensorTemperature = rearDHT22Sensor.getTemperature();
    lastReadRearEnvSensor    = rearDHT22Sensor.lastRead();
}
#endif  // DHT22_SENSOR_
#if REAR_ENV_SENSOR_TYPE == AM2320_SENSOR_
void init_rear_env_sensor() {
    consolePrintln("Initialize rear AM2320 sensor...");
  // nothing to do
}
void read_rear_am2320() {
  switch(rearAM2320Sensor.Read()) {
    case 2:
      consolePrintln("Rear AM2320 CRC failed");
      rearEnvSensorHumidity    = HUMIDITY_UNKNONW;
      rearEnvSensorTemperature = TEMPERATURE_UNKNOWN;
      break;
    case 1:
      consolePrintln("Rear AM2320 sensor offline");
      rearEnvSensorHumidity    = HUMIDITY_UNKNONW;
      rearEnvSensorTemperature = TEMPERATURE_UNKNOWN;
      break;
    case 0:
      rearEnvSensorHumidity    = rearAM2320Sensor.h;
      rearEnvSensorTemperature = rearAM2320Sensor.t;
      break;
    default:
      rearEnvSensorHumidity    = HUMIDITY_UNKNONW;
      rearEnvSensorTemperature = TEMPERATURE_UNKNOWN;
  }
    lastReadRearEnvSensor    = millis();
}
#endif  // AM2320_SENSOR_
#endif  // WITH_REAR_ENV_SENSOR


//-------------- WIFI
#ifdef WITH_ESP8266_WIFI
/* functions for WiFi support */
void connect_wifi_esp8266()
{
    // Connect to WiFi network
    consolePrintln("Connecting to %s", WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    ip = WiFi.localIP();
    consolePrintln("\r\nConnected to %s, IP address: %s", WIFI_SSID, ip.toString().c_str());
}

#ifdef WITH_ESP8266_HTTPSRV
/* functions for embedded web server */
// Handle Root URI
void rootPage() {
    webserver.send(200, F("text/plain"),
#ifdef WITH_REAR_FANS
                   "Fan speed: " + String(rearFansSpeedPercent) + "%\r\n" +
#ifdef WITH_REAR_FANS_VOLTAGE
                   "Fan voltage: " + String(rearFansVoltage) + "V\r\n" +
#endif
#endif
#ifdef WITH_WATER_PUMP
                   "Pump speed: " + String(pumpSpeedPercent) + "%\r\n" +
#ifdef WITH_WATER_PUMP_VOLTAGE
                   "Pump voltage: " + String(pumpVoltage) + "V\r\n" +
#endif
#endif
#ifdef WITH_FRONT_ENV_SENSOR
                   "Front temperature: " + String(frontEnvSensorTemperature) + "C\r\n" +
                   "Front humidity: " + String(frontEnvSensorHumidity) + "%\r\n" +
#endif
#ifdef WITH_REAR_ENV_SENSOR
                   "Rear temperature: " + String(rearEnvSensorTemperature) + "C\r\n" +
                   "Rear humidity: " + String(rearEnvSensorHumidity) + "%\r\n" +
#endif
                   "Free heap memory: " + String(freeHeapMem) + "Byte\r\n" +
#ifndef HW_ARDUINO
                   "Heap fragmentation: " + String(heapFragmentation) + "%\r\n" +
#endif
                   "");
}
 
// Handle 404
void notfoundPage(){
    webserver.send(404, F("text/plain"), F("404: Not found"));
}

void start_web_server() {
    // define URI handlers
    webserver.on("/", rootPage);
    webserver.onNotFound(notfoundPage);
  #ifdef WITH_REAR_FANS
    webserver.on(UriBraces("/fanspeed={}"), []() {
      String rearFansSpeedPercent = webserver.pathArg(0);
      long fspeed = rearFansSpeedPercent.toInt();
      setRearFansSpeedPercent(fspeed);
      rootPage();
    });
  #endif
  #ifdef WITH_WATER_PUMP
    webserver.on(UriBraces("/pumpspeed={}"), []() {
      String pumpSpeedPercent = webserver.pathArg(0);
      long pspeed = pumpSpeedPercent.toInt();
      setPumpSpeedPercent(pspeed);
      rootPage();
    });
  #endif
    // Start Web Server
    webserver.begin();
}

#endif  // WITH_ESP8266_HTTPSRV

void init_esp8266_wifi() {
    connect_wifi_esp8266();

#ifdef WITH_OTA
    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASS);
    //ArduinoOTA.setPasswordHash(OTA_PASS_HASH);
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = F("sketch");
      } else { // U_FS
        type = F("filesystem");
      }
      // NOTE: if updating FS this would be the place to unmount FS using FS.end()
      consolePrintln("Start updating %s", type);
    });
    ArduinoOTA.onEnd([]() {
      consolePrintln("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      consolePrintln("Progress: %u\r", progress / (total / 100));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      if (error == OTA_AUTH_ERROR) {
        consolePrintln("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        consolePrintln("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        consolePrintln("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        consolePrintln("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        consolePrintln("End Failed");
      }
    });
    consolePrintln("Start OTA on port %u", OTA_PORT);
    ArduinoOTA.begin();
#endif  // WITH_OTA

#ifdef WITH_ESP8266_HTTPSRV
    consolePrintln("Start web server on port %u", HTTPSRV_PORT);
    start_web_server();
#endif
}
#endif  // WITH_ESP8266_WIFI


//-------------- SERIAL
#ifdef WITH_SERIAL
/* functions for serial console */

void init_serial() {
    Serial.begin(SERIAL_BAUD);
    delay(SERIAL_DELAY);  // grace period for reinit after reboot
    int cpuspeed = getCpuSpeed();
    consolePrintln("\n==== v%s starting, board: %s, CPU speed: %u MHz", SW_VERSION, HW_NAME, cpuspeed);
}

void serial_report_values() {
  #ifdef WITH_REAR_FANS
    consolePrintln("\nFan speed%%: %u, PWM %u", rearFansSpeedPercent, rearFansPwmValue);
  #ifdef WITH_REAR_FANS_VOLTAGE
    consolePrintln("AnalogV: %u", rearFansAnalogIn);
  #endif
  #endif
  #ifdef WITH_WATER_PUMP
    consolePrintln("Pump speed%%: %u, PWM %u", pumpSpeedPercent, pumpPwmValue);
  #ifdef WITH_WATER_PUMP_VOLTAGE
    consolePrintln("AnalogV: %.2f", pumpAnalogIn);
  #endif
  #endif
  #ifdef WITH_FRONT_ENV_SENSOR
    consolePrintln("Front temperature: %.2f");
    consolePrintln("Front humidity%: %.2f");
  #endif
  #ifdef WITH_REAR_ENV_SENSOR
    consolePrintln("Rear temperature: %.2f", rearEnvSensorTemperature);
    consolePrintln("Rear humidity%: %.2f", rearEnvSensorHumidity);
  #endif
  #ifdef WITH_VOLTAGE_MEASURE
    consolePrintln("RailVolts: %.2f", railVoltage);
    consolePrintln("MCURailVolts: %.2f", mcuRailVoltage);
  #ifdef WITH_REAR_FANS
    consolePrintln("FansVolts: %.2f", rearFansVoltage);
  #endif
  #ifdef WITH_WATER_PUMP
    consolePrintln("PumpVolts: %.2f", pumpVoltage);
  #endif
  #endif
    consolePrintln("Free heap memory: %u", freeHeapMem);
  #ifndef HW_ARDUINO
    consolePrintln("Heap memory fragmentation%%: %u", heapFragmentation);
  #endif

    lastReportedOnSerial = millis();
}

#ifdef WITH_SERIAL_COMMANDS
#endif  // WITH_SERIAL_COMMANDS
#endif  // WITH_SERIAL

//-------------- SYSTEM (memory, CPU speed)
#ifdef HW_ARDUINO
//// report free heap memory on Arduino, from https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
#endif  // HW_ARDUINO

int getFreeHeapMem() {
#ifdef HW_ARDUINO
    return freeMemory();
#endif
#if defined(HW_ESP8266) || defined(ESP32)
    return ESP.getFreeHeap();
#endif
}

#ifndef HW_ARDUINO
int getMemoryFragmentation() {
    return ESP.getHeapFragmentation();
}
#endif

int getCpuSpeed() {
#ifdef HW_ARDUINO
    return HW_CPUFREQ;
#endif
#if defined(HW_ESP8266) || defined(ESP32)
    return ESP.getCpuFreqMHz();
#endif
}

void consolePrintln(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);
#ifdef WITH_SERIAL
    Serial.println(buffer);
#endif
}
