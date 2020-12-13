//---------------------------------------------------------------------------------------------------------------------
// determine board type (supported: Arduino, ESP8266)
//---------------------------------------------------------------------------------------------------------------------
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  #define HW_ARDUINO
  #define HW_NAME "Arduino"
  #define ANALOG_WRITE_RANGE 256
#elif defined(ESP8266)
  #define HW_ESP8266
  #define HW_NAME "ESP8266"
  #define ANALOG_WRITE_RANGE 1024
#elif defined(ESP32)
  #define HW_ESP32
  #define HW_NAME "ESP32"
#else
  #error "Unsupported board!"
#endif

// constants for type of LM317 voltage control
#define PWM_CONTROL 0
#define MCP4162_CONTROL 1
// constants for sensor types
#define DHT22_SENSOR 0

//---------------------------------------------------------------------------------------------------------------------
// include config from separate file
//---------------------------------------------------------------------------------------------------------------------
#include "config.h"

//---------------------------------------------------------------------------------------------------------------------
// global section - global vars and included definitions/functions for components
//---------------------------------------------------------------------------------------------------------------------

#ifdef WITH_SERIAL
long  lastReportedOnSerial     = 0;
#endif

#ifdef WITH_REAR_FANS
long  rearFansSpeedPercent     = 0;
long  rearFansPwmValue         = 0;
int   rearFansSPIValue         = 0;
float rearFansVoltage          = 0;
float rearFansAnalogIn         = 0;
long  lastReadRearFansVoltage  = 0;
#endif

#ifdef WITH_WATER_PUMP
long  pumpSpeedPercent         = 0;
long  pumpPwmValue             = 0;
int   pumpSPIValue             = 0;
float pumpVoltage              = 0;
float pumpAnalogIn             = 0;
long  lastReadPumpVoltage      = 0;
#endif

#ifdef WITH_FRONT_ENV_SENSOR
float frontEnvSensorTemperature = 0;
float frontEnvSensorHumidity    = 0;
#if FRONT_ENV_SENSOR_TYPE == DHT22_SENSOR
  #include <dhtnew.h>
  DHTNEW frontDHT22Sensor(FRONT_ENV_SENSOR_PIN);
#endif
#endif

#ifdef WITH_REAR_ENV_SENSOR
float rearEnvSensorTemperature = 0;
float rearEnvSensorHumidity    = 0;
#if REAR_ENV_SENSOR_TYPE == DHT22_SENSOR
  #include <dhtnew.h>
  DHTNEW rearDHT22Sensor(REAR_ENV_SENSOR_PIN);
#endif
#endif

#ifdef WITH_ESP8266_WIFI
#include <ESP8266WiFi.h>
IPAddress ip;

#ifdef WITH_OTA
#include <ArduinoOTA.h>
#endif

#ifdef WITH_ESP8266_HTTPSRV
#include <ESP8266WebServer.h>
#include <uri/UriBraces.h>
//#include <uri/UriRegex.h>
ESP8266WebServer webserver(HTTPSRV_PORT);
#endif
#endif  // WITH_ESP8266_WIFI

#ifdef WITH_HIGH_PWMFREQ
#if (defined(WITH_REAR_FANS) && REAR_FANS_CONTROL == PWM_CONTROL) || (defined(WITH_WATER_PUMP) && WATER_PUMP_CONTROL == PWM_CONTROL)
#define USE_HIGH_PWMFREQ
#endif
#endif

#if (defined(WITH_REAR_FANS) && REAR_FANS_CONTROL == MCP4162_CONTROL) || (defined(WITH_WATER_PUMP) && WATER_PUMP_CONTROL == MCP4162_CONTROL)
#include "SPI.h"
#define SPI_WRITE_RANGE 256
#define USE_SPI
#endif

#if defined(WITH_SERIAL) && defined(WITH_DEBUG)
#define USE_SERIAL_DEBUG
#endif

//-------------------------------------------------------------------------------------------
// setup()
//-------------------------------------------------------------------------------------------
void setup ()
{
#ifdef WITH_SERIAL
    init_serial();
#endif

#ifdef USE_HIGH_PWMFREQ
    init_pwmfreq();
#endif

#ifdef USE_SPI
    init_spi();
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

#ifdef WITH_PUSHBUTTON_LED
    pinMode(PUSHBUTTON_LED_PIN, OUTPUT);
#endif

#ifdef WITH_BICOLOR_STATUS_LED
    pinMode(STATUS_GREEN_LED_PIN, OUTPUT);
    pinMode(STATUS_RED_LED_PIN, OUTPUT);
#endif

#ifdef WITH_ESP8266_WIFI
    init_esp8266_wifi();
#endif

#if defined(WITH_SERIAL) && defined(WITH_SERIAL_COMMANDS)
    serial_prompt();
#endif
} // end setup()


//-------------------------------------------------------------------------------------------
// loop()
//-------------------------------------------------------------------------------------------
void loop()
{
#ifdef WITH_OTA
    // check for over-the-air updates
    ArduinoOTA.handle();
#endif

#if defined(WITH_FRONT_ENV_SENSOR) && FRONT_ENV_SENSOR_TYPE == DHT22_SENSOR
    // read front environment sensor
    if ((millis() - frontDHT22Sensor.lastRead()) > FRONT_ENV_SENSOR_READ_INTERVAL) {
      read_front_dht22();
    }
#endif

#if defined(WITH_REAR_ENV_SENSOR) && REAR_ENV_SENSOR_TYPE == DHT22_SENSOR
    // read rear environment sensor
    if ((millis() - rearDHT22Sensor.lastRead()) > REAR_ENV_SENSOR_READ_INTERVAL) {
      read_rear_dht22();
    }
#endif

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
    if ((millis() - lastReportedOnSerial) > SERIAL_REPORT_INTERVAL) {
        serial_report_values();
    }

#ifdef WITH_SERIAL_COMMANDS
    if (Serial.available() > 0) {
      rearFansSpeedPercent = Serial.parseInt();
      // clear trailing RETURN char from input
      Serial.read();
  
#ifdef WITH_REAR_FANS
      setRearFansSpeedPercent(rearFansSpeedPercent);
#endif
      serial_reply();
      serial_prompt();
    }
#endif

#endif  // WITH_SERIAL

    delay(LOOP_DELAY);
} // end loop()


//-------------------------------------------------------------------------------------------
// functions
//-------------------------------------------------------------------------------------------

#ifdef USE_SPI
void init_spi() {
  #ifdef USE_SERIAL_DEBUG
    Serial.println(F("Initialize SPI..."));
  #endif
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);  // needed for MCP4162
}

void writeSPIvalue(int line, int value) {
  #ifdef USE_SERIAL_DEBUG
    Serial.print(F("Write "));
    Serial.print(value);
    Serial.print(F(" to SPI line "));
    Serial.println(line);
  #endif
    digitalWrite(line, LOW);
    SPI.transfer( (value & 0x100) ? 1 : 0 );
    SPI.transfer( value & 0xff ); // send value (0~255)
    digitalWrite(line, HIGH);
}
#endif  // USE_SPI


#ifdef USE_HIGH_PWMFREQ
void init_pwmfreq() {
  #ifdef USE_SERIAL_DEBUG
    Serial.println(F("Using PWM frequency >30kHz"));
  #endif
  #ifdef HW_ARDUINO
    set_pwm_freq_arduino();
  #endif
  #ifdef HW_ESP8266
    analogWriteFreq(PWMFREQ_ESP8266);
  #endif
}

#ifdef HW_ARDUINO
/*
 * boilerplate code to set PWM frequency on Arduino
 */
void set_pwm_freq_arduino()
{
  #ifdef USE_SERIAL_DEBUG
    Serial.println(F("Initialize PWM frequency..."));
  #endif
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
#endif  // USE_HIGH_PWMFREQ


#ifdef WITH_REAR_FANS
/* functions for rear fans */

void init_rear_fans() {
  #ifdef USE_SERIAL_DEBUG
    Serial.println(F("Initialize rear fans to zero..."));
  #endif
  #if REAR_FANS_CONTROL == PWM_CONTROL
    pinMode(REAR_FANS_PWM_PIN, OUTPUT);
    // initialize fans at zero speed
    analogWrite(REAR_FANS_PWM_PIN, 0);
  #endif
  #if REAR_FANS_CONTROL == MCP4162_CONTROL
    pinMode(SPI_SELECT_REAR_FANS_PIN, OUTPUT);
    writeSPIvalue(SPI_SELECT_REAR_FANS_PIN, 0);
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
  #ifdef USE_SERIAL_DEBUG
    Serial.print(F("Set rear fans speed% to "));
    Serial.println(rearFansSpeedPercent);
  #endif
  #if REAR_FANS_CONTROL == PWM_CONTROL
    rearFansPwmValue = (rearFansSpeedPercent / 100.0) * (ANALOG_WRITE_RANGE - 1);
    analogWrite(REAR_FANS_PWM_PIN, rearFansPwmValue);
  #endif
  #if REAR_FANS_CONTROL == MCP4162_CONTROL
    rearFansSPIValue = (rearFansSpeedPercent / 100.0) * (SPI_WRITE_RANGE - 1);
    writeSPIvalue(SPI_SELECT_REAR_FANS_PIN, rearFansSPIValue);
  #endif
}

#ifdef WITH_REAR_FANS_VOLTAGE
void read_rear_fans_voltage() {
    int sum       = 0;      // sum of samples taken
    int count     = 0;      // current sample number
    //float voltage = 0.0;

    // take a number of analog samples and add them up
    while (count < REAR_FANS_VOLTAGE_NUM_SAMPLES) {
        sum += analogRead(REAR_FANS_VOLTAGE_PIN);
        count++;
        delay(10);
    }
    // calculate the voltage
    //voltage = ((float)sum / (float)REAR_FANS_VOLTAGE_NUM_SAMPLES * REAR_FANS_VOLTAGE_CALIBRATED) / 1024.0;
    rearFansAnalogIn = ((float)sum / (float)REAR_FANS_VOLTAGE_NUM_SAMPLES);

    // send voltage for display on Serial Monitor
    // voltage multiplied by 11 when using voltage divider that
    // divides by 11. 11.132 is the calibrated voltage divide
    // value
    //Serial.print(voltage * 11.132);
    //Serial.print(voltage);
    //Serial.println (" V");

    lastReadRearFansVoltage = millis();
}
#endif  // WITH_REAR_FANS_VOLTAGE
#endif  // WITH_REAR_FANS


#ifdef WITH_WATER_PUMP
/* functions for water pump */

// called from setup()
void init_water_pump() {
  #ifdef USE_SERIAL_DEBUG
    Serial.println(F("Initialize water pump to zero..."));
  #endif
  #if WATER_PUMP_CONTROL == PWM_CONTROL
    pinMode(WATER_PUMP_PWM_PIN, OUTPUT);
    // initialize pump at zero speed
    analogWrite(WATER_PUMP_PWM_PIN, 0);
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
  #ifdef USE_SERIAL_DEBUG
    Serial.print(F("Set pump speed% to "));
    Serial.println(pumpSpeedPercent);
  #endif
  #if WATER_PUMP_CONTROL == PWM_CONTROL
    pumpPwmValue = (pumpSpeedPercent / 100.0) * (ANALOG_WRITE_RANGE - 1);
    analogWrite(WATER_PUMP_PWM_PIN, pumpPwmValue);
  #endif
  #if WATER_PUMP_CONTROL == MCP4162_CONTROL
    // unimplemented
  #endif
}

#ifdef WITH_WATER_PUMP_VOLTAGE
void read_water_pump_voltage() {
    int sum       = 0;      // sum of samples taken
    int count     = 0;      // current sample number
    //float voltage = 0.0;

    // take a number of analog samples and add them up
    while (count < WATER_PUMP_VOLTAGE_NUM_SAMPLES) {
        sum += analogRead(WATER_PUMP_VOLTAGE_PIN);
        count++;
        delay(10);
    }
    // calculate the voltage
    //voltage = ((float)sum / (float)WATER_PUMP_VOLTAGE_NUM_SAMPLES * WATER_PUMP_VOLTAGE_CALIBRATED) / 1024.0;
    pumpAnalogIn = ((float)sum / (float)WATER_PUMP_VOLTAGE_NUM_SAMPLES);

    // send voltage for display on Serial Monitor
    // voltage multiplied by 11 when using voltage divider that
    // divides by 11. 11.132 is the calibrated voltage divide
    // value
    //Serial.print(voltage * 11.132);
    //Serial.print(voltage);
    //Serial.println (" V");

    lastReadPumpVoltage = millis();
}
#endif  // WITH_WATER_PUMP_VOLTAGE
#endif  // WITH_WATER_PUMP


#ifdef WITH_FRONT_ENV_SENSOR
#if FRONT_ENV_SENSOR_TYPE == DHT22_SENSOR
/* functions for front DHT22 env sensor */
void init_front_env_sensor() {
  #ifdef USE_SERIAL_DEBUG
    Serial.println(F("Initialize front DHT22 sensor..."));
  #endif
    // maybe adjust DHT22 sensor offsets (example)
    //frontDHT22Sensor.setHumOffset(10);
    //frontDHT22Sensor.setTempOffset(-3.5);
}

void read_front_dht22() {
    frontDHT22Sensor.read();
    frontEnvSensorHumidity    = frontDHT22Sensor.getHumidity();
    frontEnvSensorTemperature = frontDHT22Sensor.getTemperature();
}
#endif
#endif  // WITH_REAR_ENV_SENSOR


#ifdef WITH_REAR_ENV_SENSOR
#if REAR_ENV_SENSOR_TYPE == DHT22_SENSOR
/* functions for rear DHT22 env sensor */
void init_rear_env_sensor() {
  #ifdef USE_SERIAL_DEBUG
    Serial.println(F("Initialize rear DHT22 sensor..."));
  #endif
    // maybe adjust DHT22 sensor offsets (example)
    //rearDHT22Sensor.setHumOffset(10);
    //rearDHT22Sensor.setTempOffset(-3.5);
}

void read_rear_dht22() {
    rearDHT22Sensor.read();
    rearEnvSensorHumidity    = rearDHT22Sensor.getHumidity();
    rearEnvSensorTemperature = rearDHT22Sensor.getTemperature();
}
#endif
#endif  // WITH_REAR_ENV_SENSOR


#ifdef WITH_ESP8266_WIFI
/* functions for WiFi support */
void connect_wifi_esp8266()
{
    // Connect to WiFi network
  #ifdef USE_SERIAL_DEBUG
    Serial.println();
    Serial.print(F("Connecting to "));
    Serial.println(WIFI_SSID);
  #endif
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
  #ifdef USE_SERIAL_DEBUG
      Serial.print(F("."));
  #endif
    }
    ip = WiFi.localIP();

  #ifdef USE_SERIAL_DEBUG
    Serial.println();
    Serial.print(F("Connected to "));
    Serial.println(WIFI_SSID);
    Serial.print(F("IP address: "));
    Serial.println(ip);
  #endif
}

#ifdef WITH_ESP8266_HTTPSRV
/* functions for embedded web server */
// Handle Root URI
void rootPage() {
    webserver.send(200, "text/plain", 
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
#ifdef WITH_REAR_ENV_SENSOR
                   "Temperature: " + String(rearEnvSensorTemperature) + "C\r\n" +
                   "Humidity: " + String(rearEnvSensorHumidity) + "%\r\n" +
#endif
                   "");
}
 
// Handle 404
void notfoundPage(){
    webserver.send(404, "text/plain", F("404: Not found"));
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
#ifdef WITH_SERIAL
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_FS
        type = "filesystem";
      }
      // NOTE: if updating FS this would be the place to unmount FS using FS.end()
      Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });
#endif  // WITH_SERIAL
  #ifdef USE_SERIAL_DEBUG
    Serial.print(F("Start OTA on port "));
    Serial.println(OTA_PORT);
  #endif
    ArduinoOTA.begin();
#endif  // WITH_OTA

#ifdef WITH_ESP8266_HTTPSRV
  #ifdef USE_SERIAL_DEBUG
    Serial.print(F("Start web server on port "));
    Serial.println(HTTPSRV_PORT);
  #endif
    start_web_server();
#endif
}
#endif  // WITH_ESP8266_WIFI

#ifdef WITH_SERIAL
/* functions for serial console */

void init_serial() {
    Serial.begin(SERIAL_BAUD);
    delay(SERIAL_DELAY);  // grace period for reinit after reboot
  #ifdef USE_SERIAL_DEBUG
    Serial.print(F("Detected Board: "));
    Serial.println(HW_NAME);
  #endif
}

void serial_report_values() {
    Serial.println();
  #ifdef WITH_REAR_FANS
    Serial.print(F("Fan speed%: "));
    Serial.println(rearFansSpeedPercent);
    Serial.print(F("PWM value: "));
    Serial.println(rearFansPwmValue);
  #ifdef WITH_REAR_FANS_VOLTAGE
    Serial.print(F("AnalogV: "));
    Serial.println(rearFansAnalogIn);
  #endif
  #endif
  #ifdef WITH_WATER_PUMP
    Serial.print(F("Pump speed%: "));
    Serial.println(pumpSpeedPercent);
    Serial.print(F("PWM value: "));
    Serial.println(pumpPwmValue);
  #ifdef WITH_WATER_PUMP_VOLTAGE
    Serial.print(F("AnalogV: "));
    Serial.println(pumpAnalogIn);
  #endif
  #endif
  #ifdef WITH_REAR_ENV_SENSOR
    Serial.print(F("Temperature: "));
    Serial.println(rearEnvSensorTemperature);
    Serial.print(F("Humidity%: "));
    Serial.println(rearEnvSensorHumidity);
  #endif

    lastReportedOnSerial = millis();
}

#ifdef WITH_SERIAL_COMMANDS
void serial_prompt() {
    Serial.println();
    Serial.println(F("Enter fan speed (0 - 100 %):"));
}

void serial_reply() {
    Serial.println();
    Serial.print(F("Current fan speed: "));
    Serial.println(rearFansSpeedPercent);
}
#endif  // WITH_SERIAL_COMMANDS
#endif  // WITH_SERIAL
