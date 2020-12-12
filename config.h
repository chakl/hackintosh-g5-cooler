//---------------------------------------------------------------------------------------------------------------------
// pin config per board type
//---------------------------------------------------------------------------------------------------------------------
#ifdef HW_ARDUINO
  #define REAR_FANS_PWM_PIN 3
  #define REAR_FANS_VOLTAGE_PIN A0
  #define WATER_PUMP_PWM_PIN 11
  #define WATER_PUMP_VOLTAGE_PIN A1
  #define FRONT_ENV_SENSOR_PIN 7
  #define REAR_ENV_SENSOR_PIN 8
  // status LED pins, if used
  //#define STATUS_GREEN_LED_PIN x
  //#define STATUS_RED_LED_PIN x
  // SPI pins, if used (for MCP4162)
  //#define SPI_SELECT_REAR_FANS_PIN 10
  //#define SPI_SELECT_WATER_PUMP_PIN 9
  //#define SPI_MOSI_PIN 11
  //#define SPI_MISO_PIN 12
  //#define SPI_SCK_PIN 13
#endif

#ifdef HW_ESP8266
  // pin number values refer to GPIO#, not D# printed on Wemos module
  #define REAR_FANS_PWM_PIN 14        // D5
  #define REAR_FANS_VOLTAGE_PIN A0
  //#define WATER_PUMP_PWM_PIN 12       // D6
  //#define WATER_PUMP_VOLTAGE_PIN A0
  #define FRONT_ENV_SENSOR_PIN 5      // D1 XXX
  #define REAR_ENV_SENSOR_PIN 5       // D1
  // status LED pins, if used
  //#define STATUS_GREEN_LED_PIN x
  //#define STATUS_RED_LED_PIN x
  // SPI pins, if used (for MCP4162)
  //#define SPI_SELECT_REAR_FANS_PIN 15 // D8
  //#define SPI_SELECT_WATER_PUMP_PIN 4 // D2
  //#define SPI_MOSI_PIN 13             // D7
  //#define SPI_MISO_PIN 12             // D6
  //#define SPI_SCK_PIN 14              // D5
#endif

//---------------------------------------------------------------------------------------------------------------------
// config section - comment out #define WITH_* to disable features
//---------------------------------------------------------------------------------------------------------------------
// support rear fans speed control
#define WITH_REAR_FANS
// how to control rear fans speed (either PWM or MCP4162)
#define REAR_FANS_CONTROL PWM_CONTROL
// support fan voltage measurement
#define WITH_REAR_FANS_VOLTAGE                 // requires WITH_REAR_FANS, otherwise ignored
#define REAR_FANS_VOLTAGE_READ_INTERVAL 2000   // 2 secs
#define REAR_FANS_VOLTAGE_NUM_SAMPLES 5
//#define REAR_FANS_VOLTAGE_CALIBRATED 5.0

// support water pump power control
//#define WITH_WATER_PUMP
// how to control water pump power (either PWM or MCP4162)
#define WATER_PUMP_CONTROL PWM_CONTROL
// support water pump voltage measurement
//#define WITH_WATER_PUMP_VOLTAGE              // requires WITH_WATER_PUMP, otherwise ignored
#define WATER_PUMP_VOLTAGE_READ_INTERVAL 2000  // 2 secs
#define WATER_PUMP_VOLTAGE_NUM_SAMPLES 5
//#define WATER_PUMP_VOLTAGE_CALIBRATED 5.0

// support front temperature/humidity sensor
#define WITH_FRONT_ENV_SENSOR
#define FRONT_ENV_SENSOR_TYPE DHT22_SENSOR      // type of front sensor (DHT22 only)
#define FRONT_ENV_SENSOR_READ_INTERVAL 3000     // DHT22 requires min 2 secs between readings

// support rear temperature/humidity sensor
#define WITH_REAR_ENV_SENSOR
#define REAR_ENV_SENSOR_TYPE DHT22_SENSOR      // type of rear sensor (DHT22 only)
#define REAR_ENV_SENSOR_READ_INTERVAL 3000     // DHT22 requires min 2 secs between readings

// support bicolor status LED
//#define WITH_BICOLOR_STATUS_LED

// support WIFI features if hardware is ESP8266
#ifdef HW_ESP8266
  // use WIFI (comment out to disable)
  #define WITH_ESP8266_WIFI
  // run HTTP server (requires WITH_ESP8266_WIFI, otherwise ignored)
  #define WITH_ESP8266_HTTPSRV
  #define HTTPSRV_PORT 80
  // support over-the-air updates (requires WITH_ESP8266_WIFI, otherwise ignored)
  #define WITH_OTA
  #define OTA_PORT 8266
  #define OTA_HOSTNAME "g5-cooler"
#endif  // HW_ESP8266

// support WIFI features if hardware is Arduino
#ifdef HW_ARDUINO
  // use WIFI (set only if an ESP8266-01 is attached)
  //#define WITH_MODEM_WIFI
#endif  // HW_ARDUINO

// serial console
#define WITH_SERIAL
#define SERIAL_BAUD 115200
#define SERIAL_DELAY 2000
#define SERIAL_REPORT_INTERVAL 5000
// accept commands from the serial console (requires WITH_SERIAL, otherwise ignored)
#define WITH_SERIAL_COMMANDS

// debugging (requires WITH_SERIAL, otherwise ignored)
#define WITH_DEBUG

// recommended
#define WITH_HIGH_PWMFREQ
#define PWMFREQ_ESP8266 32000

// include credentials from separate file
#include "credentials.h"

#define LOOP_DELAY 500  // msec
