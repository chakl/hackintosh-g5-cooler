// board config
//---------------------------------------------------------------------------------------------------------------------
//#define HW_ARDUINO  // deprecated since Aug 1 2021
#define HW_ESP8266
//#define HW_ESP32  // experimental, not fully supported

// pin config per board type
//---------------------------------------------------------------------------------------------------------------------
#ifdef HW_ARDUINO  // deprecated since Aug 1 2021
  // pins for PWM output (fans, pump)
  #define PWM_CONTROL1_PIN 3
  #define PWM_CONTROL2_PIN 11
  // pins for analog input (tank, voltages)
  #define ANALOG_IN1_PIN A0
  #define ANALOG_IN2_PIN A1
  #define ANALOG_IN3_PIN A2
  #define ANALOG_IN4_PIN A3
  #define ANALOG_IN5_PIN A4
  // digital inputs (env sensors)
  #define DIGITAL_IN1_PIN 7  // (for DHT22 only)
  #define DIGITAL_IN2_PIN 8  // (for DHT22 only)
  // I2C
  #define I2C_SDA_PIN A4
  #define I2C_SCL_PIN A5
  // SPI(for MCP4162, pin conflicts!)
  //#define SPI_MOSI_PIN 11
  //#define SPI_MISO_PIN 12
  //#define SPI_SCK_PIN 13
  //#define SPI_SELECT1_PIN 10
  //#define SPI_SELECT2_PIN 9

  //#define REAR_FANS_VOLTAGE_PIN A0
  //#define WATER_PUMP_VOLTAGE_PIN A1
#endif

#ifdef HW_ESP8266
  //// pin number values refer to GPIO#, not D# printed on Wemos module
  // pins for PWM output (fans, pump)
  #define PWM_CONTROL1_PIN 0          // D3
  #define PWM_CONTROL2_PIN 2          // D4
  //#define PWM_CONTROL3_PIN x
  // pins for analog input (tank, voltages)
  #define ANALOG_IN1_PIN A0
  // digital inputs (env sensors)
  #define DIGITAL_IN1_PIN 13          // D7 (for DHT22 only)
  #define DIGITAL_IN2_PIN 15          // D8 (for DHT22 only)
  // digital outputs (LEDs)
  //#define DIGITAL_OUT1_PIN x
  //#define DIGITAL_OUT2_PIN x
  //#define DIGITAL_OUT3_PIN x
  // I2C
  #define I2C_SDA_PIN 4               // D2
  #define I2C_SCL_PIN 5               // D1
  // SPI (for MCP4162, pin conflicts!)
  //#define SPI_MOSI_PIN 13             // D7
  //#define SPI_MISO_PIN 12             // D6
  //#define SPI_SCK_PIN 14              // D5
  //#define SPI_SELECT1_PIN 15          // D8
  //#define SPI_SELECT2_PIN 4           // D2
  // pins to 4051 MUX (if used)
  #define MUX4051_OUT ANALOG_IN1_PIN  // (conflicts!)
  #define MUX4051_ADDR_A 14           // D5
  #define MUX4051_ADDR_B 12           // D6
  #define MUX4051_ADDR_C 13           // D7
#endif

#ifdef HW_ESP32
  // pin number values refer to GPIO#, not D# printed on Wemos module
  // pins for PWM output (fans, pump)
  #define PWM_CONTROL1_PIN 16
  #define PWM_CONTROL2_PIN 17
  //#define PWM_CONTROL3_PIN x
  // pins for analog input (tank, voltages)
  //#define ANALOG_IN1_PIN A0
  //#define ANALOG_IN2_PIN A1
  //#define ANALOG_IN3_PIN A2
  //#define ANALOG_IN4_PIN A3
  //#define ANALOG_IN5_PIN A4
  // digital inputs (env sensors)
  //#define DIGITAL_IN1_PIN x
  //#define DIGITAL_IN2_PIN x
  // digital outputs (LEDs)
  //#define DIGITAL_OUT1_PIN x
  //#define DIGITAL_OUT2_PIN x
  //#define DIGITAL_OUT3_PIN x
  // I2C
  #define I2C_SDA_PIN 21
  #define I2C_SCL_PIN 22
  // SPI
#endif

//---------------------------------------------------------------------------------------------------------------------
// config section - comment out #define WITH_* to disable features
//---------------------------------------------------------------------------------------------------------------------
#define HOSTNAME "g5-cooler"

// generic voltage reading, for both WITH_TANK_LEVEL and WITH_VOLTAGE_MEASURE
#define VOLTAGE_RAIL_REF 15.0
#define VOLTAGE_MCURAIL_REF 5.0
#define VOLTAGE_NUM_SAMPLES 5
#define VOLTAGE_READ_DELAY 10

// support rear fans speed control
#define WITH_REAR_FANS
#ifdef WITH_REAR_FANS
  // how to control rear fans speed (either PWM or MCP4162)
  #define REAR_FANS_CONTROL PWM_CONTROL
  // support fan voltage measurement
  //#define WITH_REAR_FANS_VOLTAGE                 // requires WITH_REAR_FANS, otherwise ignored
  //#define REAR_FANS_VOLTAGE_READ_INTERVAL 2000   // 2 secs
  //#define REAR_FANS_VOLTAGE_NUM_SAMPLES 5
  //#define REAR_FANS_VOLTAGE_CALIBRATED 5.0
#endif

// support water pump power control
#define WITH_WATER_PUMP
#ifdef WITH_WATER_PUMP
  // how to control water pump power (either PWM or MCP4162)
  #define WATER_PUMP_CONTROL PWM_CONTROL
  // support water pump voltage measurement
  //#define WITH_WATER_PUMP_VOLTAGE              // requires WITH_WATER_PUMP, otherwise ignored
  //#define WATER_PUMP_VOLTAGE_READ_INTERVAL 2000  // 2 secs
  //#define WATER_PUMP_VOLTAGE_NUM_SAMPLES 5
  //#define WATER_PUMP_VOLTAGE_CALIBRATED 5.0
#endif

// support front temperature/humidity sensor
//#define WITH_FRONT_ENV_SENSOR
#ifdef WITH_FRONT_ENV_SENSOR
  #define FRONT_ENV_SENSOR_TYPE AM2320_SENSOR_     // type of front sensor (DHT22 or AM2320)
  #define FRONT_ENV_SENSOR_READ_INTERVAL 3000     // DHT22 requires min 2 secs between readings
#endif

// support rear temperature/humidity sensor
//#define WITH_REAR_ENV_SENSOR
#ifdef WITH_REAR_ENV_SENSOR
  #define REAR_ENV_SENSOR_TYPE AM2320_SENSOR_     // type of rear sensor (DHT22 or AM2320)
  #define REAR_ENV_SENSOR_READ_INTERVAL 3000     // DHT22 requires min 2 secs between readings
#endif

// support tank level sensor
#define WITH_TANK_LEVEL
#ifdef WITH_TANK_LEVEL
  #define TANK_LEVEL_PIN ANALOG_IN1_PIN
  #define TANK_LEVEL_READ_INTERVAL 900000        // every 15min
  #define TANK_LEVEL_REF_VOLTAGE VOLTAGE_RAIL_REF  // sensor driven with 15V
#endif

// support voltage measurements
//#define WITH_VOLTAGE_MEASURE
#ifdef WITH_VOLTAGE_MEASURE
  #define VOLTAGE_READ_INTERVAL 2000
  // ESP8266 have only one analog input port, so a port multiplexer is required
  #if defined(HW_ESP8266) && defined(WITH_VOLTAGE_MEASURE)
    #define ANALOG_MUX_TYPE HC4051_MUX_             // type of analog mux (ADS1115, HC4051 or PCF8591)
    #define MUX_PIN_RAIL_VOLTAGE 0
    #define MUX_PIN_MCURAIL_VOLTAGE 1
    #define MUX_PIN_FANS_VOLTAGE 2
    #define MUX_PIN_PUMP_VOLTAGE 3
    #if ANALOG_MUX_TYPE == ADS1115_MUX_ || ANALOG_MUX_TYPE == PCF8591_MUX_
      #define ANALOG_MUX_I2C
    #endif
  #endif  // HW_ESP8266
#endif  // WITH_VOLTAGE_MEASURE

// support bicolor status LED
//#define WITH_BICOLOR_STATUS_LED

#define WITH_FS
#ifdef WITH_FS
#ifdef HW_ARDUINO
  #error "WITH_FS currently not supported on Arduino"
#endif
  #define FS_TYPE LITTLEFS_  // either SPIFFS or LITTLEFS
#endif

// support WIFI features if hardware is ESP8266
#ifdef HW_ESP8266
  // use WIFI (comment out to disable)
  #define WITH_ESP8266_WIFI
  #ifdef WITH_ESP8266_WIFI
    // run HTTP server (comment out to disable)
    #define WITH_ESP8266_HTTPSRV
    #ifdef WITH_ESP8266_HTTPSRV
      #ifndef WITH_FS
      #error "WITH_ESP8266_HTTPSRV requires WITH_FS"
      #endif
      #define HTTPSRV_XMLBUFSIZE 1024
      #define HTTPSRV_PORT 80
    #endif
    // support over-the-air updates (comment out to disable)
    #define WITH_OTA
    #define OTA_PORT 8266
    #define OTA_HOSTNAME HOSTNAME
  #endif
#endif  // HW_ESP8266

// serial console
#define WITH_SERIAL
#ifdef WITH_SERIAL
  #define SERIAL_BAUD 115200
  #define SERIAL_DELAY 1000  // initial delay for serial port communication to settle
  #define SERIAL_REPORT_INTERVAL 5000
  // accept commands from the serial console (requires WITH_SERIAL, otherwise ignored)
  #define WITH_SERIAL_COMMANDS
#endif

// debugging (currently unused)
//#define WITH_DEBUG

// use high PWM frequency (recommended if PWM is used)
#define WITH_HIGH_PWMFREQ
// Arduinos get set to ca 32kHz; ESP8266 and ESP32 can set PWM frequency by function call
#define PWMFREQ_ESP 32000

// include credentials from separate file
#include "credentials.h"

#define LOOP_DELAY 500  // msec
