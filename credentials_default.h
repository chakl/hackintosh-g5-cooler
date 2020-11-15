/*
 * this file holds credentials and other confidential data, separate from the main code
 * 
 * copy this to file credentials.h, enter your own credentials, and do not include
 * your credentials.h in version control systems such as git!
 */

#ifdef WITH_ESP8266_WIFI
  #define WIFI_SSID "My WLAN"
  #define WIFI_PASS "My WLAN password"
#ifdef WITH_OTA
  #define OTA_PASS "admin"
  // MD5 (echo "admin" | md5sum)
  //#define OTA_PASS_HASH "456b7016a916a4b178dd72b947c152b7"
#endif
#endif
