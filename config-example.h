// Example configuration of station data
// Copy to config.h and modify values as necessary.

const char* ssid = "first-wifi";
const char* password = "first-wifi-password";

const char* ssid_backup = "backup-wifi";
const char* password_backup = "backup-wifi-password";

// Define to enable password (untested)
// #define OTA_PASSWORD "ota123"

// Static Wifi config
IPAddress staticIP(192,168,2,11);
IPAddress gateway(192,168,2,1);
IPAddress subnet(255,255,255,0);
IPAddress dnsIP(192,168,2,1);
IPAddress dnsIPA(8,8,8,8);
