/*
 * Target: Lolin D1 Mini
 */
#include "Arduino.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoJson.h>
#include <ModbusIP_ESP8266.h>
#include <U8x8lib.h>
// #include <WiFiManager.h>
#include <ArduinoOTA.h>

#define VERSION "v2021.09.26"

#include "config.h"

#define CURRENT_LIMIT_BOOT 13
#define CURRENT_LIMIT_MIN 6
#define CURRENT_LIMIT_ONE 20
#define CURRENT_LIMIT_THREE 16
#define PP_CURRENT_LIMIT 20

#define THREE_PHASE_RELAY D8
#define THREE_PHASE_OFF 0
#define THREE_PHASE_ON 1
// Unused D5
// Unused A0

#define PUSH_BUTTON D6
#define PUSH_BUTTON_LED D7

#define RXPin        D4  // Serial Receive pin
#define TXPin        D3  // Serial Transmit pin

//RS485 control
#define SERIAL_COMMUNICATION_CONTROL_PIN D0 // Transmission set pin
#define RS485_TX_PIN_VALUE HIGH
#define RS485_RX_PIN_VALUE LOW

U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(/*reset=*/ U8X8_PIN_NONE, /* clock=*/ D1, /* data=*/ D2);

SoftwareSerial RS485Serial(RXPin, TXPin); // RX, TX
ESP8266WebServer server(80);
ModbusIP mb;


const byte evse_slave_id = 0x01;
const byte sdm_slave_id = 0x02;

int byteSend;

float sdm_voltage, sdm_current, sdm_power, sdm_energy, sdm_frequency,
      sdm_import_energy, sdm_export_energy, sdm_import_power, sdm_export_power;

unsigned long sdm_updated = 0, evse_updated = 0;

// 4: constant led, 8: on ventilation, 16: clear RCD error
// 16384: disable EVSE, 8192: disable EVSE after charge
// Register 2005
#define EVSE_CONFIG (4 | 8 | 16)
#define EVSE_CONFIG_DISABLE_CHARGING 16384

bool evse_charging = false, evse_vehicle_present = false, evse_charging_enabled = true;
unsigned int evse_config[10], evse_fw, evse_amp, evse_vehicle_state, evse_state, evse_status,
         evse_set_current = 0, evse_reg_charging = 0, evse_config_reg;
         
int request_evse_set_current = -1;
bool request_evse_stop_charging = false, request_evse_start_charging = false;

#define MODE_MANUAL_THREE 3
#define MODE_MANUAL 1
#define MODE_AUTO 2

int evse_mode = MODE_AUTO;
int request_evse_mode = -1;
int request_phases = -1;

unsigned long chargeStart = 0, chargeDuration = 0;
float chargeEnergy = 0, chargeEnergyStart = 0;
#define STATE_INIT 1
#define STATE_READY 2
#define STATE_CHARGING 3
#define STATE_ERROR 4
unsigned int state = STATE_INIT;

#define PHASE_UNKNOWN 0
#define PHASE_ONE 1
#define PHASE_THREE 3

unsigned int phase_state = PHASE_UNKNOWN;
String state_error, last_error;

// WiFiManager wifiManager;
ESP8266WiFiMulti wifiMulti;

#define CRC_SEED 0xFFFF  //initialization for CRC16
#define CRC_GP   0xA001  //generating polynomial

// #define MODBUS_DEBUG 1

void modbus_crc_single(unsigned char b, unsigned int* CRC)
{
  int carry, i ;

  CRC[0] ^= b & 0xFF;
  for (i = 0; i < 8; i++)
  {
    carry = CRC[0] & 0x0001;
    CRC[0] >>= 1;
    if (carry) CRC[0] ^= CRC_GP;
  }
}

void modbus_crc(unsigned char *b, int len, unsigned int *CRC) {
  *CRC = CRC_SEED;
  for (int i = 0; i < len; i++) {
    modbus_crc_single(b[i], CRC);
  }
}
/*
   01 The slave address (always 01)
  03 Function code (03 is for read holding registers function), 16 for write multiple registers
  04 The number of data bytes to follow (2 registers x 2 bytes each = 4)
  00 20 The contents of register 1000 (32A)
  00 20 The contents of register 1001 (32A)
  FA 21 CRC-16 Modbus (cyclic redundancy check)
*/

void byte2float(byte *data, float *res) {
  ((byte *)res)[0] = data[3];
  ((byte *)res)[1] = data[2];
  ((byte *)res)[2] = data[1];
  ((byte *)res)[3] = data[0];
}

void float2byte(float f, byte *data) {
  data[3] = ((byte *)&f)[0];
  data[2] = ((byte *)&f)[1];
  data[1] = ((byte *)&f)[2];
  data[0] = ((byte *)&f)[3];
}

// #define MODBUS_DEBUG 1

// read holding registers: modbus_send_command(0x01, 3, 1000, 10, 0);
// read input registers : modbus_send_command(0x01, 4, 1000, 10, 0);
// write multiple holding registers : modbus_send_command(0x01, 16, 1000, 10, int16);
int modbus_send_command(byte slave_id, byte function_code, int address, int count,
                        byte *payload, int payload_length,
                        byte *reply, int reply_length) {
  unsigned char data[10];
  unsigned int CRC;
  byte crc_offset = 6;
  int err = 0;

  data[0] = slave_id;
  data[1] = function_code; // read
  data[2] = address >> 8;
  data[3] = address & 0xff;
  data[4] = count >> 8;
  data[5] = count & 0xff;
  for (int i = 0; i < payload_length; i += 1) {
    data[6 + i] = payload[i];
    crc_offset += 1;
  }
  modbus_crc(data, crc_offset, &CRC);
  data[crc_offset + 1] = CRC >> 8;
  data[crc_offset] = CRC & 0xff;
#ifdef MODBUS_DEBUG
  Serial.print("TX ");
  for (int i = 0; i < (crc_offset + 2); i++) {
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println(".");
#endif
  // flush input queue
  while (RS485Serial.available()) {
    RS485Serial.read();
  }
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_TX_PIN_VALUE);
  delay(1);
  RS485Serial.write(data, crc_offset + 2);
  RS485Serial.flush();
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX_PIN_VALUE);
  delay(1);
  // read return value
  byte buf[4];
  unsigned int rx_crc = CRC_SEED;
  //delay(10);
  int rx_count = RS485Serial.readBytes(buf, 3);

  if (rx_count != 3) {
    err = 6;
    return err;
  }
  int rx_slave = buf[0];
  int rx_function = buf[1];
  int rx_len = buf[2];
  modbus_crc_single(buf[0], &rx_crc);
  modbus_crc_single(buf[1], &rx_crc);
  modbus_crc_single(buf[2], &rx_crc);

#ifdef MODBUS_DEBUG
  Serial.print("RX ");
  Serial.print(rx_slave);
  Serial.print(" F");
  Serial.print(rx_function);
  Serial.print(" L");
  Serial.print(rx_len);
  Serial.print(": ");
#endif

  if (rx_slave != slave_id) {
    err = 3;
  }
  if (rx_function != function_code) {
    err = 4;
  }

  for (int i = 0; i < rx_len; i += 1) {
    rx_count = RS485Serial.readBytes(buf, 1);
    if (rx_count == 1) {
#ifdef MODBUS_DEBUG
      Serial.print(buf[0], HEX);
      Serial.print(" ");
#endif
      modbus_crc_single(buf[0], &rx_crc);
      if (i < reply_length) {
        reply[i] = buf[0];
      }
    } else {
#ifdef MODBUS_DEBUG
      Serial.print("TO");
#endif
      err = 2;
      break;
    }
  }
  rx_count = RS485Serial.readBytes(buf, 2);
  if ((buf[0] != (rx_crc & 0xff)) || (buf[1] != (rx_crc >> 8))) {
    Serial.print("CRCerr ");
    Serial.print(buf[0], HEX);
    Serial.print(" ");
    Serial.print(buf[1], HEX);
    Serial.print(" vs. ");
    Serial.print(rx_crc, HEX);
    Serial.print(" ");
    err = 1;
  }
#ifdef MODBUS_DEBUG
  Serial.println(".");
#endif
  return err;
}

int modbus_handle_error(const char *fun, int address, int error) {
  if (error) {
    Serial.print("Modbus Error ");
    Serial.print(fun);
    Serial.print(" ");
    Serial.print(address);
    Serial.print(" ");
    Serial.println(error);
  }
  return error;
}
int modbus_read_input_float(byte slave_id, int address, float *res) {
  byte buf[4], tries = 0;
  int err = 0;
  while (tries < 3) {
    err = modbus_send_command(slave_id, 4, address, 2, NULL, 0, buf, 4);
    if (!err) break;
    tries++;
  }
  if (err) return modbus_handle_error("modbus_read_input_float", address, err);
  byte2float(buf, res);
  return 0;
}

int modbus_read_holding_int(byte slave_id, int address, unsigned int *res) {
  byte buf[4], tries = 0;
  int err = 0;
  while (tries < 3) {
    err = modbus_send_command(slave_id, 3, address, 2, NULL, 0, buf, 4);
    if (!err) break;
    tries++;
  }
  if (err) return modbus_handle_error("modbus_read_holding_int", address, err);
  *res = ((unsigned int)buf[0] << 8) + buf[1];
  return 0;
}

int modbus_read_holding_multi_int(byte slave_id, int address, unsigned int count, unsigned int *res) {
  byte buf[40], tries = 0;
  int err = 0;
  while (tries < 3) {
    err = modbus_send_command(slave_id, 3, address, count, NULL, 0, buf, 40);
    if (!err) break;
    tries++;
  }
  if (err) return modbus_handle_error("modbus_read_holding_multi_int", address, err);
  for (unsigned int i = 0; i < count; i++) {
    res[i] = ((unsigned int)buf[0 + 2 * i] << 8) + buf[1 + 2 * i];
  }
  return 0;
}

int modbus_read_holding_uint32(byte slave_id, int address, uint32_t *res) {
  byte buf[4];
  int err = modbus_send_command(slave_id, 3, address, 2, NULL, 0, buf, 4);
  if (err) return modbus_handle_error("modbus_read_holding_uint32", address, err);
  *res = ((uint32_t)buf[0] << 24) + ((uint32_t)buf[1] << 16) + ((uint32_t)buf[2] << 8) + buf[3];
  return 0;
}


int modbus_read_float(byte slave_id, int address, float *res) {
  byte buf[4];
  int err = modbus_send_command(slave_id, 3, address, 2, NULL, 0, buf, 4);
  if (err) return modbus_handle_error("modbus_read_float", address, err);
  byte2float(buf, res);
  return 0;
}

int modbus_write_float(byte slave_id, int address, float value) {
  byte buf[5];
  buf[0] = 4;
  float2byte(value, buf + 1);
  int err = modbus_send_command(slave_id, 16, address, 2, buf, 5, NULL, 0);
  if (err) return modbus_handle_error("modbus_write_float", address, err);
  return 0;
}


int modbus_write_int(byte slave_id, int address, unsigned int value) {
  byte buf[3], tries = 0;
  int err;
  buf[0] = 2;
  buf[1] = value >> 8;
  buf[2] = value & 0xff;
  while (tries < 3) {
    err = modbus_send_command(slave_id, 16, address, 2, buf, 3, NULL, 0);
    if (!err) break;
    tries++;
  }
  if (err) return modbus_handle_error("modbus_write_int", address, err);
  return 0;
}



#define REG_MODEL 5000
#define REG_SERIAL 5001
#define REG_FW 5007
#define REG_MODE 5009
#define REG_START_STOP 5010
#define REG_AC_L1P 5011
#define REG_AC_L2P 5012
#define REG_AC_L3P 5013
#define REG_AC_P 5014
#define REG_STATUS 5015
#define REG_SET_CURRENT 5016
#define REG_MAX_CURRENT 5017
#define REG_CURRENT 5018
#define REG_CHARGING_TIME 5019
#define REG_ENERGY_FORWARD 5021

uint16_t cbWrite(TRegister* reg, uint16_t val)  {
  if ((state == STATE_INIT) || (state == STATE_ERROR)) {
     return val;
  }
  if (reg->value == val) {
    return val;
  }
  Serial.print("Write. Reg RAW#: ");
  Serial.print(reg->address.address);
  Serial.print(" Old: ");
  Serial.print(reg->value);
  Serial.print(" New: ");
  Serial.println(val);
  switch (reg->address.address) {
    case REG_MODE:
      Serial.println("REG_MODE not implemented");
      break;
    case REG_START_STOP:
      if (val == 1) {
        if (!evse_charging)
          request_evse_start_charging = true;
      } else {
        if (evse_charging)
          request_evse_stop_charging = true;
      }
      break;
    case REG_SET_CURRENT:
      Serial.print("REG_SET_CURRENT to");
      Serial.println(val);
      request_evse_set_current = val;
      break;
  }
  return val;

}

bool cbConn(IPAddress ip) {
  /*
    Serial.print("Modbus connection: ");
    Serial.println(ip);
  */
  return true;
}

uint16_t cbEVSE(TRegister* reg, uint16_t val) {
  Serial.println("cbEVSE callback GET");
  unsigned int r = 0;
  int err = modbus_read_holding_int(evse_slave_id, reg->address.address, &r);
  if (err) {
    Serial.print("ERROR ");
    Serial.println(err);
  }
  return r;
}

uint16_t cbEVSESet(TRegister* reg, uint16_t val) {
  Serial.println("cbEVSE callback SET");
  int err = modbus_write_int(evse_slave_id, reg->address.address, val);
  if (err) {
    Serial.print("ERROR ");
    Serial.println(err);
  }
  return val;
}

void init_modbus() {
  // see https://github.com/victronenergy/dbus-modbus-client/blob/master/ev_charger.py, models
  mb.addHreg(REG_MODEL, 0xc024);
  for (int i = 0; i < 6; i++) {
    mb.addHreg(REG_SERIAL + i, 65);
  }
  mb.addHreg(REG_FW, 1);
  mb.addHreg(REG_FW + 1, 0);
  mb.addHreg(REG_MODE, 1); // 0 MANUAL or 1 for auto
  mb.onSetHreg(REG_MODE, cbWrite, 1);
  mb.addHreg(REG_START_STOP, 0); // or 1 for start
  mb.onSetHreg(REG_START_STOP, cbWrite, 1);
  mb.addHreg(REG_AC_L1P, 1);
  mb.addHreg(REG_AC_L2P, 1);
  mb.addHreg(REG_AC_L3P, 1);
  mb.addHreg(REG_AC_P, 1);

  mb.addHreg(REG_SET_CURRENT, 1);
  mb.onSetHreg(REG_SET_CURRENT, cbWrite, 1);
  mb.addHreg(REG_MAX_CURRENT, 1);
  mb.onSetHreg(REG_MAX_CURRENT, cbWrite, 1);
  mb.addHreg(REG_CURRENT, 10);
  mb.addHreg(REG_CHARGING_TIME, 0);
  mb.addHreg(REG_CHARGING_TIME + 1, 0);

  mb.addHreg(REG_ENERGY_FORWARD, 100);

  // Passthrough for debugging of EVSE
  for (int i = 1000; i < 1010; i++) {
    mb.onGetHreg(i, cbEVSE);
    mb.onSetHreg(i, cbEVSESet, 1);
  }
  for (int i = 2000; i < 2020; i++) {
    mb.onGetHreg(i, cbEVSE);
    mb.onSetHreg(i, cbEVSESet, 1);
  }
  updateModbus();
  mb.onConnect(cbConn);
  mb.server();
}
void updateModbus() {
  // https://github.com/victronenergy/dbus-modbus-client/blob/master/ev_charger.py
  unsigned int p = sdm_power;
  mb.Hreg(REG_MODE, 1);
  mb.Hreg(REG_AC_L1P, p);
  if (phase_state == PHASE_ONE) {
    mb.Hreg(REG_AC_L2P, 0);
    mb.Hreg(REG_AC_L3P, 0);
    mb.Hreg(REG_AC_P, p);
  } else {
    mb.Hreg(REG_AC_L2P, p);
    mb.Hreg(REG_AC_L3P, p);
    mb.Hreg(REG_AC_P, 3 * p);
  }
  /*
         DISCONNECTED    = 0
    CONNECTED       = 1
    CHARGING        = 2
    CHARGED         = 3
    WAIT_SUN        = 4
    WAIT_RFID       = 5
    WAIT_START      = 6
    LOW_SOC         = 7
    GND_ERROR       = 8
    WELD_CON        = 9
    CP_SHORTED      = 10
  */
  int mb_status = 0;
  if (evse_charging) {
    mb_status = 2;
  } else if (evse_vehicle_present) {
    mb_status = 1;
  }
  mb.Hreg(REG_STATUS, mb_status);

  mb.Hreg(REG_SET_CURRENT, evse_set_current);
  mb.Hreg(REG_MAX_CURRENT, PP_CURRENT_LIMIT);

  mb.Hreg(REG_CURRENT, evse_amp * 10);
  
  mb.Hreg(REG_CHARGING_TIME, chargeDuration);

  mb.Hreg(REG_ENERGY_FORWARD, (int)(chargeEnergy*100));


}

DynamicJsonDocument doc(1024);

void getJson() {

  doc["meter"]["updated"] = sdm_updated;
  doc["meter"]["voltage"] = sdm_voltage;
  doc["meter"]["current"] = sdm_current;
  doc["meter"]["power"] = sdm_power;
  doc["meter"]["energy"] = sdm_energy;
  doc["meter"]["frequency"] = sdm_frequency;
  doc["meter"]["energy_import"] = sdm_import_energy;
  doc["meter"]["energy_export"] = sdm_export_energy;
  doc["meter"]["power_import"] = sdm_import_power;
  doc["meter"]["power_export"] = sdm_export_power;
  doc["meter"]["updated"] = sdm_updated;

  doc["evse"]["config_reg"] = evse_config_reg;
  doc["evse"]["charging_reg"] = evse_reg_charging;
  doc["evse"]["charging_enabled"] = evse_charging_enabled;
  doc["evse"]["charging"] = evse_charging;
  doc["evse"]["vehicle_present"] = evse_vehicle_present;
  doc["evse"]["state"] = evse_state;
  doc["evse"]["vehicle_state"] = evse_vehicle_state;
  doc["evse"]["status"] = evse_status;
  doc["evse"]["set_current"] = evse_set_current;
  doc["evse"]["announced_current"] = evse_amp;
  doc["evse"]["firmware"] = evse_fw;
  doc["evse"]["phases"] = phase_state;
  doc["evse"]["mode"] = evse_mode;
  doc["evse"]["updated"] = evse_updated;
  doc["charge"]["duration"] = chargeDuration;
  doc["charge"]["energy"] = chargeEnergy;

  doc["state"] = state;
  doc["state_error"] = state_error;
  doc["last_error"] = last_error;
  doc["uptime"] = millis();
  doc["version"] = VERSION;
  WiFiClient client = server.client();
  client.println("HTTP/1.0 200 OK");
  client.println("Content-Type: application/json");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();

  // server.sendHeader("Content-Type", "application/json");
  // client.println("Content-Type: application/json");
  serializeJson(doc, client);
  client.stop();
}

void getIndex() {
  server.send(200, F("text/html"),
              F("Welcome to the Trixing EVSE Web Server"));
}

void post_charging() {
  Serial.print("Post Change Charging State: ");
  if (server.hasArg("value")) {
    int val = server.arg("value").toInt();
    if (val == 1) {
      Serial.println("Start");
      request_evse_start_charging = true;
      server.send(200, "text/plain", "OK");
      return;
    } else if (val == 2) {
      Serial.println("Stop");
      request_evse_stop_charging = true;
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  Serial.println("Error");
  server.send(400, "text/plain", "ERR value:(1=start, 2=stop)");
}

void post_current() {
  Serial.print("Post Set Current: ");
  if (server.hasArg("value")) {
    int val = server.arg("value").toInt();
    if ((val >= CURRENT_LIMIT_MIN) && (val <= PP_CURRENT_LIMIT)) {
      Serial.println(val);
      request_evse_set_current = val;
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  Serial.println("Error");
  server.send(400, "text/plain", "ERR value:current");
}

void post_mode() {
  Serial.print("Post Mode: ");
  if (server.hasArg("value")) {
    int val = server.arg("value").toInt();
    if (val == 1) {
      Serial.println(val);
      request_evse_mode = val;
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  Serial.println("Error");
  server.send(400, "text/plain", "ERR value:1");
}

void post_phases() {
  Serial.print("Post Three Phase: ");
  if (server.hasArg("value")) {
    int val = server.arg("value").toInt();
    if ((val == 3)  || (val == 1)) {
      Serial.print(val);
      request_phases = val;
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  Serial.println("Error");
  server.send(400, "text/plain", "ERR value:3");
}


String getContentType(String filename) { // convert the file extension to the MIME type
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  return "text/plain";
}

void handleNotFound() {
  String path = server.uri();
  if (path.endsWith("/")) path += "index.html";
  Serial.print("Request: ");
  Serial.print(path);
  if (SPIFFS.exists(path)) {
    String contentType = getContentType(path);
    File file = SPIFFS.open(path, "r");                 // Open it
    size_t sent = server.streamFile(file, contentType); // And send it to the client
    file.close();     // Then close the file again
    Serial.println(" OK");
    return;
  }
  Serial.println(" 404");
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}


void start_webserver() {
  server.on(F("/j"), HTTP_GET, getJson);
  // 0 to disable charging for an hour or until reboot
  server.on(F("/charging"), HTTP_POST, post_charging);
  server.on(F("/current"), HTTP_POST, post_current);
  server.on(F("/mode"), HTTP_POST, post_mode);
  server.on(F("/phases"), HTTP_POST, post_phases);
  server.on("/update", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    }, []() {
      HTTPUpload& upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.setDebugOutput(true);
        WiFiUDP::stopAll();
        Serial.printf("Update: %s\n", upload.filename.c_str());
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if (!Update.begin(maxSketchSpace)) { //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
        Serial.setDebugOutput(false);
      }
      yield();
    });
  // Set not found response
  server.onNotFound(handleNotFound);
  // Start server
  server.close();

  server.begin();
}

int swap_line(const char *new_line, char *old_line, U8X8 u8x8) {
  int i, chg = 0;
  char b[2] = {0, 0};
  //unsigned long t = millis(), t2;
  bool eol = false;
  for (i = 0; i < 8; i++) {
    if (!eol && (new_line[i] == 0)) eol = true;

    if (eol) {
      if (old_line[i] != 0) {
        u8x8.drawString(i * 2, 1, " ");
        old_line[i] = 0;
        chg++;
      }
    } else {
      if (new_line[i] != old_line[i]) {
        b[0] = new_line[i];
        old_line[i] = new_line[i];
        u8x8.drawString(i * 2, 1, b);
        chg++;
      }
    }
  }
  //t2 = millis();


  if (chg > 0) {
    Serial.println(new_line);
    /*
      Serial.print(new_line);
      Serial.print("  ");
      Serial.print(chg);
      Serial.print("x ");
      Serial.print(t2 - t);
      Serial.println(" ms");
    */
  }

  return chg;
}

char line_out_0[9];
char ubuf[32];


volatile unsigned long button_trigger = 0;

ICACHE_RAM_ATTR void push_button() {
  int btn = digitalRead(PUSH_BUTTON);

  Serial.print(millis());
  Serial.print(" ");
  Serial.println(btn);

  if (btn == 0) {
    button_trigger = millis();
    return;
  }

  unsigned long duration =  millis() - button_trigger;
  Serial.print("PUSH BUTTON: ");
  Serial.println(duration);
  if (duration < 10) {
    // ignore
  } else if (duration < 1000) {
    request_evse_mode = 1;
  } else {
    Serial.println("  press too long");
  }
}
/*
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());

  Serial.println(myWiFiManager->getConfigPortalSSID());
  sprintf(ubuf, "WifiConf");
  swap_line(ubuf, line_out_0, u8x8);
}
*/
void update_display(const char *text) {
  sprintf(ubuf, "%s", text);
  swap_line(ubuf, line_out_0, u8x8);
}

void setupOTA() {
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
#ifdef OTA_PASSWORD
  ArduinoOTA.setPassword(OTA_PASSWORD);
#endif
  ArduinoOTA.begin();
}

void setup()  {
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Trixing EVSE Wifi");
  Serial.print("Version " VERSION);
  Serial.println("");

  pinMode(PUSH_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON), push_button, CHANGE);

  pinMode(PUSH_BUTTON_LED, OUTPUT);
  digitalWrite(PUSH_BUTTON_LED, 0);


  pinMode(THREE_PHASE_RELAY, OUTPUT);
  digitalWrite(THREE_PHASE_RELAY, 0);

  pinMode(SERIAL_COMMUNICATION_CONTROL_PIN, OUTPUT);
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX_PIN_VALUE);

  RS485Serial.begin(9600);   // set the data rate
  RS485Serial.setTimeout(100);

  u8x8.setBusClock(800000);
  u8x8.begin();
  u8x8.setFont(u8x8_font_profont29_2x3_r);
  update_display("TrixEVSE");

  wifiMulti.addAP(ssid, password);
  wifiMulti.addAP(ssid_backup, password_backup);
  WiFi.persistent(false);
  WiFi.hostname("trixing-evse");
  WiFi.mode(WIFI_STA);  
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setAutoReconnect(true);

  WiFi.config(staticIP, gateway, subnet, dnsIP, dnsIPA);
  WiFi.begin(ssid, password);
  /*
  // wifiManager.setConfigPortalTimeout(180);
  wifiManager.setConnectTimeout(5000);
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.autoConnect("TrixEVSE");
  */
  sprintf(ubuf, "Trix EVSE");
  swap_line(ubuf, line_out_0, u8x8);

  Serial.print(" - Mount SPIFFS");
  SPIFFS.begin();
  
  Serial.print(" - IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println(" - setup http server");

  start_webserver();


  if (MDNS.begin("trixing-evse")) {
    Serial.println(" - MDNS responder started");
  }
  Serial.println(" - setup modbus/tcp server");
  init_modbus();
  Serial.println(" - setup OTA");
#ifdef OTA_PASSWORD
  Serial.println(" - OTA password " OTA_PASSWORD);
#endif
  setupOTA();
  /*
     from pymodbus.client.sync import ModbusTcpClient
     client = ModbusTcpClient('192.168.178.112')
     reg = client.read_holding_registers(100)
     print('%x' % reg.getRegister(0))
     client.write_register(100, 0x1234)
  */
  Serial.println(" - setup end");

}

int evse_stop_charging() {
  // stop
  modbus_write_int(evse_slave_id, 1004, 1);
  delay(100);
  // disable to prevent re-enabling
  modbus_write_int(evse_slave_id, 2005, EVSE_CONFIG | EVSE_CONFIG_DISABLE_CHARGING);
  return 0;
}

int evse_start_charging() {
  // clear inhibit charging bit
  modbus_write_int(evse_slave_id, 2005, EVSE_CONFIG);
  delay(100);
  // start charging
  modbus_write_int(evse_slave_id, 1004, 0);
  return 0;
}



unsigned int sdm_meter_code, sdm_sw_version;
uint32_t sdm_serial;

int sdmInit() {
  int err =
    modbus_read_holding_uint32(sdm_slave_id, 0xFC00, &sdm_serial) ||
    modbus_read_holding_int(sdm_slave_id, 0xFC02, &sdm_meter_code) ||
    modbus_read_holding_int(sdm_slave_id, 0xFC03, &sdm_sw_version);
  if (err) {
    Serial.print("sdm_init failed ");
    Serial.println(err);
  } else {
    Serial.print("sdm_init success: meter_code ");
    Serial.print(sdm_meter_code);
    Serial.print(", serial ");
    Serial.print(sdm_serial);
    Serial.print(", version ");
    Serial.println(sdm_sw_version);
  }
  return err;
}


int evseInit() {
  int err = modbus_read_holding_multi_int(evse_slave_id, 2000, 10, evse_config);
  err +=  10 * modbus_read_holding_int(evse_slave_id, 1000, &evse_set_current);
  err +=  100 * modbus_read_holding_int(evse_slave_id, 1005, &evse_fw);
  err +=  1000 * modbus_read_holding_int(evse_slave_id, 1002, &evse_vehicle_state);
  
  if (err) {
    Serial.print("evse_init failed ");
    Serial.println(err);
    return err;
  }
  Serial.print("evse_init firmware ");
  Serial.println(evse_fw);

  // Disable PP detection, fixed limit
  if (evse_config[7] != PP_CURRENT_LIMIT) {
    err = modbus_write_int(evse_slave_id, 2007, PP_CURRENT_LIMIT);
    if (err) return err;
  }

  if (evse_config[5] != EVSE_CONFIG) {
    err = modbus_write_int(evse_slave_id, 2005, EVSE_CONFIG);
    if (err) return err;
  }

  // default amps after boot
  if (evse_config[0] != CURRENT_LIMIT_BOOT) {
    err = modbus_write_int(evse_slave_id, 2000, CURRENT_LIMIT_BOOT);
    if (err) return err;
  }
  
  return 0;
}

void evse_configure_set_current() {
  if (request_evse_set_current <= 0) return;
  if (request_evse_set_current == (int)evse_set_current) {
    request_evse_set_current = -1;
    return;
  }

  // write current limit
  if (request_evse_set_current < CURRENT_LIMIT_MIN) {
    request_evse_set_current = -1;
    return;
  }
  if (request_evse_set_current > PP_CURRENT_LIMIT) {
    request_evse_set_current = -1;
    return;
  }
  if (
    ((phase_state == PHASE_ONE) && (request_evse_set_current <= CURRENT_LIMIT_ONE)) ||
    ((phase_state == PHASE_THREE) && (request_evse_set_current <= CURRENT_LIMIT_THREE))
  )
  {
    int err = modbus_write_int(evse_slave_id, 1000, request_evse_set_current);
    if (err) return;
    evse_set_current = request_evse_set_current;
    request_evse_set_current = -1;
  } else {
    request_evse_set_current = -1;
  }
}


const char *vehicle_states[] = { // 1002
  "none", // 0
  "ready", // 1
  "EV is present", // 2
  "charging", // 3
  "charging with ventilation", // 4
  "failure"
};

const char *evse_states[] = { // 1006
  "none",
  "steady 12V", // 1
  "PWM is being generated", // 2
  "OFF, steady 12V"
};


bool evse_relay_on, evse_diode_check_fail, evse_vent_req_fail, evse_wait_pilot_rel, evse_rcd_check_err;
void decodeEvseStatus(unsigned int status) {
  evse_relay_on = (status & 1);

  evse_diode_check_fail = (status & 2);
  evse_vent_req_fail = (status & 4);
  evse_wait_pilot_rel = (status & 8);
  evse_rcd_check_err = (status & 16);
}

void evse_toggle_three_phases(unsigned int newstate) {
  int tries = 10, err;
  unsigned int hw_state;
  if (newstate == phase_state) {
    // nothing to do
    return;
  }
  evse_stop_charging();
  while (tries) {

    err = modbus_read_holding_int(evse_slave_id, 1007, &hw_state);
    if (err) {
      tries--;
      delay(20);
      continue;
    }
    if (!(hw_state & 0x01)) {
      break;// wait until relay is off
    }
    tries--;
    delay(500); // 2.
  }
  if (!tries) {
    // FAILED, do not touch relay!!
    Serial.println("Three phase failed: charge stop");
    return;
  }
  // Write new current config, default to boot config.
  err = modbus_write_int(evse_slave_id, 1000, CURRENT_LIMIT_BOOT);
  if (err) {
    Serial.println("Three phase failed: current config");
    return;
  }
  evse_set_current = CURRENT_LIMIT_BOOT;
  request_evse_set_current = -1;
  // PHASE L2 & L3 RELAY ON or OFF
  if (newstate == PHASE_THREE) {
    digitalWrite(THREE_PHASE_RELAY, THREE_PHASE_ON);
    phase_state = PHASE_THREE;
  } else {
    digitalWrite(THREE_PHASE_RELAY, THREE_PHASE_OFF);
    phase_state = PHASE_ONE;
  }
  // short delay to avoid too fast trigger of relay
  delay(500);
  // enable charging possibility again
  evse_start_charging();
}


int update_modbus_cnt = 0;
int updateModbusData() {
  int reterr = 0;
  bool extended_update = ((update_modbus_cnt % 10) == 0);
  update_modbus_cnt++;

  int err =
    modbus_read_input_float(sdm_slave_id, 0x0006, &sdm_current) ||
    modbus_read_input_float(sdm_slave_id, 0x000C, &sdm_power) ||
    modbus_read_input_float(sdm_slave_id, 0x0156, &sdm_energy);

  if (extended_update) {
    err = err || modbus_read_input_float(sdm_slave_id, 0x0000, &sdm_voltage) ||
          modbus_read_input_float(sdm_slave_id, 0x0046, &sdm_frequency) ||
          modbus_read_input_float(sdm_slave_id, 0x0048, &sdm_import_energy) ||
          modbus_read_input_float(sdm_slave_id, 0x004a, &sdm_export_energy) ||
          modbus_read_input_float(sdm_slave_id, 0x0058, &sdm_import_power) ||
          modbus_read_input_float(sdm_slave_id, 0x005c, &sdm_export_power) ;
  }
  if (err) {
    Serial.print("update_modbus_data sdm failed ");
    Serial.println(err);
    reterr = 1;
  } else {
    sdm_updated = millis();
  }
  
  err = modbus_read_holding_int(evse_slave_id, 1001, &evse_amp) ||
        modbus_read_holding_int(evse_slave_id, 1002, &evse_vehicle_state) ||
        modbus_read_holding_int(evse_slave_id, 1004, &evse_reg_charging) ||
        modbus_read_holding_int(evse_slave_id, 1006, &evse_state) ||
        modbus_read_holding_int(evse_slave_id, 1007, &evse_status) || 
        modbus_read_holding_int(evse_slave_id, 2005, &evse_config_reg);
  if (err) {
    Serial.print("update_modbus_data evse failed ");
    Serial.println(err);
    reterr = 2;
  } else {
    evse_updated = millis();
  }
  decodeEvseStatus(evse_status);
  evse_vehicle_present = (evse_vehicle_state >= 2);
  /*
    Serial.print(evse_relay_on);
    Serial.print(" ");
    Serial.print(evse_state);
    Serial.print(" ");meter
    Serial.print(evse_vehicle_state);
    Serial.print(" ");
    Serial.print(sdm_power);
    Serial.println();
  */
  evse_charging = (evse_relay_on || (evse_vehicle_state == 3) || (evse_vehicle_state == 4) || (sdm_power > 5.0));

  evse_charging_enabled = !(evse_config_reg & EVSE_CONFIG_DISABLE_CHARGING) && !(evse_reg_charging & 1);
  
  if (request_evse_stop_charging) {
    evse_stop_charging();
    evse_charging_enabled = false;
    request_evse_stop_charging = false;
  }
  if (request_evse_start_charging) {
    evse_start_charging();
    evse_charging_enabled = true;
    request_evse_start_charging = false;
  }
  if (!evse_charging && (phase_state == PHASE_UNKNOWN)) {
    // After boot we know that the relay is off, hence once the charging
    // stops, we know it is configured to one phase.
    Serial.println("Phase-state ONE");
    phase_state = PHASE_ONE;
  }
  if (!evse_charging) {
    evse_amp = 0;
  }
  evse_configure_set_current();


  return reterr;
}

unsigned long lastInitRetry = 0;
unsigned long lastModbusUpdate = 0;
unsigned long lastStateError = 0;
unsigned long modeTimeout = 0;
unsigned int last_state = 99;

unsigned long blink_last = 0;
char blink_pattern[10] = "1";
int blink_idx;

void blink_mode(const char *pattern) {
  strncpy(blink_pattern, pattern, 10);
}

void blink_manager() {
  unsigned long now = millis();
  if ((now - blink_last) < 200)  {
    return;
  }
  blink_idx++;
  if (blink_pattern[blink_idx] == 0) {
    blink_idx = 0;
  }
  if (blink_pattern[blink_idx] == '1') {
    digitalWrite(PUSH_BUTTON_LED, 1);
  } else {
    digitalWrite(PUSH_BUTTON_LED, 0);
  }
  blink_last = now;
}

bool wifi_manager_portal_active = true;

void loop() {
  unsigned long now = millis();
  switch (state) {
    case STATE_INIT:
      if ((now - lastInitRetry) > 5000) {
        lastInitRetry = now;
        if (sdmInit()) {
          state_error = "SDM Init failed";
          state = STATE_ERROR;
          lastStateError = now;
          break;
        }
        if (evseInit()) {
          state_error = "EVSE Init failed";
          state = STATE_ERROR;
          lastStateError = now;
          break;
        }
        state_error = "";
        updateModbusData();
      } else {
        break;
      }
      Serial.println("Ready?");
      request_evse_mode = -1;
      state = STATE_READY;
    // Intentionally no break to fall through to charging check.
    case STATE_READY:
      if (evse_charging) {
        state = STATE_CHARGING;
        chargeStart = now;
        chargeDuration = 0;
        chargeEnergyStart = sdm_energy;
        chargeEnergy = 0;
        break;
      }
      if (request_evse_mode != -1) {
        modeTimeout = now + 2 * 60000; // 2 minutes
        if (!evse_charging_enabled) {
          request_evse_start_charging = true;
        } else if (evse_mode == MODE_AUTO) {
          evse_mode = MODE_MANUAL;
          request_evse_set_current = CURRENT_LIMIT_ONE;
        } else if (evse_mode == MODE_MANUAL) {
          evse_mode = MODE_MANUAL_THREE;
          request_evse_set_current = CURRENT_LIMIT_THREE;
        } else {
          evse_mode = MODE_AUTO;
          // 1 phase has a higher limit anyways than 3-phase
          // request_evse_set_current = CURRENT_LIMIT_BOOT;
          request_phases = PHASE_ONE;
          // digitalWrite(THREE_PHASE_RELAY, THREE_PHASE_OFF);
          // phase_state = PHASE_ONE;
        }

      }
      if ((now > modeTimeout) && (evse_mode != MODE_AUTO)) {
        evse_mode = MODE_AUTO;
        // We are not charging, so this is safe to do.
        request_evse_set_current = CURRENT_LIMIT_BOOT;
        request_phases = PHASE_ONE;
      }
      request_evse_mode = -1;
      break;
    case STATE_CHARGING:
      if (request_evse_mode != -1) {
        evse_stop_charging();
      }
      request_evse_mode = -1;
      if (!evse_charging) {
        // sane default after charging.
        state = STATE_READY;
        phase_state = PHASE_ONE;
        evse_mode = MODE_AUTO;
        // Do not reset current, if we go from 3 to 1 phase, the limit will be smaller anyways.
        // request_evse_set_current = CURRENT_LIMIT_BOOT;
        break;
      }
      if (chargeEnergyStart == 0) {
        chargeEnergyStart = sdm_energy;
      }
      chargeDuration = now - chargeStart;
      chargeEnergy = sdm_energy - chargeEnergyStart;
      if (phase_state == PHASE_THREE) {
        chargeEnergy *= 3;
        // We detected we are in the charging state, we consume power, that
        // means the relay is hold on by the "back-current" from the charging
        // phase.  Turn this off that in case of power interruption, we want
        // to fall back to PHASE_ONE state. (see above)
        if (evse_charging && (sdm_power > 5.0)) {
          digitalWrite(THREE_PHASE_RELAY, THREE_PHASE_OFF);
          // not setting phase_state to ONE until after charging is done (see above)
        }
      }
      break;
    case STATE_ERROR:
      if ((now - lastStateError) > 30000) {
        state = STATE_INIT;
      }
      request_evse_mode = -1;
      break;
    default:
      // should never happen(tm)
      state = STATE_ERROR;
      lastStateError = now;
      break;
  }

  if (((evse_mode == MODE_MANUAL_THREE) && (phase_state != PHASE_THREE)) || (request_phases == PHASE_THREE)) {
    evse_toggle_three_phases(PHASE_THREE);
  } else if (((evse_mode == MODE_MANUAL) && (phase_state != PHASE_ONE))  || (request_phases == PHASE_ONE)) {
    evse_toggle_three_phases(PHASE_ONE);
  }
  request_phases = -1;
  char tmp[10], tmp2[10];
  unsigned long  s = (now % 15000) / 5000;
  int phase_multiplier = 1;
  if (phase_state == PHASE_THREE) {
    phase_multiplier = 3;
  }
  switch (state) {
    case STATE_INIT:
      update_display("Init");
      break;
    case STATE_READY:
      if (evse_vehicle_present) {
        sprintf(tmp, "Veh");
      } else {
        sprintf(tmp, "Rdy");
      }
      if (!evse_charging_enabled) {
        sprintf(tmp2, "%s DIS", tmp);
      } else if (s == 0) {
        switch (evse_mode) {
          case MODE_AUTO:
            sprintf(tmp2, "%s AUTO", tmp);
            blink_mode("1");
            break;
          case MODE_MANUAL:
            sprintf(tmp2, "%s 1-PH", tmp);
            blink_mode("11110000");
            break;
          case MODE_MANUAL_THREE:
            sprintf(tmp2, "%s 3-PH", tmp);
            blink_mode("101010000");
            break;
        }
      } else {
        if (phase_state == PHASE_ONE) {
          sprintf(tmp2, "%s %dA", tmp, evse_set_current);
        } else if (phase_state == PHASE_THREE) {
          sprintf(tmp2, "%s 3x%d", tmp, evse_set_current);
        } else {
          sprintf(tmp2, "%s ?P?", tmp);
        }
      }
      update_display(tmp2);
      break;
    case STATE_CHARGING:


      switch (evse_mode) {
        case MODE_AUTO:
          sprintf(tmp2, "Chg %.0fkW", phase_multiplier * sdm_power / 1000);

          blink_mode("1");
          break;
        case MODE_MANUAL:
          sprintf(tmp2, "1ph %.0fkW", phase_multiplier * sdm_power / 1000);
          blink_mode("11110000");
          break;
        case MODE_MANUAL_THREE:
          sprintf(tmp2, "3ph %.0fkW", phase_multiplier * sdm_power / 1000);
          blink_mode("101010000");
          break;
      }

      if (s == 1) {
        sprintf(tmp2, "%.2fkWH", phase_multiplier * chargeEnergy);
      } else if (s == 2) {
        unsigned long duration = (now - chargeStart) / 1000;
        if (duration < 300) {
          sprintf(tmp2, "%ld sec", duration);
        } else if (duration < 3600) {
          sprintf(tmp2, "%ldm%lds", duration / 60, duration % 60);
        } else {
          unsigned long m = (duration % 3600) / 60;
          sprintf(tmp2, "%ldh%ldm", duration / 3600, m);
        }
      }

      update_display(tmp2);
      break;
    case STATE_ERROR:
      update_display(state_error.c_str());
      blink_mode("10"); // fast blink
      break;

  }

  if ((now - lastModbusUpdate) > 1000) {
    //Serial.print(".");
    if ((state != STATE_INIT) && (state != STATE_ERROR)) {
      updateModbusData();

    }
    updateModbus(); // update external (server modbus)
    lastModbusUpdate = now;
  }

  blink_manager();
  server.handleClient();
  mb.task();
  /*
  if (WiFi.status() == WL_CONNECTED) {
    if (wifi_manager_portal_active) {
      Serial.println("Wifi Connected, disable portal");
      wifiManager.stopWebPortal();
      start_webserver();
      wifi_manager_portal_active = false;
    }
  } else {
    // TODO add a timeout before starting the web portal.
    if (!wifi_manager_portal_active) {
      Serial.println("Wifi Disconnected, enable portal");
      wifiManager.startWebPortal();
      wifi_manager_portal_active = true;
    }
  }
  wifiManager.process();
  */
  ArduinoOTA.handle();
  MDNS.update();
  wifiMulti.run();
  yield();
}
