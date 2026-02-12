/*
  Bridge UART via Wireless or USB

  A simple bridge designed for the XIAO ESP32C6, connecting WiFi or USB to UART.
  All configuration is done via the USB port and saved to the NVM.
  Except for the parts decoding PUSR's VCOM UART updates and the IOCTL_SERIAL_LSRMST_INSERT protocol, it is extremely straightforward.

  SPDX-License-Identifier: MIT
  SPDX-FileCopyrightText: (C) 2026 mukyokyo
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <lwip/sockets.h>
#include <Preferences.h>
#include <soc/usb_serial_jtag_struct.h>
#include "esp_wifi.h"
#include "us.h"

#include "esp_system.h"
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "soc/rtc_cntl_reg.h"
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
#include "soc/lp_aon_reg.h"
#endif

#include "USBCDC.h"

// Enable when using a USB serial bridge
#define USBBRIDGE

#ifdef USBBRIDGE
#include "USB.h"
#if !ARDUINO_USB_CDC_ON_BOOT
USBCDC USBSerial;
#endif
#if !ARDUINO_USB_CDC_ON_BOOT
HWCDC HWCDCSerial;
#endif
#endif

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define CONNECT_TIMEOUT_MS 10000
#define _MAX_BAUDRATE 16000000
#define _MIN_BAUDRATE 110

#ifndef min
#define min(a, b) \
  ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#endif

#ifndef max
#define max(a, b) \
  ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#endif

template< typename typ, std::size_t size > size_t GetNumOfElems(const typ (&array)[size]) {
  return size;
}

// Network settings related
typedef struct {
  char hostname[64];    // Host name
  uint8_t mode;         // 0:OFF 1:AP 2:STA
  char ssid[64];        // SSID
  char psk[64];         // Password
  IPAddress ip;         // IP address
  IPAddress mask;       // Net mask
  uint16_t port;        // Port for client connection
  uint8_t encprotocol;  // 0:OFF 1:PUSR 2:LsrMstInsert
  uint32_t baudrate;    // default baudrate
  char serconfig[10];   // default serial config
} __attribute__((packed)) TNetInfo;

const TNetInfo default_netinfo = {
  "XIAO_WiFi2Serial",           // Host name
  1,                            // WiFi operating mode
  "Xiao_wifi2Serial",           // SSID
  "12345678",                   // PSK
  IPAddress(10, 0, 0, 1),       // Pico's IP address
  IPAddress(255, 255, 255, 0),  // Pico's IP mask
  23,                           // Client connection port

  0,       // Method for including baudrate and configuration in serial data from a PC
  115200,  // boottime baudrate
  "8N1"    // boottime config
};

const char *databits_s = "5678";
const char *parity_s = "NOEMS";
const char *stopbit_s = "12";

TNetInfo info;
Preferences preferences;
WiFiServer *server = NULL;
WiFiClient client;
uint32_t current_baud;
String current_serconfig;

uint32_t cdc_baud, cdc_prevbaud;
String cdc_config, cdc_prevconfig;

//---------------------
// etc
//---------------------
// Display current settings and current status in the terminal
void print_stat(void) {
  const char *mode_s[] = { "Off", "AP", "STA" };
  const char *serprot_s[] = { "Off", "PUSR", "LsrMstIns" };
  Serial.printf("Status:\r\n Mode is %s\r\n", mode_s[info.mode]);
  switch (info.mode) {
    case 0:
      Serial.print("WiFi off\r\n");
      break;
    case 1:  // AP
      Serial.printf(" Hostname is %s\r\n", info.hostname); 
      Serial.printf(" AP is '%s' with '%s'\r\n", info.ssid, pass(info.psk).c_str());
      Serial.printf(" My IP is %s/%s\r\n", WiFi.softAPIP().toString().c_str(), WiFi.softAPSubnetMask().toString().c_str());
      Serial.printf(" TCP server started at %s:%d\r\n", WiFi.softAPIP().toString().c_str(), info.port);
      break;
    case 2:  // STA
      Serial.printf(" Hostname is %s\r\n", info.hostname); 
      Serial.printf(" STA is '%s' with '%s'\r\n", info.ssid, pass(info.psk).c_str());
      Serial.printf(" My IP is %s/%s\r\n", WiFi.localIP().toString().c_str(), WiFi.subnetMask().toString().c_str());
      Serial.printf(" TCP server started at %s:%d\r\n", WiFi.localIP().toString().c_str(), info.port);
      Serial.printf(" RSSI is %ddBm\r\n", WiFi.RSSI());
      break;
  }
  Serial.printf(" UART protocol is %s\r\n", serprot_s[info.encprotocol]);
  Serial.printf(" UART is %lubps %s\r\n", current_baud, current_serconfig.c_str());
}

uint32_t conv_str2serconfig(const char *s, char *d = NULL) {
  struct {
    const char *str;
    uint32_t param;
  } const cparam[] {
    { "5N1", SERIAL_5N1 }, { "6N1", SERIAL_6N1 }, { "7N1", SERIAL_7N1 }, { "8N1", SERIAL_8N1 },
    { "5N2", SERIAL_5N2 }, { "6N2", SERIAL_6N2 }, { "7N2", SERIAL_7N2 }, { "8N2", SERIAL_8N2 },
    { "5E1", SERIAL_5E1 }, { "6E1", SERIAL_6E1 }, { "7E1", SERIAL_7E1 }, { "8E1", SERIAL_8E1 },
    { "5E2", SERIAL_5E2 }, { "6E2", SERIAL_6E2 }, { "7E2", SERIAL_7E2 }, { "8E2", SERIAL_8E2 },
    { "5O1", SERIAL_5O1 }, { "6O1", SERIAL_6O1 }, { "7O1", SERIAL_7O1 }, { "8O1", SERIAL_8O1 },
    { "5O2", SERIAL_5O2 }, { "6O2", SERIAL_6O2 }, { "7O2", SERIAL_7O2 }, { "8O2", SERIAL_8O2 },
#if 0
    { "5M1", SERIAL_5M1 }, { "6M1", SERIAL_6M1 }, { "7M1", SERIAL_7M1 }, { "8M1", SERIAL_8M1 },
    { "5M2", SERIAL_5M2 }, { "6M2", SERIAL_6M2 }, { "7M2", SERIAL_7M2 }, { "8M2", SERIAL_8M2 },
    { "5S1", SERIAL_5S1 }, { "6S1", SERIAL_6S1 }, { "7S1", SERIAL_7S1 }, { "8S1", SERIAL_8S1 },
    { "5S2", SERIAL_5S2 }, { "6S2", SERIAL_6S2 }, { "7S2", SERIAL_7S2 }, { "8S2", SERIAL_8S2 }
#endif
  };
  for (int i = 0; i < GetNumOfElems(cparam); i++) {
    if (strcasecmp(s, cparam[i].str) == 0) {
      if (d != NULL) strcpy(d, cparam[i].str);
      return cparam[i].param;
    }
  }
  if (d != NULL) strcpy(d, "8N1");
  return SERIAL_8N1;
}

// Changing the ESP32's WiFi output (not sure if it's even necessary)
void power(void) {
  int8_t power_dbm = 15;
  int8_t power_param = (int8_t)(power_dbm / 0.25);
  esp_err_t err = esp_wifi_set_max_tx_power(power_param);
  if (err == ESP_OK) {
    Serial.print("Successfully set WiFi TX Power to: ");
    Serial.print(power_dbm);
    Serial.print(" dBm\r\n");
  } else {
    Serial.print("Failed to set WiFi TX Power.\r\n");
  }
}

// Reboot
void reboot(void) {
  if (client) {
    if (client.connected()) client.stop();
  }
  WiFi.disconnect();
  Serial.end();
  delay(200);
  ESP.restart();  // Software reset
  while (1)
    ;
}

void bootloader(void) {
  if (client) {
    if (client.connected()) client.stop();
  }
  WiFi.disconnect();
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
  esp_restart();
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
  USB_SERIAL_JTAG.chip_rst.usb_uart_chip_rst_dis = 0;
  USB_SERIAL_JTAG.config_update.config_update = 0;
  //  Serial.end();
  //  delay(200);
  REG_WRITE(LP_AON_SYS_CFG_REG, LP_AON_FORCE_DOWNLOAD_BOOT);
  esp_restart();
#endif
}

//----------------------------------------------------------------
// Decoding from packets including baudrate and other parameters
//----------------------------------------------------------------
#ifdef USBBRIDGE
//ARDUINO_USB_CDC_EVENTS
static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  String s = "   ";
  if (event_base == ARDUINO_USB_CDC_EVENTS) {
    arduino_usb_cdc_event_data_t *data = (arduino_usb_cdc_event_data_t *)event_data;
    switch (event_id) {
      case ARDUINO_USB_CDC_LINE_CODING_EVENT:
        cdc_baud = max(min(data->line_coding.bit_rate, _MAX_BAUDRATE), _MIN_BAUDRATE);
        s[0] = databits_s[max(min(data->line_coding.data_bits, 8), 5) - 5];
        s[1] = parity_s[max(min(data->line_coding.parity, 4), 0)];
        s[2] = stopbit_s[max(min(data->line_coding.stop_bits, 1), 0)];
        cdc_config = s;
        break;
      default:
        break;
    }
  }
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
  if (event_base == ARDUINO_HW_CDC_EVENTS) {
    switch (event_id) {
      case ARDUINO_HW_CDC_ANY_EVENT:
        Serial1.println("CDC EVENT:: ARDUINO_HW_CDC_ANY_EVENT");
        break;
      case ARDUINO_HW_CDC_MAX_EVENT:
        Serial1.println("CDC EVENT:: ARDUINO_HW_CDC_MAX_EVENT");
        break;

      case ARDUINO_HW_CDC_CONNECTED_EVENT:
        Serial1.println("CDC EVENT:: ARDUINO_HW_CDC_CONNECTED_EVENT");
        break;
      case ARDUINO_HW_CDC_BUS_RESET_EVENT:
        Serial1.println("CDC EVENT:: ARDUINO_HW_CDC_BUS_RESET_EVENT");
        break;
      case ARDUINO_HW_CDC_RX_EVENT:
        Serial1.println("\nCDC EVENT:: ARDUINO_HW_CDC_RX_EVENT");
        // sends all bytes read from USB Hardware Serial to UART0
        //        while (HWCDCSerial.available()) {
        //          Serial.write(HWCDCSerial.read());
        //        }
        break;
      case ARDUINO_HW_CDC_TX_EVENT:
        Serial1.println("CDC EVENT:: ARDUINO_HW_CDC_TX_EVENT");
        break;

      default:
        Serial1.println("CDC EVENT:: default");
        break;
    }
  } else {
    Serial1.printf("EVENT:: ???(%d)", event_base);
  }
#endif
}
#endif

// Processing corresponding to PUSR's proprietary serial port parameter change packets
// The packet is 8 bytes: 0x55 0xAA 0x55 P0 P1 P2 P3 SUM
//  P0~P2: baudrate
//  P3: data width, parity, stop bit
//  SUM: Sum of P0 to P3
bool PUSR_portconfig_check(uint8_t *p) {
  uint32_t baud = 0;
  uint32_t config = 0;
  static uint32_t prevbaud = 0;
  static uint32_t prevconf = 0;

  String s = "   ";

  // Verify packet match
  if (p[0] == 0x55) {
    if (p[1] == 0xaa) {
      if (p[2] == 0x55) {
        if ((uint8_t)(p[3] + p[4] + p[5] + p[6]) == p[7]) {
          baud = max(min((p[3] << 16) | (p[4] << 8) | p[5], _MAX_BAUDRATE), _MIN_BAUDRATE);
          union {
            struct {
              uint8_t databits : 2;    // 0:5bit 1:6bit 2:7bit 3:8bit
              uint8_t stopbits : 1;    // 0:1bit 1:2bit
              uint8_t parityen : 1;    // 0:di 1:en
              uint8_t paritytype : 2;  // 0:ODD 1:EVEN 2:MASK 3:SPACE
            };
            uint8_t byte;
          } bitset;
          bitset.byte = p[6];
          s[0] = databits_s[bitset.databits];
          s[1] = parity_s[(bitset.parityen == 1) ? bitset.paritytype + 1 : 0];
          s[2] = stopbit_s[bitset.stopbits];
          char tmp[10];
          config = conv_str2serconfig(s.c_str(), tmp);
          // If change requests occur frequently, ignore them if no changes are needed from the current state.
          if ((prevconf != config) || (prevbaud != baud)) {
            Serial1.flush();
            Serial1.begin(baud, config);
            current_baud = baud;
            current_serconfig = s;
            prevconf = config;
            prevbaud = baud;
            Serial.printf("Update UART to %ubps %s\r\n", Serial1.baudRate(), tmp);
          }
          return true;
        }
      }
    }
  }
  return false;
}

// When the host enables LSRMST_INSERT via IOCTL_SERIAL_LSRMST_INSERT, it decodes the line status from the data stream.
void LSRMSTINS_portconfig_check(uint8_t *pBuf, int len) {

#define SERIAL_LSRMST_ESCAPE ((uint8_t)0x00)
#define SERIAL_LSRMST_LSR_DATA ((uint8_t)0x01)
#define SERIAL_LSRMST_LSR_NODATA ((uint8_t)0x02)
#define SERIAL_LSRMST_MST ((uint8_t)0x03)
#define C0CE_INSERT_RBR ((uint8_t)16)
#define C0CE_INSERT_RLC ((uint8_t)17)

  uint32_t baud = 0;
  uint32_t config = 0;
  static uint32_t prevbaud = 0;
  static uint32_t prevconf = 0;
  static String s = "   ";

  const char escapeChar = 'a';
  uint8_t baud_byte[sizeof(uint32_t)];
  static int state = 0;
  static uint8_t code = 0;
  static int subState = 0;

  uint32_t _baudrate;
  int _byteSize;
  int _parity;
  int _stopBits;

  uint8_t mybuf[len];
  int mylen = 0;

  for (; len; len--) {
    uint8_t ch = *pBuf++;

    switch (state) {
      case 0:
        break;
      case 1:
        code = ch;
        state++;
      case 2:
        switch (code) {
          case SERIAL_LSRMST_ESCAPE:
            mybuf[mylen++] = escapeChar;
            //          Serial1.write(escapeChar);
            state = subState = 0;
            break;
          case SERIAL_LSRMST_LSR_DATA:
            if (subState == 0) subState++;
            else if (subState == 1) subState++;
            else if (subState == 2) state = subState = 0;
            else state = subState = 0;
            break;
          case SERIAL_LSRMST_LSR_NODATA:
            if (subState == 0) subState++;
            else if (subState == 1) state = subState = 0;
            else state = subState = 0;
            break;
          case SERIAL_LSRMST_MST:
            if (subState == 0) subState++;
            else if (subState == 1) state = subState = 0;
            else state = subState = 0;
            break;
          case C0CE_INSERT_RBR:
            if (subState == 0) {
              subState++;
            } else if (subState >= 1 && subState < (int)(sizeof(uint32_t) + 1)) {
              baud_byte[subState - 1] = ch;
              if (subState < (int)sizeof(uint32_t)) {
                subState++;
              } else {
                baud = max(min((long)*(uint32_t *)baud_byte, _MAX_BAUDRATE), _MIN_BAUDRATE);
                Serial.printf("BaudRate=%lu\r\n", baud);
                if (baud != prevbaud) {
                  Serial1.write(mybuf, mylen);
                  mylen = 0;

                  Serial1.flush();
                  Serial1.begin(baud, prevconf);
                  Serial.printf("Update UART to *%ubps %s\r\n", Serial1.baudRate(), s.c_str());
                  current_baud = baud;
                  prevbaud = baud;
                }
                state = subState = 0;
              }
            } else
              state = subState = 0;
            break;
          case C0CE_INSERT_RLC:
            if (subState == 0) {
              subState++;
            } else if (subState == 1) {
              _byteSize = ch & 0xFF;
              subState++;
            } else if (subState == 2) {
              _parity = ch & 0xFF;
              subState++;
            } else if (subState == 3) {
              _stopBits = ch & 0xFF;
              Serial.printf("ByteSize=%d Parity=%d StopBits=%d\r\n", _byteSize, _parity, _stopBits);
              s[0] = databits_s[max(min(_byteSize - 5, 3), 0)];
              s[1] = parity_s[max(min(_parity, 4), 0)];
              s[2] = stopbit_s[max(min(_stopBits, 2), 0)];
              char tmp[10];
              config = conv_str2serconfig(s.c_str(), tmp);
              if (config != prevconf) {
                Serial1.write(mybuf, mylen);
                mylen = 0;

                Serial1.flush();
                Serial1.begin(prevbaud, config);
                Serial.printf("Update UART to %ubps *%s\r\n", Serial1.baudRate(), tmp);
                current_serconfig = s;
                prevconf = config;
              }
              state = subState = 0;
            } else
              state = subState = 0;
            break;
          default:
            state = subState = 0;
            break;
        }
        continue;

      default:
        state = subState = 0;
        ;
    }
    if (ch == escapeChar) {
      state = 1;
      continue;
    }
    //    Serial1.write(ch);
    mybuf[mylen++] = ch;
  }
  Serial1.write(mybuf, mylen);
}

//----------------------------------------------------------------
// setup
//----------------------------------------------------------------
// Configure network settings from the terminal
void CommandProc(bool waitforexit) {
  uint32_t blink_t = millis() + 500;
  static String s;
  static char b[64];
  uint8_t mode = 0;
  static char bu[5][64];
  uint16_t port = 0;
  IPAddress ip;
  uint8_t protocol = 0;
  uint32_t baudrate = 115200;
  static char bc[10];

  while (Serial.available() || waitforexit) {
    if (waitforexit) {
      if (millis() > blink_t) {
        blink_t = millis() + 500;
        digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
      }
    }

    char c = Serial.read();
    switch (c) {
      case '\33':
        Serial.printf("\x1b[2J");
        break;
      // Exit command mode
      case '#':
        us_rx_flush();
        waitforexit = false;
        digitalWrite(LED_BUILTIN, 1);
        break;
      // Reboot
      case '!':
        bootloader();
        break;
      // Status
      case 'i':
        us_rx_flush();
        print_stat();
        break;

      // Save default values to NVM
      case 'f':
        Serial.print("Reset NVM settings\r\n");
        if (are_you_sure()) {
          preferences.begin("wifi2serial", false);
          preferences.putBytes("netinfo", &default_netinfo, sizeof(TNetInfo));
          preferences.getBytes("netinfo", &info, sizeof(TNetInfo));
          preferences.end();
          reboot();
        } else
          Serial.printf("canceled\r\n");
        break;
      // Settings
      case 's':
        Serial.printf("Configure network settings\r\n");
        memset(bu, 0, sizeof(bu));
        memset(b, 0, sizeof(b));
        memset(bc, 0, sizeof(b));
        Serial.print("Select WiFi mode (0:Off 1:AP 2:STA)=");
        if (us_gets(b, sizeof(b)) > 0 && strlen(b) > 0) {
          s = b;
          mode = s.toInt();
          if (2 >= mode && mode >= 0) {
            if (mode != 0) {
              Serial.print("hostname=");
              us_gets(bu[0], sizeof(bu[0]));
              Serial.print("ssid=");
              us_gets(bu[1], sizeof(bu[1]));
              Serial.print("psk=");
              us_gets(bu[2], sizeof(bu[2]));
              Serial.printf("ip%s=", (mode == 2) ? "(If blank, use DHCP)" : "");
              us_gets(bu[3], sizeof(bu[3]));
              Serial.printf("mask%s=", (mode == 2) ? "(If blank, use DHCP)" : "");
              us_gets(bu[4], sizeof(bu[4]));
              Serial.print("port(0..65535)=");
              if (us_gets(b, sizeof(b)) > 0) {
                s = b;
                port = max(min(s.toInt(), 65535), 0);
              }
              Serial.print("serial protocol (0:Off, 1:PUSR, 2:LsrMst)=");
              if (us_gets(b, sizeof(b)) > 0) {
                s = b;
                protocol = max(min(s.toInt(), 2), 0);
              } else
                protocol = 0;
            }

            Serial.print("serial baudrate(" TOSTRING(_MIN_BAUDRATE) "..." TOSTRING(_MAX_BAUDRATE) ")=");
            if (us_gets(b, 7) > 0) {
              s = b;
              baudrate = max(min(s.toInt(), _MAX_BAUDRATE), _MIN_BAUDRATE);
            } else
              baudrate = 115200;
            Serial.print("serial config(ex.8N1)=");
            us_gets(bc, 3);
            conv_str2serconfig(bc, bc);

            Serial.print("Input values\r\n");
            Serial.printf(" hostname:%s\r\n", bu[0]);
            Serial.printf(" mode:%d\r\n", mode);
            Serial.printf(" ssid:%s\r\n", bu[1]);
            Serial.printf(" psk :%s\r\n", pass(bu[2]).c_str());
            if (strlen(bu[3]) == 0) strcpy(bu[3], "0.0.0.0");
            ip.fromString(bu[3]);
            Serial.printf(" ip  :%s\r\n", ip.toString().c_str());
            if (strlen(bu[4]) == 0) strcpy(bu[4], "0.0.0.0");
            ip.fromString(bu[4]);
            Serial.printf(" mask:%s\r\n", ip.toString().c_str());
            Serial.printf(" port:%d\r\n", port);
            Serial.printf(" serial protocol:%d\r\n", protocol);
            Serial.printf(" serial baudrate:%lu\r\n", baudrate);
            Serial.printf(" serial config:%s\r\n", bc);
            if (are_you_sure()) {
              info.mode = mode;
              strncpy(info.hostname, bu[0], sizeof(info.hostname) - 1);
              strncpy(info.ssid, bu[1], sizeof(info.ssid) - 1);
              strncpy(info.psk, bu[2], sizeof(info.psk) - 1);
              info.ip.fromString(bu[3]);
              info.mask.fromString(bu[4]);
              info.port = port;
              info.encprotocol = protocol;
              info.baudrate = baudrate;
              strncpy(info.serconfig, bc, sizeof(info.serconfig) - 1);
              preferences.begin("wifi2serial", false);
              preferences.putBytes("netinfo", &info, sizeof(TNetInfo));
              preferences.end();
              reboot();
            } else
              Serial.printf("canceled\r\n");
          } else
            Serial.printf("canceled\r\n");
        } else
          Serial.printf("canceled\r\n");
        break;
    }
  }
}

void setup() {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
  USB_SERIAL_JTAG.chip_rst.usb_uart_chip_rst_dis = 1;
  USB_SERIAL_JTAG.config_update.config_update = 1;
#endif
  Serial.setRxBufferSize(1024);
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(500);

  preferences.begin("wifi2serial", false);
  int n = preferences.getBytes("netinfo", &info, sizeof(TNetInfo));
  if (n != sizeof(TNetInfo)) {
    preferences.putBytes("netinfo", &default_netinfo, sizeof(TNetInfo));
    preferences.getBytes("netinfo", &info, sizeof(TNetInfo));
  }
  preferences.end();

  String s;
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 0);
  pinMode(BOOT_PIN, INPUT_PULLUP);

  Serial1.setRxBufferSize(4096);
  Serial1.begin(info.baudrate, conv_str2serconfig(info.serconfig), D7, D6);
  cdc_baud = cdc_prevbaud = current_baud = info.baudrate;
  cdc_config = cdc_prevconfig = current_serconfig = info.serconfig;

  switch (info.mode) {
    case 0:  // USB UART mode
#ifdef USBBRIDGE
      HWCDCSerial.onEvent(usbEventCallback);
      HWCDCSerial.begin();
#endif
      break;
    case 1:  // AP mode
      WiFi.setHostname(info.hostname);
      WiFi.mode(WIFI_AP);
      power();
      WiFi.softAPConfig(info.ip, info.ip, info.mask);
      WiFi.softAP(info.ssid, info.psk, 1, 0, 1);
      Serial.printf("My AP is '%s' with '%s'\r\n", info.ssid, info.psk);
      Serial.printf("My IP is %s:%d\r\n", WiFi.softAPIP().toString(), info.port);

      if (!MDNS.begin(info.hostname)) {
        Serial.print("Error starting mDNS service!\r\n");
        while(1);
      }
      break;
    case 2:  // STA mode
      WiFi.setHostname(info.hostname);
      Serial.printf("Connect to '%s' with '%s'\r\n", info.ssid, pass(info.psk).c_str());
      WiFi.mode(WIFI_STA);
      power();
      WiFi.begin(info.ssid, info.psk);
      uint32_t t = millis() + CONNECT_TIMEOUT_MS;
      uint8_t cled = 0;
      // Waiting to connect to the WiFi router
      while (WiFi.status() != WL_CONNECTED) {
        CommandProc(false);
        if (millis() > t) {
          WiFi.disconnect();
          Serial.printf("Retry\r\n");
          WiFi.reconnect();
          t = millis() + CONNECT_TIMEOUT_MS;
        }
        delay(100);
        digitalWrite(LED_BUILTIN, cled);
        cled ^= 1;
        delay(100);
      }
      Serial.printf("Connected to %s\r\n", info.ssid);

      if (!MDNS.begin(info.hostname)) {
        Serial.print("Error starting mDNS service!\r\n");
        while(1);
      }

      break;
  }

  if (info.mode != 0) {
    server = new WiFiServer(info.port);
    server->begin();
    server->setNoDelay(true);
    Serial.printf("Listening at %s:%d\r\n", WiFi.localIP().toString().c_str(), info.port);
  }
}

//----------------------------------------------------------------
// loop
//----------------------------------------------------------------
void loop() {
  static bool prev_connected = false;
  bool txrx_led = false;
  static uint32_t blink_t = 0;
  static uint32_t idle_t = 0;
  static uint8_t buf[2048];
  static IPAddress cliIP;
  static uint16_t cliPort;
  int l, ll;

  switch (info.mode) {
    case 0:
      // When WiFi is off, pressing the boot switch enables setting changes
      if (digitalRead(BOOT_PIN) == LOW) {
        Serial.print("\r\nEnter config mode\r\n");
        CommandProc(true);
        Serial.print("\r\nExit config mode\r\n");
      }

      // Transfer data received from UART directly to USB
      while ((l = Serial1.available()) > 0) {
        while ((ll = Serial1.readBytes(buf, min(sizeof(buf), l))) > 0) {
          l -= ll;
          Serial.write((uint8_t *)buf, ll);
          Serial.flush();
          txrx_led = true;
        }
      }
      // Transfer data received from USB directly to UART
      while ((l = Serial.available()) > 0) {
        while ((ll = Serial.readBytes(buf, min(sizeof(buf), l))) > 0) {
          l -= ll;
          Serial1.write((uint8_t *)buf, ll);
          txrx_led = true;
        }
        Serial1.flush();
      }

      // Detection of baudrate or parameter changes
      {
        uint32_t b = cdc_baud;
        String c = cdc_config;
        if (b != cdc_prevbaud || c != cdc_prevconfig) {
          Serial1.flush();
          Serial1.begin(b, conv_str2serconfig(c.c_str()));
          cdc_prevbaud = b;
          cdc_prevconfig = c;
        }
      }
      if (txrx_led) {
        blink_t = millis() + 10;
        digitalWrite(LED_BUILTIN, 0);
        txrx_led = false;
      }
      if (millis() > blink_t) digitalWrite(LED_BUILTIN, 1);
      break;
    case 1:
    case 2:
        // Confirming the connection from the client
      if (server->hasClient()) {
        if (!client || !client.connected()) {
          if (client) client.stop();
          client = server->accept();
          client.setNoDelay(true);

          int keepAlive = 1;
          int keepIdle = 20;
          int keepInterval = 5;
          int keepCount = 3;
          client.setSocketOption(client.fd(), TCP_KEEPALIVE, (void *)&keepAlive, sizeof(keepAlive));
          client.setSocketOption(client.fd(), TCP_KEEPIDLE, (void *)&keepIdle, sizeof(keepIdle));
          client.setSocketOption(client.fd(), TCP_KEEPINTVL, (void *)&keepInterval, sizeof(keepInterval));
          client.setSocketOption(client.fd(), TCP_KEEPCNT, (void *)&keepCount, sizeof(keepCount));

          Serial.printf("New client(%s:%d) connected\r\n", client.remoteIP().toString().c_str(), client.remotePort());
          prev_connected = true;
          cliIP = client.remoteIP();
          cliPort = client.remotePort();
        }
      }
      if (client) {
        if (client.connected()) {
          // Transfers data received from the socket directly to the UART
          while ((l = client.available()) > 0) {
            if ((ll = client.readBytes(buf, min(sizeof(buf), l))) > 0) {
              l -= ll;
              switch (info.encprotocol) {
                case 0:
                  Serial1.write(buf, ll);
                  txrx_led = true;
                  break;
                case 1:
                  for (int j = 0; j < ll;) {
                    if (j + 8 <= ll) {
                      // Upon detecting a UART parameter change packet, process it appropriately and immediately discard the packet.
                      // Be careful, as it will be discarded if it matches even if that was not your intention.
                      if (PUSR_portconfig_check(&buf[j])) {
                        j += 8;
                        continue;
                      }
                    }
                    Serial1.write(buf[j++]);
                    txrx_led = true;
                  }
                  Serial1.flush();
                  break;
                case 2:
                  LSRMSTINS_portconfig_check(buf, ll);
                  Serial1.flush();
                  txrx_led = true;
                  break;
              }
            }
          }
          // Transfer data received from the UART directly to the socket
          while ((l = Serial1.available()) > 0) {
            while ((ll = Serial1.readBytes(buf, min(sizeof(buf), l))) > 0) {
              l -= ll;
              client.write((uint8_t *)buf, ll);
              txrx_led = true;
            }
          }
          if (txrx_led) {
            blink_t = millis() + 10;
            digitalWrite(LED_BUILTIN, 0);
            txrx_led = false;
          }
          if (millis() > blink_t) digitalWrite(LED_BUILTIN, 1);
        }
      } else {
        if (prev_connected) {
          prev_connected = false;
          Serial.printf("Client(%s:%d) disconnected\r\n", cliIP.toString().c_str(), cliPort);
          if (client) client.stop();
        }
        uint32_t t = millis();
        if (t > idle_t) {
          idle_t = t + (digitalRead(LED_BUILTIN) ? 30 : 2970);
          digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
        }
      }
      CommandProc(false);
      break;
  }
}