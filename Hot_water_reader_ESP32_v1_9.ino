/*
   ================================================================================
   ESP32 Hot Water Serial Monitor - Home Assistant & Web Interface Edition
   ================================================================================
   Board: ESP32-C3 Super Mini  (Arduino board: "ESP32C3 Dev Module")
   Tools menu: USB CDC On Boot = Enabled (for USB serial debug output)

   WHAT IT DOES:
     Captures raw 300-baud serial streams from a hot water system, displays live
     byte activity on a web dashboard, parses Byte 1 (Status & Fault Mode), Byte 2 (Tank Temp),
     Byte 3 (Evaporator Temp), and Byte 4 (Compressor Temp) in °C, and streams raw
     hex and telemetry (including Fault Status) to Home Assistant over MQTT using
     automatic Discovery with Last Will & Testament (LWT) and IP Address diagnostic reporting.

   ESP32 PORT NOTES (v1.9, ported from the ESP8266 v1.7 sketch):
     - SoftwareSerial replaced by hardware UART1 (Serial1) - no extra library needed.
     - RX = GPIO 20 (pin labelled "RX" / "20" on the Super Mini)
     - TX = GPIO 21 (pin labelled "TX" / "21") - unused, leave unconnected.
     - v1.9: WiFi TX power reduced to 8.5 dBm (Super Mini antenna fix) and the
       onboard LED shows WiFi state (ON = not connected / setup AP, OFF = online).

   WIRING (Resistor Divider):
     - Hot Water Signal -> 1.8k Ohm Resistor -> GPIO 20 (RX Pin)
     - GPIO 20 -> 3.3k Ohm Resistor -> Shared GND
     - Hot Water 5V Rail -> Board 5V Pin
     - Hot Water GND -> Board GND
   ================================================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>

// --------------------------------------------------------------------------------
// HARDWARE & SERIAL CONFIGURATION
// --------------------------------------------------------------------------------
#define RX_PIN 20                      // GPIO 20 (labelled RX on the ESP32-C3 Super Mini)
#define TX_PIN 21                      // GPIO 21 (labelled TX - unused, leave unconnected)
#define LED_PIN 8                      // Onboard blue LED on the Super Mini (active LOW)

const char* CONFIG_FILE = "/config.json";

// 250ms silence timeout: required for slow 300 baud transmission rates
const unsigned long PACKET_TIMEOUT_MS = 250;  

// --------------------------------------------------------------------------------
// SYSTEM & CORE NETWORKING OBJECTS
// --------------------------------------------------------------------------------
WebServer server(80);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Hardware UART1 for the hot water signal (Serial stays free for USB debug)
HardwareSerial& hwSerial = Serial1;

// --------------------------------------------------------------------------------
// PERSISTENT CONFIGURATION VARIABLES
// --------------------------------------------------------------------------------
char mqtt_server[64] = "";
int  mqtt_port       = 1883;
char mqtt_user[32]   = "";
char mqtt_pass[32]   = "";
char mqtt_topic[64]  = "homeassistant/sensor/hotwater";

int serial_baud      = 300;   // Default baud rate set to 300 bps

// --------------------------------------------------------------------------------
// RUNTIME BUFFER & MEASUREMENT VARIABLES
// --------------------------------------------------------------------------------
uint8_t rxBuffer[256];
size_t rxBufferPos = 0;
unsigned long lastRxByteMs = 0;

String lastCapturedPayload = "None";
int lastCapturedByte1      = 0;
int lastCapturedByte2      = 0;
int lastCapturedByte3      = 0;
int lastCapturedByte4      = 0;
String lastCapturedStatus  = "Idle";
bool lastCapturedFault     = false;

// Startup frame protection flag
bool isFirstPacket         = true;

String lastSentPayload     = "";
unsigned long packetCount  = 0;
unsigned long lastMQTTAttempt = 0;

// Cached last-known-good temperature readings, used to fill in MQTT
// payloads between real readings. Use -1 to mean "no valid reading cached".
// Moved to global scope (was static-local in publishMQTTData) so
// reconnectMQTT() can clear them on every (re)connect.
int lastValidByte2 = -1;
int lastValidByte3 = -1;
int lastValidByte4 = -1;

// --------------------------------------------------------------------------------
// HELPER FUNCTIONS
// --------------------------------------------------------------------------------
String getStatusText(int b1) {
  switch (b1) {
    case 0x00: return "Off / Standby";
    case 0x03: 
    case 0x20: return "Initializing / Booting";
    case 0x0A: return "Idle (Up to Temp)";
    case 0x16: return "Startup / Transition";
    case 0x17: return "Heating";
    case 0xE0:
    case 0xEE:
    case 0xFF: return "Fault Detected";
    default:   
      if (b1 >= 0xE1 && b1 <= 0xEF) return "Fault (E" + String(b1 & 0x0F) + ")";
      return "Unknown (0x" + String(b1, HEX) + ")";
  }
}

// (Re)starts the hot water UART at the configured baud rate
void beginHotWaterSerial() {
  hwSerial.begin(serial_baud, SERIAL_8N1, RX_PIN, TX_PIN);
  // Hand each byte to the sketch as soon as it arrives, so the 250ms
  // silence-based frame detection in loop() behaves as it did on the ESP8266.
  hwSerial.setRxFIFOFull(1);
}

// ESP32 replacement for ESP.getChipId(): last 3 bytes of the MAC address
String getChipIdHex() {
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i += 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  return String(chipId, HEX);
}

// Determines if Byte 1 indicates an active system fault flag
bool isFaultActive(int b1) {
  if (b1 >= 0xE0 || b1 == 0xFF) {
    return true;
  }
  return false;
}

// Determines if Byte 1 represents a state where temperature telemetry shouldn't be updated
bool isInvalidTempState(int b1) {
  return (b1 == 0x03 || b1 == 0x20 || b1 == 0x00);
}

// --------------------------------------------------------------------------------
// FUNCTION DECLARATIONS
// --------------------------------------------------------------------------------
void loadSettings();
void saveSettings();
void reconnectMQTT();
void publishHomeAssistantDiscovery();
void publishMQTTData(const String& payload, int b1, int b2, int b3, int b4, const String& statusText, bool isFault);
void setupOTA();
void handleRoot();
void handleMqtt();
void handleSaveMqtt();
void handleSerialConfig();
void handleSaveSerialConfig();
void handleWifi();
void handleSaveWifi();
void handleResetWifi();
void handleData();
String getLiveScript();

// --------------------------------------------------------------------------------
// WEB PAGE BUILDERS
// --------------------------------------------------------------------------------
String getHeader(String title) {
  String html;
  html.reserve(1600);
  html += F("<!DOCTYPE html><html><head>"
            "<meta charset='UTF-8'>"
            "<meta name='viewport' content='width=device-width, initial-scale=1'>");
  html += "<title>" + title + "</title>";
  html += F("<style>"
            "body{font-family:Arial,sans-serif;margin:0;padding:0;background:#f4f4f4;}"
            ".header-banner{background:#0066cc;color:#ffffff;padding:15px 20px;text-align:center;box-shadow:0 2px 4px rgba(0,0,0,0.1);}"
            ".header-banner h1{margin:0;font-size:24px;font-weight:bold;letter-spacing:0.5px;}"
            ".container{max-width:650px;background:#fff;padding:20px;margin:20px auto;border-radius:8px;box-shadow:0 0 10px rgba(0,0,0,0.1);}"
            "h2{color:#333;margin-top:15px;} nav a{margin-right:15px;text-decoration:none;color:#0066cc;font-weight:bold;}"
            "table{width:100%;border-collapse:collapse;margin-top:15px;} td,th{padding:8px;border-bottom:1px solid #ddd;text-align:left;}"
            "input[type='text'],input[type='number'],input[type='password'],select{width:100%;padding:8px;margin:4px 0 12px;box-sizing:border-box;}"
            "input[type='submit'],.btn{background:#28a745;color:#fff;border:none;padding:10px 15px;border-radius:4px;cursor:pointer;text-decoration:none;display:inline-block;}"
            ".btn-danger{background:#dc3545;color:#fff;border:none;padding:10px 15px;border-radius:4px;cursor:pointer;}"
            ".payload-box{background:#1e1e1e;color:#00ff66;font-family:monospace;padding:12px;border-radius:4px;word-break:break-all;font-size:14px;margin-top:10px;}"
            ".disconnected{color:#dc3545;font-weight:bold;}"
            ".fault-active{color:#dc3545;font-weight:bold;}"
            ".fault-ok{color:#28a745;font-weight:bold;}"
            ".card-danger{background:#fff3f3;border:1px solid #ffcdd2;padding:15px;border-radius:6px;margin-top:20px;}"
            "</style></head><body>");

  html += F("<div class='header-banner'><h1>Hot Water Serial Capture</h1></div>");
  html += F("<div class='container'>");
  html += F("<nav><a href='/'>Status</a><a href='/wifi'>Wi-Fi Setup</a><a href='/serial'>Serial Setup</a><a href='/mqtt'>MQTT Setup</a></nav><hr>");
  return html;
}

String getFooter() {
  return F("</div></body></html>");
}

void redirectHome() {
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

String getRssiDescription(int rssi) {
  if (rssi >= -60) return " (Strong)";
  if (rssi >= -75) return " (Medium)";
  if (rssi >= -85) return " (Weak)";
  return " (Very Weak)";
}

// --------------------------------------------------------------------------------
// LIVE-UPDATE JAVASCRIPT
// --------------------------------------------------------------------------------
String getLiveScript() {
  String js;
  js.reserve(1800);
  js += F("<script>"
    "function setText(id,t){var e=document.getElementById(id);if(e)e.textContent=t;}"
    "async function refreshStatus(){"
      "try{"
        "const res=await fetch('/data.json');"
        "if(!res.ok)return;"
        "const d=await res.json();"
        "setText('payloadVal',d.payload);"
        "setText('statusVal',d.status+' (0x'+Number(d.byte1).toString(16).toUpperCase()+')');"
        "setText('byte2Val',d.byte2+' \u00B0C');"
        "setText('byte3Val',d.byte3+' \u00B0C');"
        "setText('byte4Val',d.byte4+' \u00B0C');"
        "setText('countVal',d.count);"
        "setText('rssiVal',d.rssi+' dBm'+d.rssiDesc);"
        "var faultEl=document.getElementById('faultVal');"
        "if(faultEl){"
          "if(d.fault){faultEl.textContent='ACTIVE FAULT';faultEl.className='fault-active';}"
          "else{faultEl.textContent='Normal (OK)';faultEl.className='fault-ok';}"
        "}"
        "var mqttEl=document.getElementById('mqttVal');"
        "if(mqttEl){"
          "if(d.mqtt){mqttEl.textContent='Connected';mqttEl.className='';}"
          "else{mqttEl.textContent='Disconnected';mqttEl.className='disconnected';}"
        "}"
      "}catch(e){}"
    "}"
    "window.addEventListener('load',refreshStatus);"
    "setInterval(refreshStatus,1500);"
    "</script>");
  return js;
}

// --------------------------------------------------------------------------------
// WEB HANDLERS
// --------------------------------------------------------------------------------
void handleRoot() {
  String html;
  html.reserve(2800);
  html += getHeader("Hot Water System Status");

  html += F("<h2>System Status</h2><table>");
  html += "<tr><th>System Status (Byte 1)</th><td id='statusVal'>" + lastCapturedStatus + " (0x" + String(lastCapturedByte1, HEX) + ")</td></tr>";
  html += "<tr><th>Fault Status Flag</th><td id='faultVal' class='" + String(lastCapturedFault ? "fault-active" : "fault-ok") + "'>" + String(lastCapturedFault ? "ACTIVE FAULT" : "Normal (OK)") + "</td></tr>";
  html += "<tr><th>Tank Temp (Byte 2)</th><td id='byte2Val'>" + String(lastCapturedByte2) + " &deg;C</td></tr>";
  html += "<tr><th>Evaporator Temp (Byte 3)</th><td id='byte3Val'>" + String(lastCapturedByte3) + " &deg;C</td></tr>";
  html += "<tr><th>Compressor Temp (Byte 4)</th><td id='byte4Val'>" + String(lastCapturedByte4) + " &deg;C</td></tr>";
  html += "<tr><th>Total Packets Sent</th><td id='countVal'>" + String(packetCount) + "</td></tr>";
  html += "<tr><th>WiFi SSID</th><td>" + WiFi.SSID() + "</td></tr>";
  html += "<tr><th>WiFi IP</th><td>" + WiFi.localIP().toString() + "</td></tr>";

  int rssi = WiFi.RSSI();
  html += "<tr><th>WiFi RSSI Signal</th><td id='rssiVal'>" + String(rssi) + " dBm" + getRssiDescription(rssi) + "</td></tr>";

  bool mqttOk = mqttClient.connected();
  html += "<tr><th>MQTT Connection</th><td id='mqttVal' class='" + String(mqttOk ? "" : "disconnected") + "'>" + String(mqttOk ? "Connected" : "Disconnected") + "</td></tr>";
  html += "<tr><th>Baud Rate</th><td>" + String(serial_baud) + " bps</td></tr>";
  html += "<tr><th>RX Pin</th><td>GPIO " + String(RX_PIN) + "</td></tr>";
  html += F("</table>");

  html += F("<h2>Latest Captured Frame</h2>");
  html += "<div class='payload-box' id='payloadVal'>" + lastCapturedPayload + "</div>";

  html += getLiveScript();
  html += getFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

// --------------------------------------------------------------------------------
// WI-FI SETUP HANDLERS
// --------------------------------------------------------------------------------
void handleWifi() {
  String html;
  html.reserve(3000);
  html += getHeader("Wi-Fi Setup");

  html += F("<h2>Connect to a Wi-Fi Network</h2>");
  html += "<p>Current Network: <b>" + (WiFi.SSID().length() > 0 ? WiFi.SSID() : String("Not Connected")) + "</b></p>";

  // Scan networks
  int n = WiFi.scanNetworks();
  
  html += F("<form action='/save-wifi' method='POST'>");
  html += F("<label for='ssid'>Select Wi-Fi Network (SSID):</label><br>");
  html += F("<select name='ssid' id='ssid'>");

  if (n == 0) {
    html += F("<option value=''>No Wi-Fi networks found</option>");
  } else {
    for (int i = 0; i < n; ++i) {
      String ssidName = WiFi.SSID(i);
      int rssi = WiFi.RSSI(i);
      String enc = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Secured";
      
      html += "<option value='" + ssidName + "'";
      if (ssidName == WiFi.SSID()) html += " selected";
      html += ">" + ssidName + " (" + String(rssi) + " dBm, " + enc + ")</option>";
    }
  }
  html += F("</select><br>");

  html += F("Or enter SSID manually:<br>");
  html += F("<input type='text' name='manual_ssid' placeholder='Manual SSID (optional)'><br>");

  html += F("Wi-Fi Password:<br>");
  html += F("<input type='password' name='password' placeholder='Network Password'><br>");

  html += F("<input type='submit' value='Join Network'></form>");

  // Forget / Reset Section
  html += F("<div class='card-danger'>");
  html += F("<h3 style='color:#dc3545;margin-top:0;'>Forget Wi-Fi Credentials</h3>");
  html += F("<p>Clicking the button below will erase saved Wi-Fi details and reboot the device. ") ;
  html += F("It will restart in Access Point mode (<b>ESP32-HotWater-AP</b>) to allow re-configuration via WiFiManager.</p>");
  html += F("<form action='/reset-wifi' method='POST' onsubmit='return confirm(\"Are you sure you want to forget Wi-Fi credentials and restart in AP mode?\");'>");
  html += F("<input type='submit' class='btn-danger' value='Forget Wi-Fi & Restart AP'>");
  html += F("</form></div>");

  html += getFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

void handleSaveWifi() {
  String selectedSsid = server.arg("ssid");
  String manualSsid   = server.arg("manual_ssid");
  String password     = server.arg("password");

  String targetSsid = (manualSsid.length() > 0) ? manualSsid : selectedSsid;

  if (targetSsid.length() == 0) {
    server.send(400, "text/plain", "No SSID selected or provided.");
    return;
  }

  String responseHtml = getHeader("Connecting Wi-Fi");
  responseHtml += "<h2>Connecting to " + targetSsid + "...</h2>";
  responseHtml += "<p>The device is applying new Wi-Fi credentials and rebooting. Please reconnect your computer/phone to <b>" + targetSsid + "</b> and check your router's device list or Home Assistant dashboard for the new IP address.</p>";
  responseHtml += getFooter();

  server.send(200, "text/html; charset=utf-8", responseHtml);
  delay(1000);

  // Apply new credentials
  WiFi.persistent(true);
  WiFi.begin(targetSsid.c_str(), password.c_str());
  delay(1000);
  ESP.restart();
}

void handleResetWifi() {
  String responseHtml = getHeader("Resetting Wi-Fi");
  responseHtml += F("<h2>Wi-Fi Reset Initiated</h2>");
  responseHtml += F("<p>Wi-Fi credentials have been erased. The ESP32 is restarting and will launch the captive portal AP (<b>ESP32-HotWater-AP</b>).</p>");
  responseHtml += getFooter();

  server.send(200, "text/html; charset=utf-8", responseHtml);
  delay(1000);

  WiFiManager wm;
  wm.resetSettings();
  delay(500);
  ESP.restart();
}

// --------------------------------------------------------------------------------
// OTHER HANDLERS
// --------------------------------------------------------------------------------
void handleSerialConfig() {
  String html;
  html.reserve(2048);
  html += getHeader("Serial Calibration");
  html += F("<h2>Serial Port Setup</h2><form action='/save-serial' method='POST'>");

  html += "Serial Baud Rate:<br>"
          "<input type='number' name='baud' value='" + String(serial_baud) + "'><br>";

  html += F("<input type='submit' value='Save Settings'></form>");
  html += getFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

void handleSaveSerialConfig() {
  if (server.hasArg("baud")) {
    int newBaud = server.arg("baud").toInt();
    if (newBaud > 0) {
      serial_baud = newBaud;
      hwSerial.end();
      beginHotWaterSerial();
    }
  }
  saveSettings();
  redirectHome();
}

void handleMqtt() {
  String html;
  html.reserve(2048);
  html += getHeader("MQTT Configuration");
  html += F("<h2>MQTT Broker Settings</h2><form action='/save-mqtt' method='POST'>");
  html += "Broker IP / Host: <input type='text' name='mqtt_server' value='" + String(mqtt_server) + "'><br>";
  html += "Broker Port: <input type='number' name='mqtt_port' value='" + String(mqtt_port) + "'><br>";
  html += "User: <input type='text' name='mqtt_user' value='" + String(mqtt_user) + "'><br>";
  html += "Password: <input type='password' name='mqtt_pass' value='" + String(mqtt_pass) + "'><br>";
  html += "Topic Prefix: <input type='text' name='mqtt_topic' value='" + String(mqtt_topic) + "'><br>";

  html += F("<input type='submit' value='Save &amp; Reconnect'></form>");
  html += getFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

void handleSaveMqtt() {
  if (server.hasArg("mqtt_server")) {
    strncpy(mqtt_server, server.arg("mqtt_server").c_str(), sizeof(mqtt_server) - 1);
    mqtt_server[sizeof(mqtt_server) - 1] = '\0';
  }
  if (server.hasArg("mqtt_port")) mqtt_port = server.arg("mqtt_port").toInt();
  if (server.hasArg("mqtt_user")) {
    strncpy(mqtt_user, server.arg("mqtt_user").c_str(), sizeof(mqtt_user) - 1);
    mqtt_user[sizeof(mqtt_user) - 1] = '\0';
  }
  if (server.hasArg("mqtt_pass")) {
    strncpy(mqtt_pass, server.arg("mqtt_pass").c_str(), sizeof(mqtt_pass) - 1);
    mqtt_pass[sizeof(mqtt_pass) - 1] = '\0';
  }
  if (server.hasArg("mqtt_topic")) {
    strncpy(mqtt_topic, server.arg("mqtt_topic").c_str(), sizeof(mqtt_topic) - 1);
    mqtt_topic[sizeof(mqtt_topic) - 1] = '\0';
  }

  saveSettings();

  if (strlen(mqtt_server) > 0) {
    mqttClient.setServer(mqtt_server, mqtt_port);
    if (mqttClient.connected()) mqttClient.disconnect();
  }
  redirectHome();
}

// --------------------------------------------------------------------------------
// LIVE DATA ENDPOINT
// --------------------------------------------------------------------------------
void handleData() {
  int rssi = WiFi.RSSI();
  JsonDocument doc;
  doc["payload"]  = lastCapturedPayload;
  doc["byte1"]    = lastCapturedByte1;
  doc["status"]   = lastCapturedStatus;
  doc["fault"]    = lastCapturedFault;
  doc["byte2"]    = lastCapturedByte2;
  doc["byte3"]    = lastCapturedByte3;
  doc["byte4"]    = lastCapturedByte4;
  doc["count"]    = packetCount;
  doc["rssi"]     = rssi;
  doc["rssiDesc"] = getRssiDescription(rssi);
  doc["mqtt"]     = mqttClient.connected();
  doc["ip"]       = WiFi.localIP().toString();

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json; charset=utf-8", out);
}

// --------------------------------------------------------------------------------
// INITIAL SETUP PROCEDURE
// --------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // true = format the filesystem automatically if it cannot be mounted
  if (!LittleFS.begin(true)) {
    Serial.println(F("LittleFS mount failed."));
  }
  loadSettings();

  beginHotWaterSerial();

  // Onboard blue LED (GPIO 8, active LOW): ON while trying to join WiFi /
  // while the setup AP is running, OFF once connected.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println(F("Starting WiFi..."));

  // ESP32-C3 Super Mini fix: these boards have a poorly matched antenna and
  // often can't join WiFi (or their setup AP is invisible) at full TX power.
  // Dropping TX power to 8.5 dBm is the well-known cure.
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  wm.setAPCallback([](WiFiManager*) {
    WiFi.setTxPower(WIFI_POWER_8_5dBm);   // keep low power in AP mode too
    Serial.println(F("No saved WiFi - setup AP 'ESP32-HotWater-AP' started (192.168.4.1)"));
  });
  if (!wm.autoConnect("ESP32-HotWater-AP")) {
    Serial.println(F("Failed to connect to WiFi. Restarting..."));
    ESP.restart();
  }
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.setSleep(false);                  // more reliable web/OTA/MQTT response
  digitalWrite(LED_PIN, HIGH);           // LED off = connected
  Serial.println("Connected to WiFi! Device IP: " + WiFi.localIP().toString());

  setupOTA();

  if (strlen(mqtt_server) > 0) {
    mqttClient.setServer(mqtt_server, mqtt_port);
  }
  mqttClient.setSocketTimeout(2);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/wifi", HTTP_GET, handleWifi);
  server.on("/save-wifi", HTTP_POST, handleSaveWifi);
  server.on("/reset-wifi", HTTP_POST, handleResetWifi);
  server.on("/serial", HTTP_GET, handleSerialConfig);
  server.on("/save-serial", HTTP_POST, handleSaveSerialConfig);
  server.on("/mqtt", HTTP_GET, handleMqtt);
  server.on("/save-mqtt", HTTP_POST, handleSaveMqtt);
  server.on("/data.json", HTTP_GET, handleData);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });

  server.begin();
  Serial.println(F("Web server started on port 80"));
}

// --------------------------------------------------------------------------------
// MAIN APPLICATION LOOP
// --------------------------------------------------------------------------------
void loop() {
  ArduinoOTA.handle();
  server.handleClient();

  if (strlen(mqtt_server) > 0) {
    if (!mqttClient.connected()) {
      if (millis() - lastMQTTAttempt > 5000) {
        lastMQTTAttempt = millis();
        reconnectMQTT();
      }
    } else {
      mqttClient.loop();
    }
  }

  // 1. Read incoming byte stream from the hardware UART
  while (hwSerial.available()) {
    uint8_t b = hwSerial.read();
    if (rxBufferPos < sizeof(rxBuffer)) {
      rxBuffer[rxBufferPos++] = b;
    }
    lastRxByteMs = millis();
  }

  // 2. Detect end-of-frame timeout (bus idle >= 250ms)
  if (rxBufferPos > 0 && (millis() - lastRxByteMs >= PACKET_TIMEOUT_MS)) {
    String hexString = "";
    for (size_t i = 0; i < rxBufferPos; i++) {
      if (rxBuffer[i] < 0x10) hexString += "0";
      hexString += String(rxBuffer[i], HEX) + " ";
    }
    hexString.toUpperCase();
    hexString.trim();

    // Extract Bytes 1, 2, 3, and 4 as decimal values
    int b1 = (rxBufferPos > 1) ? rxBuffer[1] : 0;
    int b2 = (rxBufferPos > 2) ? rxBuffer[2] : 0;
    int b3 = (rxBufferPos > 3) ? rxBuffer[3] : 0;
    int b4 = (rxBufferPos > 4) ? rxBuffer[4] : 0;

    // Discard the very first frame captured after boot. The ESP32 starts
    // listening mid-stream on the 300-baud line with no sync/start condition,
    // so the first silence-to-silence "frame" it assembles can be a torn or
    // misaligned read of whatever was already on the wire - not a real
    // reading (this is what was showing up as a bogus 0x38 tank temp). Wait
    // for the next, properly framed packet before trusting or publishing
    // anything.
    if (isFirstPacket) {
      isFirstPacket = false;
      lastCapturedPayload = hexString;   // still show the raw frame on the dashboard
      rxBufferPos = 0;
      return;
    }

    String statusText = getStatusText(b1);
    bool faultState   = isFaultActive(b1);

    lastCapturedPayload = hexString;
    lastCapturedByte1   = b1;
    lastCapturedStatus  = statusText;
    lastCapturedFault   = faultState;

    // Only update web UI temperature variables if Byte 1 is in a valid state
    if (!isInvalidTempState(b1) && b2 != 0) {
      lastCapturedByte2 = b2;
      lastCapturedByte3 = b3;
      lastCapturedByte4 = b4;
    }

    // Always increment packet counter and publish MQTT payload
    if (hexString != lastSentPayload) {
      lastSentPayload = hexString;
      packetCount++;

      if (mqttClient.connected()) {
        publishMQTTData(hexString, b1, b2, b3, b4, statusText, faultState);
      }
    }

    rxBufferPos = 0;
  }
}

// --------------------------------------------------------------------------------
// OTA SETUP
// --------------------------------------------------------------------------------
void setupOTA() {
  ArduinoOTA.setHostname("ESP32-HotWaterMonitor");

  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    if (ArduinoOTA.getCommand() != U_FLASH) {
      LittleFS.end();
    }
  });

  ArduinoOTA.begin();
}

// --------------------------------------------------------------------------------
// MQTT CONNECTION & RECONNECT (LWT SETUP)
// --------------------------------------------------------------------------------
void reconnectMQTT() {
  if (strlen(mqtt_server) == 0) return;

  String clientId = "ESP32HotWater-" + getChipIdHex();
  String availTopic = String(mqtt_topic) + "/availability";

  bool ok;
  if (strlen(mqtt_user) > 0) {
    ok = mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass, availTopic.c_str(), 1, true, "offline");
  } else {
    ok = mqttClient.connect(clientId.c_str(), nullptr, nullptr, availTopic.c_str(), 1, true, "offline");
  }

  if (ok) {
    // Make sure the buffer is large enough BEFORE the first publish below.
    // The default PubSubClient buffer (256 bytes) is too small for the
    // "clearing" payload once the null fields are added, and an oversized
    // publish fails silently.
    mqttClient.setBufferSize(1024);

    // Reset cached temperature readings on every (re)connect.
    lastValidByte2 = -1;
    lastValidByte3 = -1;
    lastValidByte4 = -1;

    // STEP 1: publish the "clearing" state BEFORE announcing "online".
    // Home Assistant keeps the last known sensor value while a device is
    // unavailable, and shows that stale value the instant availability flips
    // back to "online" (this was the bogus first reading of the day).
    // Simply omitting byte_2/3/4 does NOT reset it - the template errors and
    // HA keeps the old value - so we send explicit JSON nulls, which render
    // as "None" and set the sensors to "unknown" (a gap, not a spike).
    JsonDocument clearDoc;
    clearDoc["raw_hex"] = "Reconnected - awaiting fresh frame";
    clearDoc["byte_1"]  = 0;
    clearDoc["status"]  = "Reconnected";
    clearDoc["fault"]   = false;
    clearDoc["byte_2"]  = nullptr;   // serialises as JSON null
    clearDoc["byte_3"]  = nullptr;
    clearDoc["byte_4"]  = nullptr;
    clearDoc["count"]   = packetCount;
    clearDoc["ip"]      = WiFi.localIP().toString();
    String clearPayload;
    serializeJson(clearDoc, clearPayload);
    String stateTopic = String(mqtt_topic) + "/state";
    mqttClient.publish(stateTopic.c_str(), clearPayload.c_str());

    // STEP 2: discovery config, then mark the device online LAST.
    publishHomeAssistantDiscovery();
    mqttClient.publish(availTopic.c_str(), "online", true);
  }
}

// --------------------------------------------------------------------------------
// HOME ASSISTANT MQTT AUTO-DISCOVERY
// --------------------------------------------------------------------------------
void publishHomeAssistantDiscovery() {
  String chipId = getChipIdHex();
  String stateTopic = String(mqtt_topic) + "/state";
  String availTopic = String(mqtt_topic) + "/availability";

  mqttClient.setBufferSize(1024);

  // 1. Raw Hex Telemetry Sensor
  {
    JsonDocument doc;
    doc["name"] = "Hot Water Telemetry Frame";
    doc["state_topic"] = stateTopic;
    doc["availability_topic"] = availTopic;
    doc["unique_id"] = "hotwater_" + chipId + "_telemetry";
    doc["value_template"] = "{{ value_json.raw_hex }}";
    doc["icon"] = "mdi:code-string";

    JsonObject dev = doc["device"].to<JsonObject>();
    dev["identifiers"][0] = "esp32_hotwater_" + chipId;
    dev["name"] = "Hot Water Serial Monitor";
    dev["model"] = "ESP32 Serial Sniffer";
    dev["manufacturer"] = "Custom DIY";

    String discTopic = "homeassistant/sensor/hotwater_" + chipId + "_telemetry/config";
    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(discTopic.c_str(), payload.c_str(), true);
  }

  // 2. Byte 1 - Operating State Sensor (Text)
  {
    JsonDocument doc;
    doc["name"] = "Hot Water Operating State";
    doc["state_topic"] = stateTopic;
    doc["availability_topic"] = availTopic;
    doc["unique_id"] = "hotwater_" + chipId + "_status";
    doc["value_template"] = "{{ value_json.status }}";
    doc["icon"] = "mdi:heat-pump";

    JsonObject dev = doc["device"].to<JsonObject>();
    dev["identifiers"][0] = "esp32_hotwater_" + chipId;
    dev["name"] = "Hot Water Serial Monitor";
    dev["model"] = "ESP32 Serial Sniffer";
    dev["manufacturer"] = "Custom DIY";

    String discTopic = "homeassistant/sensor/hotwater_" + chipId + "_status/config";
    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(discTopic.c_str(), payload.c_str(), true);
  }

  // 3. Fault Binary Sensor (ON when fault exists)
  {
    JsonDocument doc;
    doc["name"] = "Hot Water Fault Alert";
    doc["state_topic"] = stateTopic;
    doc["availability_topic"] = availTopic;
    doc["unique_id"] = "hotwater_" + chipId + "_fault";
    doc["value_template"] = "{{ 'ON' if value_json.fault else 'OFF' }}";
    doc["device_class"] = "problem";
    doc["icon"] = "mdi:alert-circle";

    JsonObject dev = doc["device"].to<JsonObject>();
    dev["identifiers"][0] = "esp32_hotwater_" + chipId;
    dev["name"] = "Hot Water Serial Monitor";
    dev["model"] = "ESP32 Serial Sniffer";
    dev["manufacturer"] = "Custom DIY";

    String discTopic = "homeassistant/binary_sensor/hotwater_" + chipId + "_fault/config";
    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(discTopic.c_str(), payload.c_str(), true);
  }

  // 4. Byte 1 - Operating Mode Code (Numeric Hex/Dec)
  {
    JsonDocument doc;
    doc["name"] = "Hot Water Operating Code (Byte 1)";
    doc["state_topic"] = stateTopic;
    doc["availability_topic"] = availTopic;
    doc["unique_id"] = "hotwater_" + chipId + "_byte1";
    doc["value_template"] = "{{ value_json.byte_1 }}";
    doc["icon"] = "mdi:numeric";

    JsonObject dev = doc["device"].to<JsonObject>();
    dev["identifiers"][0] = "esp32_hotwater_" + chipId;
    dev["name"] = "Hot Water Serial Monitor";
    dev["model"] = "ESP32 Serial Sniffer";
    dev["manufacturer"] = "Custom DIY";

    String discTopic = "homeassistant/sensor/hotwater_" + chipId + "_byte1/config";
    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(discTopic.c_str(), payload.c_str(), true);
  }

  // 5. Byte 2 - Tank Temp Sensor
  {
    JsonDocument doc;
    doc["name"] = "Hot Water Tank Temp";
    doc["state_topic"] = stateTopic;
    doc["availability_topic"] = availTopic;
    doc["unique_id"] = "hotwater_" + chipId + "_byte2";
    doc["value_template"] = "{{ value_json.byte_2 }}";
    doc["unit_of_measurement"] = "°C";
    doc["device_class"] = "temperature";
    doc["state_class"] = "measurement";
    doc["icon"] = "mdi:thermometer-water";

    JsonObject dev = doc["device"].to<JsonObject>();
    dev["identifiers"][0] = "esp32_hotwater_" + chipId;
    dev["name"] = "Hot Water Serial Monitor";
    dev["model"] = "ESP32 Serial Sniffer";
    dev["manufacturer"] = "Custom DIY";

    String discTopic = "homeassistant/sensor/hotwater_" + chipId + "_byte2/config";
    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(discTopic.c_str(), payload.c_str(), true);
  }

  // 6. Byte 3 - Evaporator Temp Sensor
  {
    JsonDocument doc;
    doc["name"] = "Hot Water Evaporator Temp";
    doc["state_topic"] = stateTopic;
    doc["availability_topic"] = availTopic;
    doc["unique_id"] = "hotwater_" + chipId + "_byte3";
    doc["value_template"] = "{{ value_json.byte_3 }}";
    doc["unit_of_measurement"] = "°C";
    doc["device_class"] = "temperature";
    doc["state_class"] = "measurement";
    doc["icon"] = "mdi:snowflake";

    JsonObject dev = doc["device"].to<JsonObject>();
    dev["identifiers"][0] = "esp32_hotwater_" + chipId;
    dev["name"] = "Hot Water Serial Monitor";
    dev["model"] = "ESP32 Serial Sniffer";
    dev["manufacturer"] = "Custom DIY";

    String discTopic = "homeassistant/sensor/hotwater_" + chipId + "_byte3/config";
    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(discTopic.c_str(), payload.c_str(), true);
  }

  // 7. Byte 4 - Compressor Temp Sensor
  {
    JsonDocument doc;
    doc["name"] = "Hot Water Compressor Temp";
    doc["state_topic"] = stateTopic;
    doc["availability_topic"] = availTopic;
    doc["unique_id"] = "hotwater_" + chipId + "_byte4";
    doc["value_template"] = "{{ value_json.byte_4 }}";
    doc["unit_of_measurement"] = "°C";
    doc["device_class"] = "temperature";
    doc["state_class"] = "measurement";
    doc["icon"] = "mdi:engine-outline";

    JsonObject dev = doc["device"].to<JsonObject>();
    dev["identifiers"][0] = "esp32_hotwater_" + chipId;
    dev["name"] = "Hot Water Serial Monitor";
    dev["model"] = "ESP32 Serial Sniffer";
    dev["manufacturer"] = "Custom DIY";

    String discTopic = "homeassistant/sensor/hotwater_" + chipId + "_byte4/config";
    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(discTopic.c_str(), payload.c_str(), true);
  }

  // 8. IP Address Diagnostic Sensor
  {
    JsonDocument doc;
    doc["name"] = "Hot Water Monitor IP Address";
    doc["state_topic"] = stateTopic;
    doc["availability_topic"] = availTopic;
    doc["unique_id"] = "hotwater_" + chipId + "_ip";
    doc["value_template"] = "{{ value_json.ip }}";
    doc["entity_category"] = "diagnostic";
    doc["icon"] = "mdi:ip-network";

    JsonObject dev = doc["device"].to<JsonObject>();
    dev["identifiers"][0] = "esp32_hotwater_" + chipId;
    dev["name"] = "Hot Water Serial Monitor";
    dev["model"] = "ESP32 Serial Sniffer";
    dev["manufacturer"] = "Custom DIY";

    String discTopic = "homeassistant/sensor/hotwater_" + chipId + "_ip/config";
    String payload;
    serializeJson(doc, payload);
    mqttClient.publish(discTopic.c_str(), payload.c_str(), true);
  }
}

// --------------------------------------------------------------------------------
// TELEMETRY PUBLISHER
// --------------------------------------------------------------------------------
void publishMQTTData(const String& payload, int b1, int b2, int b3, int b4, const String& statusText, bool isFault) {
  bool isUnknownStatus = statusText.startsWith("Unknown");
  bool currentReadingValid = !isUnknownStatus && !isInvalidTempState(b1) && b2 != 0;

  // Only update cached readings if Byte 1 is valid AND the incoming temperature byte is non-zero
  if (currentReadingValid) {
    lastValidByte2 = b2;
    lastValidByte3 = b3;
    lastValidByte4 = b4;
  }

  JsonDocument doc;
  doc["raw_hex"]  = payload;
  doc["byte_1"]   = b1;
  doc["status"]   = statusText;
  doc["fault"]    = isFault;

  // Only attach temperature fields when THIS frame is a genuinely fresh,
  // valid reading. Previously these always re-sent the cached value even
  // while the unit was booting/idle, which could republish a stale cached
  // reading as if it were current. Combined with the reconnect-time cache
  // clear below, this stops the "yesterday's temp" spike at startup.
  if (currentReadingValid) {
    doc["byte_2"] = lastValidByte2;
    doc["byte_3"] = lastValidByte3;
    doc["byte_4"] = lastValidByte4;
  }

  doc["count"]    = packetCount;
  doc["ip"]       = WiFi.localIP().toString();

  String jsonPayload;
  serializeJson(doc, jsonPayload);
  String stateTopic = String(mqtt_topic) + "/state";
  mqttClient.publish(stateTopic.c_str(), jsonPayload.c_str());
}

// --------------------------------------------------------------------------------
// PERSISTENT STORAGE (LittleFS JSON)
// --------------------------------------------------------------------------------
void loadSettings() {
  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) return;

  strncpy(mqtt_server, doc["mqtt_server"] | "", sizeof(mqtt_server) - 1);
  mqtt_server[sizeof(mqtt_server) - 1] = '\0';
  mqtt_port = doc["mqtt_port"] | 1883;
  strncpy(mqtt_user, doc["mqtt_user"] | "", sizeof(mqtt_user) - 1);
  mqtt_user[sizeof(mqtt_user) - 1] = '\0';
  strncpy(mqtt_pass, doc["mqtt_pass"] | "", sizeof(mqtt_pass) - 1);
  mqtt_pass[sizeof(mqtt_pass) - 1] = '\0';
  strncpy(mqtt_topic, doc["mqtt_topic"] | "homeassistant/sensor/hotwater", sizeof(mqtt_topic) - 1);
  mqtt_topic[sizeof(mqtt_topic) - 1] = '\0';

  serial_baud = doc["serial_baud"] | 300;
}

void saveSettings() {
  JsonDocument doc;

  doc["mqtt_server"] = mqtt_server;
  doc["mqtt_port"]   = mqtt_port;
  doc["mqtt_user"]   = mqtt_user;
  doc["mqtt_pass"]   = mqtt_pass;
  doc["mqtt_topic"]  = mqtt_topic;
  doc["serial_baud"] = serial_baud;

  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) return;
  serializeJson(doc, f);
  f.close();
}