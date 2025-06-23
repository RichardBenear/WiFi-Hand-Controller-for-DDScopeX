// ===============================================
// ======== WiFi Screen Mirror for DDScope =======
// ===============================================
// WifiDisplay.cpp
//
// Processor is an ESP32-S3 connected via USB to the Teensy4.1 running DDScopeX code
// 
// See rights and use declaration in License.h
//
// **** by Richard Benear 5/21/2025 ****
//
#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include "esp_wifi.h"
#include <esp_heap_caps.h>  // Needed for PSRAM allocation
#include "SPIFFS.h"
#include "esp32/spiram.h"
#include "../include/secrets.h"

#define SERIAL_TEENSY Serial  // USB port
#define SERIAL_DEBUG Serial0

#define FRAME_TYPE_RAW 0x00
#define FRAME_TYPE_RLE 0x01
#define FRAME_TYPE_DEF 0x04

#define FRAME_MAX_SIZE 307200  // For uncompressed 320 x 480 RGB565
#define TEENSY_ACK_TIMEOUT 300

#define HEADER_SIZE 5  // 1 byte type + 4 bytes size
#define FRAME_TOTAL_SIZE (FRAME_MAX_SIZE + HEADER_SIZE)


#define WIFI_DISPLAY_AP_SSID  "WIFI-DISPLAY-ESP32"
#define WIFI_DISPLAY_AP_PASSWORD        "password"
#define WIFI_DISPLAY_AP_IP_ADDR      {192,168,4,2} 
#define WIFI_DISPLAY_AP_GW_ADDR      {192,168,4,2}

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");  // WebSocket endpoint

WiFiServer lx200Server(4030);
WiFiClient lx200Client;

uint32_t frameSize = 0;
size_t bytesRead = 0;
uint8_t *fullFrameBuffer = nullptr;

enum EspState {
  SEND_IP,
  SEND_CONNECTED_STATUS,
  WAIT_FOR_CONNECTED_ACK,
  WAIT_FOR_FRAME
};

EspState espState = SEND_IP;

static String lx200Cmd = "";

volatile bool clientConnected = false;
volatile bool enableProcessFrame = false;
volatile bool touchDetected = false;
int touchX = 0;
int touchY = 0;

bool hasValidRADecCache = false;

String cachedRA = "00:00:00#";
String cachedDec = "+00*00:00#";
unsigned long lastRADecUpdate = 0;

volatile bool lockSerialTeensy = false;
portMUX_TYPE teensySerialMux = portMUX_INITIALIZER_UNLOCKED;

// wrapper functions for mutex
void sendToTeensyByte(uint8_t b) {
  portENTER_CRITICAL(&teensySerialMux);
  SERIAL_TEENSY.write(b);
  portEXIT_CRITICAL(&teensySerialMux);
}

void sendToTeensyBytes(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    portENTER_CRITICAL(&teensySerialMux);
    SERIAL_TEENSY.write(data[i]);
    portEXIT_CRITICAL(&teensySerialMux);
  }
}

void sendToTeensyString(const String& s) {
  portENTER_CRITICAL(&teensySerialMux);
  SERIAL_TEENSY.print(s);
  portEXIT_CRITICAL(&teensySerialMux);
}

void sendToTeensyFlush() {
  portENTER_CRITICAL(&teensySerialMux);
  SERIAL_TEENSY.flush();
  portEXIT_CRITICAL(&teensySerialMux);
}

// Send any Touch event to Teensy
// Touch event is asynchronous from the loop().
void sendTouchCoord() {
  if (touchDetected) {

    // Send to Teensy over USB CDC
    SERIAL_TEENSY.write('T');                  // Touch event marker
    SERIAL_TEENSY.write((uint8_t)(touchX >> 8));    // x high byte
    SERIAL_TEENSY.write((uint8_t)(touchX & 0xFF));  // x low byte
    SERIAL_TEENSY.write((uint8_t)(touchY >> 8));    // y high byte
    SERIAL_TEENSY.write((uint8_t)(touchY & 0xFF));  // y low byte
    touchDetected = false;
    ws.textAll("ack");  // Send acknowledgment back to client
  }
}

// Send the IP address to Teensy
String sendIpToTeensy() {
  IPAddress ip = WiFi.localIP();
  String ipMsg = "IP:" + ip.toString() + "\n";
  SERIAL_TEENSY.print(ipMsg);
  SERIAL_TEENSY.flush();
  return ipMsg;
}

// ==============================================================
// Receive the Frame Header and compressed Frame data from Teensy
// ==============================================================
// The metadata byte sent to the WebSocket client:
// 0x00 = Raw
// 0x01 = Compressed RLE
// 0x04 = Compressed Deflate
void processFrame() {
  if (!enableProcessFrame) return;
  frameSize = 0;
  bytesRead = 0;

  // STEP 1: Wait for and verify the sync byte
  if (SERIAL_TEENSY.peek() == 'Z') {
    SERIAL_TEENSY.read();
    SERIAL_TEENSY.write(0x06);  // ACK
    SERIAL_TEENSY.flush();
    //SERIAL_DEBUG.println("Got 'Z'");
  } else {
    //SERIAL_DEBUG.printf("Unexpected byte: 0x%02X (expected 'Z')\n", SERIAL_TEENSY.peek());
    //SERIAL_TEENSY.read();  // Discard it
    return;
  }
  delay(3);
  //******************************************************* */
  if (SERIAL_TEENSY.available() >= HEADER_SIZE) {
    // Read 5-byte header
    SERIAL_TEENSY.readBytes(fullFrameBuffer, HEADER_SIZE);

    uint8_t frameType = fullFrameBuffer[0];
    uint32_t frameSize = fullFrameBuffer[1] | (fullFrameBuffer[2] << 8) |
                         (fullFrameBuffer[3] << 16) |
                         (fullFrameBuffer[4] << 24);

    //SERIAL_DEBUG.printf("Frame type: 0x%02X\n", fullFrameBuffer[0]);
    //SERIAL_DEBUG.printf("Size (%u bytes) as [0x%02X 0x%02X 0x%02X 0x%02X]\n",
    //                    frameSize, fullFrameBuffer[1], fullFrameBuffer[2],
    //                    fullFrameBuffer[3], fullFrameBuffer[4]);

    if (frameSize == 0 || frameSize > FRAME_MAX_SIZE) {
      SERIAL_DEBUG.printf("Invalid frame size: %u\n", frameSize);
      espState = SEND_IP;
      return;
    }
   

    // Wait until full frame arrives
    size_t bytesToRead = frameSize;
    uint8_t *payloadPtr = fullFrameBuffer + HEADER_SIZE;
    unsigned long startWaitPayload = millis();

    while (bytesToRead > 0) {
      int availableBytes = SERIAL_TEENSY.available();
      if (availableBytes > 0) {
        int chunk = min((int)bytesToRead, availableBytes);
        int bytesReadNow = SERIAL_TEENSY.readBytes(payloadPtr, chunk);
        payloadPtr += bytesReadNow;
        bytesToRead -= bytesReadNow;
        delayMicroseconds(100);
      } else {
        if (millis() - startWaitPayload > 300) {
          SERIAL_DEBUG.println("Timeout waiting for frame payload");
          return;
        }
        delayMicroseconds(200); //  a yield to USB 
      }
    }
    // Send compressed data over WebSocket
    ws.binaryAll(fullFrameBuffer, HEADER_SIZE + frameSize);
  }
}

// =========== Frame handshaking Handler ====================
// =========== State Flow Signal Meanings ===================
//  Byte	     Meaning
//   'D'     ESP32S3 sends: Client has disconnected
//   'I'	   ESP32S3 sends: IP address (e.g., I192.168.1.55\n)
//   'K'	   ESP32S3 receives: Teensy acknowledgement
//   'C'	   ESP32S3 sends: Client is connected
//   'K'	   ESP32S3 receives: Teensy acknowledgement
//   'T'     ESP32S3 sends: Touchscreen event
//   'Z'     ESP32S3 receives: Start frame signal from Teensy
void checkFrameState() {
  static bool prevClientConnected = false;

  // Detect Web Client Disconnect (i.e. falling edge)
  if (prevClientConnected && !clientConnected) {
    SERIAL_DEBUG.println("[ESP32] Web client disconnected — sending 'D' to Teensy");
    SERIAL_TEENSY.write('D');
    SERIAL_TEENSY.flush();
    espState = SEND_IP;
  }
  prevClientConnected = clientConnected;
  String wifiStaIpStr = "";

  switch (espState) {
    case SEND_IP:
      enableProcessFrame = false;
      //SERIAL_DEBUG.println("[ESP32] Sending IP to Teensy");
      wifiStaIpStr = sendIpToTeensy();
      delay(1);
      if (SERIAL_TEENSY.available()) {
        char incoming = SERIAL_TEENSY.read();
        if (incoming == 'K') {
          SERIAL_DEBUG.println("[ESP32] Received IP ACK");
          SERIAL_DEBUG.print("IP Address: ");
          SERIAL_DEBUG.println(wifiStaIpStr);
          espState = SEND_CONNECTED_STATUS;
        }
      }
      break;

    case SEND_CONNECTED_STATUS:
      if (clientConnected) {
        SERIAL_DEBUG.println("[ESP32] Sending 'C'");
        SERIAL_TEENSY.write('C');
        espState = WAIT_FOR_CONNECTED_ACK;
      }
      break;

    case WAIT_FOR_CONNECTED_ACK:
       if (SERIAL_TEENSY.available()) {
        char incoming = SERIAL_TEENSY.read();
        if (incoming == 'K') {
          SERIAL_DEBUG.println("[ESP32] Teensy ACK connected");
          enableProcessFrame = true;  // Start the Frame Processing
          SERIAL_DEBUG.println("Setting enableProcessFrame = true");
          espState = WAIT_FOR_FRAME;
        }
      }
      break;

    case WAIT_FOR_FRAME:
      processFrame();
      break;

    default:
      //espState = SEND_IP;
      break;
  }
}

// ===============================================
// =================== SETUP =====================
// ===============================================
void setup() {
  delay(2000);

  SERIAL_DEBUG.begin(115200);
  SERIAL_DEBUG.println("Debug port started");

  // USB port begin...probably already initialized, but?
  SERIAL_TEENSY.begin(1000000); // baud rate doesn't affect anything
  delay(1000);

  if (!psramFound()) {
    SERIAL_DEBUG.println("PSRAM not found!");
    while (true);  // Halt
  }
  SERIAL_DEBUG.printf("PSRAM Size: %u bytes\n", ESP.getPsramSize());

  // Need SPIFFS because we are serving the HTML
  if (!SPIFFS.begin(true)) {
    SERIAL_DEBUG.println("Failed to mount SPIFFS");
    return;
  }
  SERIAL_DEBUG.println("SPIFFS mounted");

  // Single receive buffer in PSRAM
  fullFrameBuffer = (uint8_t *)ps_malloc(FRAME_TOTAL_SIZE);
  if (!fullFrameBuffer) {
    Serial.println("ERROR: PSRAM malloc failed for fullFrameBuffer!");
    while (true);  // Halt here
  }
  delay(10);

  // Using Dual mode Wifi
  WiFi.mode(WIFI_AP_STA);

  // Start WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    SERIAL_DEBUG.print(".");
  }
  SERIAL_DEBUG.print("\nWiFi connected. IP Address: ");
  SERIAL_DEBUG.println(WiFi.localIP());

  delay(10);

  // Start AP (Access Point) mode
  WiFi.setSleep(false);  // Optional, prevents disconnects

  // Set static AP IP
  WiFi.softAPConfig(
    IPAddress(WIFI_DISPLAY_AP_IP_ADDR),
    IPAddress(WIFI_DISPLAY_AP_GW_ADDR),
    IPAddress(255, 255, 255, 0)
  );

  // Now start AP :  Channel 1, hidden SSID off, max 1 client
  bool apStarted = WiFi.softAP(WIFI_DISPLAY_AP_SSID, WIFI_DISPLAY_AP_PASSWORD, 1, 0, 1);
  if (!apStarted) {
    SERIAL_DEBUG.println("Failed to start Access Point!");
  } else {
    SERIAL_DEBUG.println("Access Point started");
  }

  // Print AP IP address (this returns the softAP IP)
  IPAddress apIp = WiFi.softAPIP();
  SERIAL_DEBUG.print("AP IP Address: ");
  SERIAL_DEBUG.println(apIp);

  // Serve HTML file
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (SPIFFS.exists("/WifiDisplay.html")) {
      request->send(SPIFFS, "/WifiDisplay.html", "text/html");
    } else {
      request->send(404, "text/plain", "File not found");
    }
  });
  server.serveStatic("/", SPIFFS, "/");

  // WebSocket events
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      SERIAL_DEBUG.println("client connected");
      clientConnected = true;
    } else if (type == WS_EVT_DISCONNECT) {
      SERIAL_DEBUG.println("client not connected");
      clientConnected = false;
    } else if (type == WS_EVT_DATA) {
      AwsFrameInfo *info = (AwsFrameInfo *)arg;
      if (info->final && info->index == 0 && info->len == len &&
          info->opcode == WS_TEXT) {
        data[len] = 0;  // null-terminate string

        // SERIAL_DEBUG.printf("WebSocket received: %s\n", (char *)data);

        // Parse JSON
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (error) {
          SERIAL_DEBUG.println("Failed to parse JSON");
          return;
        }

        touchDetected = true;

        if (doc["type"] == "TOUCH") {
          touchX = doc["x"];
          touchY = doc["y"];
          // SERIAL_DEBUG.printf("TOUCH: x=%d, y=%d\n", x, y);
          SERIAL_DEBUG.println("TOUCH");
        }
      }
    }
  });
  server.addHandler(&ws);

  server.begin();
  SERIAL_DEBUG.println("Server started");
  SERIAL_DEBUG.printf("Free heap: %u, PSRAM: %u\n", ESP.getFreeHeap(),
                      ESP.getFreePsram());

  SERIAL_TEENSY.write('D');
  SERIAL_TEENSY.flush();
  clientConnected = false;
}

//====================================================
// ====================== LOOP =======================
//====================================================
void loop() {
  
  checkFrameState();
  sendTouchCoord();
  yield();

}
