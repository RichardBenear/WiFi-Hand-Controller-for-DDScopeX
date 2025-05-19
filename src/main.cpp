// ===============================================
// ======== WiFi Screen Mirror for DDScope =======
// ===============================================
#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <esp_heap_caps.h>  // Needed for PSRAM allocation

#include "SPIFFS.h"
#include "esp32/spiram.h"

#define ACK 0x06
#define NACK 0x15
#define SERIAL_TEENSY Serial  // USB
#define SERIAL_DEBUG Serial0

#define FRAME_TYPE_RAW 0x00
#define FRAME_TYPE_RLE 0x01
#define FRAME_TYPE_DEF 0x04

#define FRAME_MAX_SIZE 307200  // For uncompressed 320x480 RGB565
#define STATE_MACHINE_TIMEOUT 5000
#define IP_ADDR_UPDATE_RATE 3000
#define HEADER_SIZE 5  // 1 byte type + 4 bytes size
#define FRAME_TOTAL_SIZE (FRAME_MAX_SIZE + HEADER_SIZE)

#define WIFI_SSID "your ssid"
#define WIFI_PASSWORD "your passwork"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");  // WebSocket endpoint

bool clientConnected = false;
volatile bool isRaw = false;
volatile bool isRLECompressed = false;
volatile bool isDeflateCompressed = false;
volatile bool queueLocked = false;

uint32_t frameSize = 0;
size_t bytesRead = 0;
unsigned long lastSendTime = 0;
bool ipAckedByTeensy = false;
bool xSentToTeensy = false;
uint8_t *frameBuffer = nullptr;
bool frameReady = false;
uint8_t *fullFrameBuffer = nullptr;
size_t sz = 0;
uint8_t *buf = nullptr;
uint8_t header[5];

enum EspCommState {
  WAIT_FOR_HELLO,
  WAIT_FOR_IP_REQUEST,
  WAIT_FOR_IP_ACK,
  WAIT_FOR_CLIENT_CONNECTED,
  WAIT_FOR_CLIENT_CONNECTED_ACK,
  ERROR
};

EspCommState espState = WAIT_FOR_HELLO;
//=============================================

// Send the IP address
// Not very elegant, but simple works too
void sendIpToTeensy() {
  IPAddress ip = WiFi.localIP();
  SERIAL_TEENSY.print('I');
  SERIAL_TEENSY.print('P');
  SERIAL_TEENSY.print(':');
  SERIAL_TEENSY.print(ip[0]);
  SERIAL_TEENSY.print(".");
  SERIAL_TEENSY.print(ip[1]);
  SERIAL_TEENSY.print(".");
  SERIAL_TEENSY.print(ip[2]);
  SERIAL_TEENSY.print(".");
  SERIAL_TEENSY.println(ip[3]);  // newline-terminated
  SERIAL_TEENSY.flush();
  // SERIAL_DEBUG.println("Sent IP");
}

// =============================================
// =================== SETUP ===================
// =============================================
void setup() {
  delay(2000);

  SERIAL_DEBUG.begin(115200);
  delay(100);
  SERIAL_DEBUG.println("Debug port started");

  // USB port begin...probably already initialized, but?
  SERIAL_TEENSY.begin(1000000);
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

  // Start WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    SERIAL_DEBUG.print(".");
  }
  SERIAL_DEBUG.print("\nWiFi connected. IP Address: ");
  SERIAL_DEBUG.println(WiFi.localIP());

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

        //SERIAL_DEBUG.printf("WebSocket received: %s\n", (char *)data);

        // Parse JSON
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (error) {
          SERIAL_DEBUG.println("Failed to parse JSON");
          return;
        }

        if (doc["type"] == "TOUCH") {
          int x = doc["x"];
          int y = doc["y"];
          SERIAL_DEBUG.printf("TOUCH: x=%d, y=%d\n", x, y);

          // Send to Teensy over USB CDC
          SERIAL_TEENSY.write('T');                  // Touch event marker
          SERIAL_TEENSY.write((uint8_t)(x >> 8));    // x high byte
          SERIAL_TEENSY.write((uint8_t)(x & 0xFF));  // x low byte
          SERIAL_TEENSY.write((uint8_t)(y >> 8));    // y high byte
          SERIAL_TEENSY.write((uint8_t)(y & 0xFF));  // y low byte
          ws.textAll("ack");  // Send acknowledgment back to client
        }
      }
    }
  });
  server.addHandler(&ws);

  server.begin();
  SERIAL_DEBUG.println("Server started");
  SERIAL_DEBUG.printf("Free heap: %u, PSRAM: %u\n", ESP.getFreeHeap(),
                      ESP.getFreePsram());

  SERIAL_TEENSY.write('R');  // ESP out of reset
  SERIAL_TEENSY.flush();
}

// The metadata byte sent to the WebSocket client:
// 0x00 = Raw
// 0x01 = Compressed RLE
// 0x04 = Compressed Deflate
bool processingFrame = false;
static unsigned long lastByteTime = 0;
//====================================================
// ====================== LOOP =======================
//====================================================
void loop() {
  static bool lastClientConnected = false;

  // Detect Web Client Disconnect
  if (!lastClientConnected && clientConnected) {
    SERIAL_DEBUG.println("[ESP32] Client disconnected — sending 'R' to Teensy");
    SERIAL_TEENSY.write('R');
    SERIAL_TEENSY.flush();

    // Reset state machine too?
    espState = WAIT_FOR_HELLO;  // Optional: restart protocol
    processingFrame = false;
  }
  lastClientConnected = clientConnected;

  // Teensy4.1 and ESP32-S3 both have handshake State Machines
  // ============== State Flow ===============================
  // Command	         Byte	     Meaning
  // HELLO	           'H'	   Teensy says "I'm alive"
  // HELLO_ACK	       'A'	   ESP32 says "I hear you"
  // IP_START	         'I'	   ESP32 sends IP address (e.g., I192.168.1.55\n)
  // CLIENT_CONNECTED  'X'	   ESP says: Client is connected
  // ACK	             'K'	   Teensy acknowledgement
  // RESET	           'R'	   From ESP: Reset whole state machine
  // ==> BOTH are READY, Teensy can send frames
  // RESET ESP32 FRAME 'Z'     From Teensy to ESP32 
  if (!processingFrame && SERIAL_TEENSY.available()) {
    lastByteTime = millis();
    char incoming = SERIAL_TEENSY.read();
     SERIAL_DEBUG.print(incoming);
    switch (espState) {
      case WAIT_FOR_HELLO:
        if (incoming == 'H') {
          SERIAL_DEBUG.println("[ESP32] HELLO received. Sending ACK...");
          SERIAL_TEENSY.write('A');  // HELLO_ACK
          SERIAL_TEENSY.flush();
          espState = WAIT_FOR_IP_REQUEST;
        }
        break;

      case WAIT_FOR_IP_REQUEST:
        if (incoming == 'I') {
          SERIAL_DEBUG.println("[ESP32] IP REQUESTED");
          sendIpToTeensy();
          espState = WAIT_FOR_IP_ACK;
        }
        break;

      case WAIT_FOR_IP_ACK:
        if (incoming == 'K') {
          SERIAL_DEBUG.println("[ESP32] IP ACK received");
          if (clientConnected) {
            SERIAL_DEBUG.println("[ESP32] Web Client connected");
            SERIAL_TEENSY.write('X');  // Client Connected
            SERIAL_TEENSY.flush();
            delay(20);
            SERIAL_DEBUG.println("[ESP32] Sending X");
            espState = WAIT_FOR_CLIENT_CONNECTED_ACK;
          } else {
            SERIAL_DEBUG.println("[ESP32] Client not connected");
            delay(1000);
            sendIpToTeensy();
            espState = WAIT_FOR_IP_ACK;
          }
        } else if (incoming == 'H') {
          espState = WAIT_FOR_HELLO;
        }
        break;

      case WAIT_FOR_CLIENT_CONNECTED_ACK:
        if (incoming == 'K') {
          SERIAL_DEBUG.println("[ESP32] Client Connected ACK received");
          espState = WAIT_FOR_CLIENT_CONNECTED;
          processingFrame = true;
        }
        break;

      default:
        break;
    }
  }

  static unsigned long lastByteTime = 0;

  // Check for reset frame command
  if (processingFrame && SERIAL_TEENSY.available()) {
    char peek = SERIAL_TEENSY.peek();
    if (peek == 'Z') {
      SERIAL_TEENSY.read();  // Consume it
      //SERIAL_DEBUG.println("Received 'Z' reset signal from Teensy");

      // Reset state machine or any internal flags as needed
      frameSize = 0;
      bytesRead = 0;
      isRaw = isRLECompressed = isDeflateCompressed = false;
      return;  // Skip processing this cycle
    }
  }

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
      processingFrame = false;
      espState = WAIT_FOR_HELLO;
      return;
    }
    SERIAL_TEENSY.write(ACK);
    SERIAL_TEENSY.flush();

    // Wait until full frame arrives
    size_t bytesToRead = frameSize;
    uint8_t *payloadPtr = fullFrameBuffer + HEADER_SIZE;
    unsigned long startWait = millis();

    while (bytesToRead > 0) {
      int availableBytes = SERIAL_TEENSY.available();
      if (availableBytes > 0) {
        int chunk = min((int)bytesToRead, availableBytes);
        int bytesReadNow = SERIAL_TEENSY.readBytes(payloadPtr, chunk);
        payloadPtr += bytesReadNow;
        bytesToRead -= bytesReadNow;
      } else {
        if (millis() - startWait > 2000) {
          SERIAL_DEBUG.println("Timeout waiting for frame payload");
          processingFrame = false;
          espState = WAIT_FOR_HELLO;
          return;
        }
        delay(1);  // yield to USB stack, but doesn't hurt anymore
      }
    }

    // Send over WebSocket
    ws.binaryAll(fullFrameBuffer, HEADER_SIZE + frameSize);
  }
}