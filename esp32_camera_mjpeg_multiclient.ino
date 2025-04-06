/*
  This is a simple MJPEG streaming webserver implemented for AI-Thinker ESP32-CAM
  and ESP-EYE modules.
  This is tested to work with VLC and Blynk video widget and can support up to 10
  simultaneously connected streaming clients.
  Simultaneous streaming is implemented with FreeRTOS tasks.

  Inspired by and based on this Instructable: $9 RTSP Video Streamer Using the ESP32-CAM Board
  (https://www.instructables.com/id/9-RTSP-Video-Streamer-Using-the-ESP32-CAM-Board/)

  Board: AI-Thinker ESP32-CAM or ESP-EYE
  Compile as:
   ESP32 Dev Module
   CPU Freq: 240
   Flash Freq: 80
   Flash mode: QIO
   Flash Size: 4Mb
   Patrition: Minimal SPIFFS
   PSRAM: Enabled

# Modifications on this build:
# web interface added for viewing, configuring camera settings and wifi settings 
# simple ftp server to upload the webserver html files to internal SPIFFS
            ( edit the FtpServerKey.h file and modify to #define DEFAULT_STORAGE_TYPE_ESP32 				STORAGE_SPIFFS )
# arduino OTA for firmware upload
# some performance tweaks
# use of external wifi antena is highly recommended for the esp32cam board
# set "Events Run On: core 0" and "Arduino Run On: core 0"
# used the board flash on gpio 4
# used the board led on gpio 33
# added external led on gpio 2
# added a button to enter AP mode and configure Wifi credentials which are then saved to SPIFFS and loaded on boot along with camera settings
            (button is connected though a 220 ohm series rezistor from gpio 13 to 14)
*/

// ESP32 has two cores: APPlication core and PROcess core (the one that runs ESP32 SDK stack)
#define APP_CPU 1
#define PRO_CPU 0
// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER
#define FLASH_PIN 4           // Define the GPIO pin for the flash LED
#define LED_PIN 33            // Define the GPIO pin for the internal LED
#define RED_LED_PIN 2         // Define the GPIO pin for the external RED LED
#define BUTTON_PIN_INPUT 13   // GPIO 13
#define BUTTON_PIN_OUTPUT 14  // GPIO 14

#include "src/OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <SimpleFTPServer.h>
#include "camera_pins.h"

unsigned long lastTime = 0;
unsigned long timerDelay = 50;
int led_state = LOW;
bool led_flag = false;

String ssid;
String password;
String hostname;

OV2640 cam;

WebServer server(80);
FtpServer ftpSrv;

// ===== rtos task handles =========================
// Streaming is implemented with 3 tasks:
TaskHandle_t tMjpeg = NULL;  // handles client connections to the webserver
//TaskHandle_t tCam;     // handles getting picture frames from the camera and storing them locally
TaskHandle_t camTaskHandle = NULL;
//TaskHandle_t tStream;  // actually streaming frames to all connected clients
TaskHandle_t streamTaskHandle = NULL;
// frameSync semaphore is used to prevent streaming buffer as it is replaced with the next frame
SemaphoreHandle_t frameSync = NULL;
// Queue stores currently connected clients to whom we are streaming
QueueHandle_t streamingClients;
// We will try to achieve 14 FPS frame rate
int FPS = 14;  // Default FPS value
// We will handle web client requests every 10 ms (100 Hz)
const int WSINTERVAL = 10;

// Function to handle camera settings
void handleCameraSettings() {
  if (server.hasArg("plain") == false) {  // Check if body received
    server.send(400, "application/json", "{\"status\":\"Invalid Request\"}");
    return;
  }

  String body = server.arg("plain");
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, body);

  if (error) {
    server.send(400, "application/json", "{\"status\":\"Invalid JSON\"}");
    return;
  }

  int quality = doc["quality"];
  int brightness = doc["brightness"];
  int contrast = doc["contrast"];
  int saturation = doc["saturation"];
  int specialEffect = doc["specialEffect"];
  int whiteBalance = doc["whiteBalance"];
  int awbGain = doc["awbGain"];
  int wbMode = doc["wbMode"];
  int hmirror = doc["hmirror"];
  int vflip = doc["vflip"];
  int colorbar = doc["colorbar"];
  int gammaCorrection = doc["gammaCorrection"];
  int aec2 = doc["aec2"];
  int aeLevel = doc["aeLevel"];
  int aecValue = doc["aecValue"];
  int exposureControl = doc["exposureControl"];
  int gainControl = doc["gainControl"];
  int agcGain = doc["agcGain"];
  int dcw = doc["dcw"];
  int fps = doc["fps"];  // Retrieve the FPS setting
  int led = doc["led"];  // Retrieve the led setting
  String resolution = doc["resolution"];
  // Apply camera settings
  sensor_t* s = esp_camera_sensor_get();
  if (resolution == "QVGA") {
    s->set_framesize(s, FRAMESIZE_QVGA);
  } else if (resolution == "VGA") {
    s->set_framesize(s, FRAMESIZE_VGA);
  } else if (resolution == "SVGA") {
    s->set_framesize(s, FRAMESIZE_SVGA);
  } else if (resolution == "XGA") {
    s->set_framesize(s, FRAMESIZE_XGA);
  } else if (resolution == "SXGA") {
    s->set_framesize(s, FRAMESIZE_SXGA);
  } else if (resolution == "UXGA") {
    s->set_framesize(s, FRAMESIZE_UXGA);
  }
  s->set_quality(s, quality);
  s->set_brightness(s, brightness);
  s->set_contrast(s, contrast);
  s->set_saturation(s, saturation);
  s->set_special_effect(s, specialEffect);
  s->set_whitebal(s, whiteBalance);          // Auto White Balance
  s->set_awb_gain(s, awbGain);               // AWB Gain
  s->set_wb_mode(s, wbMode);                 // White Balance Mode
  s->set_hmirror(s, hmirror);                // Horizontal Mirror
  s->set_vflip(s, vflip);                    // Vertical Flip
  s->set_colorbar(s, colorbar);              // Color Bar
  s->set_raw_gma(s, gammaCorrection);        // RAW Gamma Correction
  s->set_aec2(s, aec2);                      // AEC2
  s->set_ae_level(s, aeLevel);               // AE Level
  s->set_aec_value(s, aecValue);             // AEC Value
  s->set_exposure_ctrl(s, exposureControl);  // Exposure Control
  s->set_gain_ctrl(s, gainControl);          // Gain Control
  s->set_agc_gain(s, agcGain);               // AGC Gain
  s->set_dcw(s, dcw);                        // Downsize Mode
  FPS = fps;                                 // Update the global FPS variable
  if (led == 1) {
    digitalWrite(FLASH_PIN, HIGH);
  } else {
    digitalWrite(FLASH_PIN, LOW);
  }
  // Save settings to SPIFFS
  saveSettings(body);
  // Send response
  server.send(200, "application/json", "{\"status\":\"Settings applied\"}");
}

// Apply settings on reboot
void applySettings(const String& settings) {
  // Parse the settings and apply them to the camera
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, settings);

  if (error) {
    Serial.println("Failed to parse settings");
    return;
  }
  int quality = doc["quality"];
  int brightness = doc["brightness"];
  int contrast = doc["contrast"];
  int saturation = doc["saturation"];
  int specialEffect = doc["specialEffect"];
  int whiteBalance = doc["whiteBalance"];
  int awbGain = doc["awbGain"];
  int wbMode = doc["wbMode"];
  int hmirror = doc["hmirror"];
  int vflip = doc["vflip"];
  int colorbar = doc["colorbar"];
  int gammaCorrection = doc["gammaCorrection"];
  int aec2 = doc["aec2"];
  int aeLevel = doc["aeLevel"];
  int aecValue = doc["aecValue"];
  int exposureControl = doc["exposureControl"];
  int gainControl = doc["gainControl"];
  int agcGain = doc["agcGain"];
  int dcw = doc["dcw"];
  int fps = doc["fps"];  // Retrieve the FPS setting
  int led = doc["led"];  // Retrieve the led setting
  String resolution = doc["resolution"];
  // Apply camera settings
  sensor_t* s = esp_camera_sensor_get();
  if (resolution == "QVGA") {
    s->set_framesize(s, FRAMESIZE_QVGA);
  } else if (resolution == "VGA") {
    s->set_framesize(s, FRAMESIZE_VGA);
  } else if (resolution == "SVGA") {
    s->set_framesize(s, FRAMESIZE_SVGA);
  } else if (resolution == "XGA") {
    s->set_framesize(s, FRAMESIZE_XGA);
  } else if (resolution == "SXGA") {
    s->set_framesize(s, FRAMESIZE_SXGA);
  } else if (resolution == "UXGA") {
    s->set_framesize(s, FRAMESIZE_UXGA);
  }
  s->set_quality(s, quality);
  s->set_brightness(s, brightness);
  s->set_contrast(s, contrast);
  s->set_saturation(s, saturation);
  s->set_special_effect(s, specialEffect);
  s->set_whitebal(s, whiteBalance);          // Auto White Balance
  s->set_awb_gain(s, awbGain);               // AWB Gain
  s->set_wb_mode(s, wbMode);                 // White Balance Mode
  s->set_hmirror(s, hmirror);                // Horizontal Mirror
  s->set_vflip(s, vflip);                    // Vertical Flip
  s->set_colorbar(s, colorbar);              // Color Bar
  s->set_raw_gma(s, gammaCorrection);        // RAW Gamma Correction
  s->set_aec2(s, aec2);                      // AEC2
  s->set_ae_level(s, aeLevel);               // AE Level
  s->set_aec_value(s, aecValue);             // AEC Value
  s->set_exposure_ctrl(s, exposureControl);  // Exposure Control
  s->set_gain_ctrl(s, gainControl);          // Gain Control
  s->set_agc_gain(s, agcGain);               // AGC Gain
  s->set_dcw(s, dcw);                        // Downsize Mode
  FPS = fps;                                 // Update the global FPS variable
  if (led == 1) {
    digitalWrite(FLASH_PIN, HIGH);
  } else {
    digitalWrite(FLASH_PIN, LOW);
  }
}

// ======== Server Connection Handler Task ==========================
void mjpegCB(void* pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(WSINTERVAL);

  // Creating frame synchronization semaphore and initializing it
  frameSync = xSemaphoreCreateBinary();
  xSemaphoreGive(frameSync);

  // Creating a queue to track all connected clients
  streamingClients = xQueueCreate(10, sizeof(WiFiClient*));

  //=== setup section  ==================
  //  Creating RTOS task for grabbing frames from the camera
  xTaskCreatePinnedToCore(
    camCB,           // callback
    "cam",           // name
    6 * 1024,        // stack size
    NULL,            // parameters
    6,               // priority
    &camTaskHandle,  // RTOS task handle
    APP_CPU);        // core

  //  Creating task to push the stream to all connected clients
  xTaskCreatePinnedToCore(
    streamCB,
    "strmCB",
    6 * 1024,
    NULL,               // parameters
    6,                  // priority
    &streamTaskHandle,  // RTOS task handle
    PRO_CPU);           // core

  //  Registering webserver handling routines
  server.on("/stream", HTTP_GET, handleJPGSstream);
  server.on("/jpg", HTTP_GET, handleJPG);
  server.on("/", HTTP_GET, []() {
    File file = SPIFFS.open("/index.html", "r");
    server.streamFile(file, "text/html");
    file.close();
  });
  server.on("/cam_settings", HTTP_GET, []() {
    File file = SPIFFS.open("/cam_settings.html", "r");
    server.streamFile(file, "text/html");
    file.close();
  });
  server.on("/wifi_settings", HTTP_GET, []() {
    File file = SPIFFS.open("/wifi_settings.html", "r");
    server.streamFile(file, "text/html");
    file.close();
  });
  server.on("/getSettings", HTTP_GET, []() {
    String settings = loadSettings();
    server.send(200, "application/json", settings);
  });
  server.on("/saveSettings", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");
      File file = SPIFFS.open("/wifi_settings.json", FILE_WRITE);
      if (!file) {
        server.send(500, "application/json", "{\"message\":\"Failed to save settings\"}");
        return;
      }
      file.print(body);
      file.close();
      server.send(200, "application/json", "{\"message\":\"Settings saved successfully\"}");
    }
  });
  server.on("/reboot", HTTP_POST, []() {
    server.send(200, "application/json", "{\"message\":\"Rebooting...\"}");
    // Delete the ap_mode.flag file
    if (SPIFFS.exists("/ap_mode.flag")) {
      SPIFFS.remove("/ap_mode.flag");
    }
    delay(1000);
    ESP.restart();
  });
  server.on("/rebootAp", HTTP_POST, []() {
    File file = SPIFFS.open("/ap_mode.flag", FILE_WRITE);
    if (!file) {
      server.send(500, "application/json", "{\"message\":\"Failed to set AP mode flag\"}");
      return;
    }
    file.close();
    server.send(200, "application/json", "{\"message\":\"Rebooting into AP mode...\"}");
    delay(1000);
    ESP.restart();
  });
  // Serve static files
  server.serveStatic("/cam_styles.css", SPIFFS, "/cam_styles.css");
  server.serveStatic("/cam_script.js", SPIFFS, "/cam_script.js");
  server.serveStatic("/wifi_styles.css", SPIFFS, "/wifi_styles.css");
  server.serveStatic("/wifi_script.js", SPIFFS, "/wifi_script.js");
  server.serveStatic("/index_script.js", SPIFFS, "/index_script.js");
  server.on("/settings", HTTP_POST, handleCameraSettings);
  server.onNotFound(handleNotFound);

  //  Starting webserver
  server.begin();
  //=== loop() section  ===================
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    server.handleClient();
    //  After every server client handling request, we let other tasks run and then pause
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Commonly used variables:
volatile size_t camSize;  // size of the current frame, byte
volatile char* camBuf;    // pointer to the current frame

// ==== RTOS task to grab frames from the camera =========================
void camCB(void* pvParameters) {
  TickType_t xLastWakeTime;
  //  A running interval associated with currently desired frame rate
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / FPS);
  // Mutex for the critical section of swithing the active frames around
  portMUX_TYPE xSemaphore = portMUX_INITIALIZER_UNLOCKED;
  //  Pointers to the 2 frames, their respective sizes and index of the current frame
  char* fbs[2] = { NULL, NULL };
  size_t fSize[2] = { 0, 0 };
  int ifb = 0;
  //=== loop() section  ===================
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    //  Grab a frame from the camera and query its size
    cam.run();
    size_t s = cam.getSize();
    //  If frame size is more that we have previously allocated - request  125% of the current frame space
    if (s > fSize[ifb]) {
      fSize[ifb] = s * 4 / 3;
      fbs[ifb] = allocateMemory(fbs[ifb], fSize[ifb]);
    }
    //  Copy current frame into local buffer
    char* b = (char*)cam.getfb();
    memcpy(fbs[ifb], b, s);
    //  Let other tasks run and wait until the end of the current frame rate interval (if any time left)
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    //  Only switch frames around if no frame is currently being streamed to a client
    //  Wait on a semaphore until client operation completes
    xSemaphoreTake(frameSync, portMAX_DELAY);
    //  Do not allow interrupts while switching the current frame
    portENTER_CRITICAL(&xSemaphore);
    camBuf = fbs[ifb];
    camSize = s;
    ifb++;
    ifb &= 1;  // this should produce 1, 0, 1, 0, 1 ... sequence
    portEXIT_CRITICAL(&xSemaphore);
    //  Let anyone waiting for a frame know that the frame is ready
    xSemaphoreGive(frameSync);
    //  Technically only needed once: let the streaming task know that we have at least one frame
    //  and it could start sending frames to the clients, if any
    xTaskNotifyGive(streamTaskHandle);
    //  Immediately let other (streaming) tasks run
    taskYIELD();
    //  If streaming task has suspended itself (no active clients to stream to)
    //  there is no need to grab frames from the camera. We can save some juice
    //  by suspedning the tasks
    if (eTaskGetState(streamTaskHandle) == eSuspended) {
      vTaskSuspend(NULL);  // passing NULL means "suspend yourself"
    }
  }
}

// ==== Memory allocator that takes advantage of PSRAM if present =======================
char* allocateMemory(char* aPtr, size_t aSize) {
  //  Since current buffer is too smal, free it
  if (aPtr != NULL) free(aPtr);
  size_t freeHeap = ESP.getFreeHeap();
  char* ptr = NULL;
  // If memory requested is more than 2/3 of the currently free heap, try PSRAM immediately
  if (aSize > freeHeap * 2 / 3) {
    if (psramFound() && ESP.getFreePsram() > aSize) {
      ptr = (char*)ps_malloc(aSize);
    }
  } else {
    //  Enough free heap - let's try allocating fast RAM as a buffer
    ptr = (char*)malloc(aSize);

    //  If allocation on the heap failed, let's give PSRAM one more chance:
    if (ptr == NULL && psramFound() && ESP.getFreePsram() > aSize) {
      ptr = (char*)ps_malloc(aSize);
    }
  }
  // Finally, if the memory pointer is NULL, we were not able to allocate any memory, and that is a terminal condition.
  if (ptr == NULL) {
    ESP.restart();
  }
  return ptr;
}

// ==== STREAMING ======================================================
const char HEADER[] = "HTTP/1.1 200 OK\r\n"
                      "Access-Control-Allow-Origin: *\r\n"
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);


// ==== Handle connection request from clients ===============================
void handleJPGSstream(void) {
  //  Can only acommodate 10 clients. The limit is a default for WiFi connections
  if (!uxQueueSpacesAvailable(streamingClients)) return;
  //  Create a new WiFi Client object to keep track of this one
  WiFiClient* client = new WiFiClient();
  *client = server.client();
  //  Immediately send this client a header
  client->write(HEADER, hdrLen);
  client->write(BOUNDARY, bdrLen);
  // Push the client to the streaming queue
  xQueueSend(streamingClients, (void*)&client, 0);
  // Wake up streaming tasks, if they were previously suspended:
  if (eTaskGetState(camTaskHandle) == eSuspended) vTaskResume(camTaskHandle);
  if (eTaskGetState(streamTaskHandle) == eSuspended) vTaskResume(streamTaskHandle);
}


// ==== Actually stream content to all connected clients ========================
void streamCB(void* pvParameters) {
  char buf[16];
  TickType_t xLastWakeTime;
  TickType_t xFrequency;

  //  Wait until the first frame is captured and there is something to send
  //  to clients
  ulTaskNotifyTake(pdTRUE,         /* Clear the notification value before exiting. */
                   portMAX_DELAY); /* Block indefinitely. */
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // Default assumption we are running according to the FPS
    xFrequency = pdMS_TO_TICKS(1000 / FPS);
    //  Only bother to send anything if there is someone watching
    UBaseType_t activeClients = uxQueueMessagesWaiting(streamingClients);
    if (activeClients) {
      // Adjust the period to the number of connected clients
      xFrequency /= activeClients;
      //  Since we are sending the same frame to everyone,
      //  pop a client from the the front of the queue
      WiFiClient* client;
      xQueueReceive(streamingClients, (void*)&client, 0);

      //  Check if this client is still connected.

      if (!client->connected()) {
        //  delete this client reference if s/he has disconnected
        //  and don't put it back on the queue anymore. Bye!
        delete client;
        led_flag = false;
      } else {
        led_flag = true;
        //  Ok. This is an actively connected client.
        //  Let's grab a semaphore to prevent frame changes while we
        //  are serving this frame
        xSemaphoreTake(frameSync, portMAX_DELAY);

        //client->write(HEADER, hdrLen);
        //client->write(BOUNDARY, bdrLen);
        client->write(CTNTTYPE, cntLen);
        sprintf(buf, "%d\r\n\r\n", camSize);
        client->write(buf, strlen(buf));
        client->write((char*)camBuf, (size_t)camSize);
        client->write(BOUNDARY, bdrLen);

        // Since this client is still connected, push it to the end
        // of the queue for further processing
        xQueueSend(streamingClients, (void*)&client, 0);

        //  The frame has been served. Release the semaphore and let other tasks run.
        //  If there is a frame switch ready, it will happen now in between frames
        xSemaphoreGive(frameSync);
        taskYIELD();
      }
    } else {
      //  Since there are no connected clients, there is no reason to waste battery running
      vTaskSuspend(NULL);
    }
    //  Let other tasks run after serving every client
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

const char JHEADER[] = "HTTP/1.1 200 OK\r\n"
                       "Content-disposition: inline; filename=capture.jpg\r\n"
                       "Content-type: image/jpeg\r\n\r\n";
const int jhdLen = strlen(JHEADER);

// ==== Serve up one JPEG frame =============================================
void handleJPG(void) {
  WiFiClient client = server.client();

  if (!client.connected()) return;
  digitalWrite(FLASH_PIN, HIGH);  // flash on for capture jpg
  cam.run();
  client.write(JHEADER, jhdLen);
  client.write((char*)cam.getfb(), cam.getSize());
  digitalWrite(FLASH_PIN, LOW);
}
// ==== Handle invalid URL requests ============================================
void handleNotFound() {
  String message = "Server is running!\n\n";
  message += "input address is wrong!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text / plain", message);
}
// ==== SETUP method ==================================================================
void setup() {
  // Setup Serial connection:
  Serial.begin(115200);
  delay(1000);  // wait for a second to let Serial connect

  // Initialize LED pins
  pinMode(FLASH_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // internal led pin is inverted

  pinMode(BUTTON_PIN_INPUT, INPUT_PULLUP);  // Set GPIO 13 as input
  digitalWrite(BUTTON_PIN_INPUT, HIGH);     // Initialize GPIO 13 as high

  pinMode(BUTTON_PIN_OUTPUT, OUTPUT);    // Set GPIO 14 as output
  digitalWrite(BUTTON_PIN_OUTPUT, LOW);  // Initialize GPIO 14 as LOW


  // Attach the interrupt to pin 13, triggering on FALLING (HIGH to LOW transition)
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_INPUT), handleButtonInterrupt, FALLING);

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
    return;
  }
  // Configure the camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  //config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; /*!< Fills buffers when they are empty. Less resources but first 'fb_count' frames might be old */
  config.grab_mode = CAMERA_GRAB_LATEST; /*!< Except when 1 frame buffer is used, queue will always contain the last 'fb_count' frames */
  // Frame parameters: pick one
  //  config.frame_size = FRAMESIZE_UXGA;
  //  config.frame_size = FRAMESIZE_SVGA;
  //  config.frame_size = FRAMESIZE_QVGA;
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 20;
  config.fb_count = 1;

  if (cam.init(config) != ESP_OK) {
    Serial.println("Error initializing the camera");
    delay(10000);
    ESP.restart();
  }
  sensor_t* s = esp_camera_sensor_get();
  s->set_brightness(s, 0);  // -2 to 2
  s->set_contrast(s, 0);    // -2 to 2
  s->set_saturation(s, 0);  // -2 to 2
  //s->set_sharpness(s, 2);                   // -2 to 2 //unsuported
  s->set_special_effect(s, 0);              // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);                    // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);                    // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);                     // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);               // 0 = disable , 1 = enable
  s->set_aec2(s, 1);                        // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);                    // -2 to 2
  s->set_aec_value(s, 600);                 // 0 to 1200
  s->set_gain_ctrl(s, 1);                   // 0 = disable , 1 = enable
  s->set_agc_gain(s, 15);                   // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 1);                         // 0 = disable , 1 = enable
  s->set_wpc(s, 1);                         // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);                     // 0 = disable , 1 = enable
  s->set_lenc(s, 1);                        // 0 = disable , 1 = enable
  s->set_hmirror(s, 1);                     // 0 = disable , 1 = enable
  s->set_vflip(s, 1);                       // 0 = disable , 1 = enable
  s->set_dcw(s, 1);                         // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);                    // 0 = disable , 1 = enable
  //s->set_denoise(s, 2);                     //unsuported
  delay(1000);

  createDefault_cam_Settings();  // Create default cam settings if they don't exist
  // Load and apply camera settings on startup
  String settings = loadSettings();
  if (settings != "") {
    applySettings(settings);
  }
  //  Configure and connect to WiFi
  createDefault_wifi_Settings();  // Create default  wifi settings if they don't exist
  // Check if we should boot into AP mode
  if (shouldBootAPMode()) {
    setupAPMode();
  } else {
    load_wifi_Settings();
    wifi_Connect();
  }
  // Start mainstreaming RTOS task
  xTaskCreatePinnedToCore(
    mjpegCB,
    "mjpeg",
    4 * 1024,
    NULL,
    6,
    &tMjpeg,
    PRO_CPU);

  ftpSrv.begin("user", "pasw");  //username, password for ftp.  set ports in ESP8266FtpServer.h  (default 21, 50009 for PASV)
  Serial.println("FTP Server Ready");
  ota_setup();
}

volatile bool buttonPressed = false;  // Flag to indicate button press

// ISR to set the flag
void handleButtonInterrupt() {
  buttonPressed = true;  // Set the flag when the interrupt triggers
}

void loop() {
  unsigned long currentMillis = millis();
  // Handle LED blinking logic, signal if there are any clients watching
  if (((currentMillis - lastTime) >= timerDelay) && (led_flag)) {
    lastTime = currentMillis;
    led_state = !led_state;  // Toggle LED state
    digitalWrite(LED_PIN, led_state);
    digitalWrite(RED_LED_PIN, led_state);
  } else if (!led_flag && led_state == HIGH) {
    led_state = LOW;
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
  }

  if (buttonPressed) {
    buttonPressed = false;  // Clear the flag
    File file = SPIFFS.open("/ap_mode.flag", FILE_WRITE);
    if (!file) {
      Serial.println("Failed to create file");
      return;
    }
    file.close();
    delay(1000);
    ESP.restart();
  }

  ArduinoOTA.handle();  // Check for OTA updates
  ftpSrv.handleFTP();
  taskYIELD();
  vTaskDelay(pdMS_TO_TICKS(1));
}

void setupAPMode() {
  WiFi.softAP("ESP32CAM-AP", "12345678");
  // Set custom IP address
  IPAddress local_IP(192, 168, 0, 1);  // Change to your desired IP address
  IPAddress gateway(192, 168, 0, 1);   // Change to your desired gateway address
  IPAddress subnet(255, 255, 255, 0);  // Subnet mask
  WiFi.softAPConfig(local_IP, gateway, subnet);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

// Function to save settings to SPIFFS
void saveSettings(const String& settings) {
  File file = SPIFFS.open("/cam_settings.json", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.print(settings);
  file.close();
  Serial.println("Cam Settings saved to SPIFFS");
}

// Function to load settings from SPIFFS
String loadSettings() {
  File file = SPIFFS.open("/cam_settings.json");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return "";
  }

  String settings = file.readString();
  file.close();
  Serial.println("Cam Settings loaded from SPIFFS");
  return settings;
}

void createDefault_wifi_Settings() {
  if (!SPIFFS.exists("/wifi_settings.json")) {
    File file = SPIFFS.open("/wifi_settings.json", FILE_WRITE);
    if (!file) {
      Serial.println("Failed to create wifi settings file");
      return;
    }
    DynamicJsonDocument doc(1024);
    doc["ssid"] = "free-wifi";
    doc["password"] = "default";
    doc["hostname"] = "esp32cam";
    serializeJson(doc, file);
    file.close();
    Serial.println("Default wifi settings file created");
  }
}

void createDefault_cam_Settings() {
  if (!SPIFFS.exists("/cam_settings.json")) {
    File file = SPIFFS.open("/cam_settings.json", FILE_WRITE);
    if (!file) {
      Serial.println("Failed to create cam settings file");
      return;
    }
    DynamicJsonDocument doc(1024);
    doc["quality"] = "20";
    doc["fps"] = "14";
    doc["brightness"] = "0";
    doc["contrast"] = "0";
    doc["saturation"] = "0";
    doc["specialEffect"] = "0";
    doc["whiteBalance"] = "1";
    doc["awbGain"] = "1";
    doc["wbMode"] = "0";
    doc["hmirror"] = "0";
    doc["vflip"] = "0";
    doc["colorbar"] = "0";
    doc["gammaCorrection"] = "1";
    doc["aec2"] = "1";
    doc["aeLevel"] = "0";
    doc["aecValue"] = "600";
    doc["exposureControl"] = "1";
    doc["gainControl"] = "1";
    doc["agcGain"] = "15";
    doc["dcw"] = "1";
    doc["led"] = "0";
    doc["resolution"] = "VGA";
    serializeJson(doc, file);
    file.close();
    Serial.println("Default cam settings file created");
  }
}

void wifi_Connect() {
  IPAddress ip;
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);
  WiFi.setHostname(hostname.c_str());
  WiFi.begin(ssid.c_str(), password.c_str());
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.print("Stream Link: http://");
  Serial.println(ip);
}

bool shouldBootAPMode() {
  if (SPIFFS.exists("/ap_mode.flag")) {
    return true;
  } else {
    return false;
  }
}

void load_wifi_Settings() {
  if (!SPIFFS.exists("/wifi_settings.json")) {
    Serial.println("wifi_settings.json file does not exist");
    return;
  }

  File file = SPIFFS.open("/wifi_settings.json", FILE_READ);
  if (!file) {
    Serial.println("Failed to open wifi_settings.json file for reading");
    return;
  }

  size_t size = file.size();
  if (size == 0) {
    Serial.println("wifi_settings.json file is empty");
    file.close();
    return;
  }

  std::unique_ptr<char[]> buf(new char[size + 1]);
  file.readBytes(buf.get(), size);
  buf[size] = '\0';  // Null-terminate the buffer
  file.close();
  Serial.println("wifi_settings.json content:");
  Serial.println(buf.get());

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, buf.get());
  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.f_str());
    return;
  }
  ssid = doc["ssid"].as<String>();
  password = doc["password"].as<String>();
  hostname = doc["hostname"].as<String>();

  Serial.println("WiFi settings loaded from SPIFFS");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("Hostname: ");
  Serial.println(hostname);
}

// OTA setup
void ota_setup() {
  ArduinoOTA.setHostname("ESPCAM_OTA");
  ArduinoOTA.setPassword("OTA-pasw");  // Set a strong password for OTA updates
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    delay(30000);
    ESP.restart();
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
  ArduinoOTA.begin();
  Serial.println("OTA updates Ready");
}
