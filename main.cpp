#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <PsychicHttp.h>
#include <stdint.h>
#include <algorithm>

#include "line_tracing.h"
#include "motor_control.h"

volatile bool lineTracingEnabled = false;

// Network Address Config
IPAddress AP_Addr = IPAddress(192, 168, 120, 1); // IP Address
IPAddress AP_Gate = IPAddress(192, 168, 120, 1); // Gateway
IPAddress AP_Mask = IPAddress(255, 255, 255, 0); // Mask

// Access Point Config
#define SSID "D0WD"            // AP SSID
#define PASSWORD "01234567890" // AP Password
#define AP_CHANNEL 11          // Channel
#define AP_MAX_CONNECTIONS 2   // AP Clients

PsychicHttpServer server;          // main server
PsychicWebSocketHandler wsHandler; // websocket handler

void setup();
void http_Controller();
void start_WiFi();
void loop();

void start_WiFi()
{
  WiFi.disconnect(true);
  vTaskDelay(pdMS_TO_TICKS(250));
  WiFi.mode(WIFI_OFF);
  vTaskDelay(pdMS_TO_TICKS(250));
  WiFi.setSleep(WIFI_PS_NONE);
  vTaskDelay(pdMS_TO_TICKS(250));
  WiFi.mode(WIFI_AP);
  vTaskDelay(pdMS_TO_TICKS(250));
  WiFi.softAPConfig(AP_Addr, AP_Gate, AP_Mask);
  vTaskDelay(pdMS_TO_TICKS(250));

  bool ok = WiFi.softAP(SSID, PASSWORD, AP_CHANNEL, false, AP_MAX_CONNECTIONS);
  Serial.printf("SoftAP Started: %s, IP: %s\n", ok ? "OK" : "FAIL", WiFi.softAPIP().toString().c_str());

  // Start HTTP handlers
  vTaskDelay(pdMS_TO_TICKS(250));
  http_Controller();
}

// Read a file from LittleFS into a String
String readFileToString(const char *filePath)
{
  if (!LittleFS.exists(filePath))
    return String();
  File file = LittleFS.open(filePath, "r");
  if (!file)
    return String();
  String fileContent;
  while (file.available())
    fileContent += (char)file.read();
  file.close();
  return fileContent;
}

void http_Controller()
{
  server.config.max_uri_handlers = 12;
  server.listen(80);

  // serve index.html
  server.on("/", HTTP_GET, [](PsychicRequest *r) -> esp_err_t
            {
    String html = readFileToString("/index.html");
    if (html.length() == 0) return r->reply("404 Not Found");
    return r->reply(html.c_str()); });

  // WebSocket handler: accepts "left,right" or "MODE:LINE"/"MODE:MANUAL"
  wsHandler.onFrame([](PsychicWebSocketRequest *r, httpd_ws_frame *frame) -> esp_err_t
                    {
    if (!frame || frame->len <= 0) return r->reply("ERR");

    int len = std::min((int)frame->len, 255);
    // keep a local buffer (NUL-terminated)
    char buf[256];
    memcpy(buf, frame->payload, len);
    buf[len] = '\0';

    // MODE messages: "MODE:LINE" or "MODE:MANUAL"
    if (strncmp(buf, "MODE:", 5) == 0) {
      char *mode = buf + 5;
      if (strcasecmp(mode, "LINE") == 0) {
        lineTracingEnabled = true;
        Serial.println("Mode -> LINE (line tracing enabled)");
        return r->reply("MODE:LINE:OK");
      } else {
        lineTracingEnabled = false;
        Serial.println("Mode -> MANUAL (joystick/manual enabled)");
        return r->reply("MODE:MANUAL:OK");
      }
    }

    // otherwise, expect "left,right" numeric values
    char *tok = strtok(buf, ",");
    if (!tok) return r->reply("ERR");
    long left = strtol(tok, nullptr, 10);
    tok = strtok(NULL, ",");
    if (!tok) return r->reply("ERR");
    long right = strtol(tok, nullptr, 10);

    // clamp to allowed range
    if (left > PWM_MAX_INPUT) left = PWM_MAX_INPUT;
    if (left < -PWM_MAX_INPUT) left = -PWM_MAX_INPUT;
    if (right > PWM_MAX_INPUT) right = PWM_MAX_INPUT;
    if (right < -PWM_MAX_INPUT) right = -PWM_MAX_INPUT;

    // Only apply manual commands when line-tracing is OFF
    if (!lineTracingEnabled) {
      movement_Control(0, (int16_t)left);
      movement_Control(1, (int16_t)right);
      return r->reply("OK");
    } else {
      // if in line-tracing mode, ignore manual commands
      return r->reply("IGNORED:LINE_MODE");
    } });

  server.on("/ws", &wsHandler);
  Serial.println("HTTP Server Started");
}

void setup()
{
  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  vTaskDelay(pdMS_TO_TICKS(250));

  // mount LittleFS (must succeed before serving files)
  if (!LittleFS.begin())
  {
    Serial.println("LittleFS mount failed! index.html will not be served.");
  }
  else
  {
    Serial.println("LittleFS mounted.");
  }

  vTaskDelay(pdMS_TO_TICKS(250));
  init_Motors();
  vTaskDelay(pdMS_TO_TICKS(250));
  start_Line_Tracer();
  vTaskDelay(pdMS_TO_TICKS(250));
  start_WiFi();
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(250));
}