#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <PsychicHttp.h>
#include "line_tracing.h"
#include "motor_control.h"
#include "arm_claw.h"

// Globals
volatile bool line_Tracing_Enabled = false;
volatile bool arm_Enabled = false;

// Network Address Config
IPAddress AP_Addr = IPAddress(192, 168, 120, 1); // IP Address
IPAddress AP_Gate = IPAddress(192, 168, 120, 1); // Gateway
IPAddress AP_Mask = IPAddress(255, 255, 255, 0); // Mask

// Access Point Config
#define SSID "D0WD"            // AP SSID
#define PASSWORD "01234567890" // AP Password
#define AP_CHANNEL 11          // Channel
#define AP_MAX_CONNECTIONS 1   // AP Clients

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

  bool wifi_OK = WiFi.softAP(SSID, PASSWORD, AP_CHANNEL, false, AP_MAX_CONNECTIONS);
  Serial.printf("SoftAP Started: %s, IP: %s\n", wifi_OK ? "OK" : "FAIL", WiFi.softAPIP().toString().c_str());

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
  server.on("/", HTTP_GET, [](PsychicRequest *request) -> esp_err_t
            {
    String htmlContent = readFileToString("/index.html");
    if (htmlContent.length() == 0) return request->reply("404 Not Found");
    return request->reply(htmlContent.c_str()); });

  wsHandler.onFrame([](PsychicWebSocketRequest *r, httpd_ws_frame *frame) -> esp_err_t
                    {
  if (!frame || frame->len <= 0) return r->reply("ERR");

  // copy into local buffer and NUL-terminate (limit to 255 chars)
  int len = (int)frame->len;
  if (len > 255) len = 255;
  char buf[256];
  memcpy(buf, frame->payload, len);
  buf[len] = '\0';

  // trim trailing whitespace/newlines
  while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r' || buf[len-1] == ' ' || buf[len-1] == '\t')) {
    buf[--len] = '\0';
  }

  // Empty payload?
  if (len == 0) return r->reply("ERR");

  // === MODE:... handler (MODE:LINE or MODE:ARM, optionally with :1 or :0) ===
  if (strncmp(buf, "MODE:", 5) == 0) {
    char *modePart = buf + 5; // points after "MODE:"
    // look for second colon
    char *sep = strchr(modePart, ':');
    if (sep) {
      *sep = '\0';
      char *value = sep + 1;
      if (strcasecmp(modePart, "LINE") == 0) {
        if (value[0] == '1') { line_Tracing_Enabled = true; return r->reply("MODE:LINE:1"); }
        else { line_Tracing_Enabled = false; return r->reply("MODE:LINE:0"); }
      } else if (strcasecmp(modePart, "ARM") == 0) {
        if (value[0] == '1') { arm_Enabled = true; return r->reply("MODE:ARM:1"); }
        else { arm_Enabled = false; return r->reply("MODE:ARM:0"); }
      } else {
        return r->reply("ERR:MODE");
      }
    } else {
      // no trailing value: treat as enable
      if (strcasecmp(modePart, "LINE") == 0) { line_Tracing_Enabled = true; return r->reply("MODE:LINE:1"); }
      if (strcasecmp(modePart, "ARM") == 0)  { arm_Enabled = true; return r->reply("MODE:ARM:1"); }
      return r->reply("ERR:MODE");
    }
  }

  // === ARM:base,up,back,claw  ===
  if (strncmp(buf, "ARM:", 4) == 0) {
    char *p = buf + 4;
    int b=90, u=90, bk=90, c=90;
    int n = sscanf(p, "%d,%d,%d,%d", &b, &u, &bk, &c);
    if (n >= 4) {
      // clamp each to servo-safe ranges if desired (0..180)
      if (b < 0) b = 0; if (b > 180) b = 180;
      if (u < 0) u = 0; if (u > 180) u = 180;
      if (bk < 0) bk = 0; if (bk > 180) bk = 180;
      if (c < 0) c = 0; if (c > 180) c = 180;
      arm_set_targets((uint8_t)b, (uint8_t)u, (uint8_t)bk, (uint8_t)c);
      return r->reply("ARM:OK");
    } else {
      return r->reply("ERR:ARM");
    }
  }

  // === Joystick/manual drive: expect "left,right" (signed integers) ===
  // Example: "200,-150" or "-255,255"
  char *tok = strtok(buf, ",");
  if (!tok) return r->reply("ERR");

  // parse left
  long left = strtol(tok, nullptr, 10);
  tok = strtok(nullptr, ",");
  if (!tok) return r->reply("ERR");
  long right = strtol(tok, nullptr, 10);

  // clamp to -PWM_MAX_INPUT..+PWM_MAX_INPUT
  if (left > PWM_MAX_INPUT) left = PWM_MAX_INPUT;
  if (left < -PWM_MAX_INPUT) left = -PWM_MAX_INPUT;
  if (right > PWM_MAX_INPUT) right = PWM_MAX_INPUT;
  if (right < -PWM_MAX_INPUT) right = -PWM_MAX_INPUT;

  // If line tracing is enabled ignore manual joystick commands
  if (line_Tracing_Enabled) {
    return r->reply("IGNORED:LINE_MODE");
  }

  // Otherwise apply manual motor commands (works even if armEnabled)
  movement_Control(0, (int16_t)left);
  movement_Control(1, (int16_t)right);
  return r->reply("OK"); });

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
    Serial.println("LittleFS mount failed! HTML will not be served.");
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
  start_Arm_Controller();
  vTaskDelay(pdMS_TO_TICKS(250));
  start_WiFi();
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(20));
}
