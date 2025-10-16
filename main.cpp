#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <PsychicHttp.h>
#include <stdint.h>
#include <algorithm>

// Motor A pins (Left)
#define AIN1 GPIO_NUM_26
#define AIN2 GPIO_NUM_27
#define PWMA GPIO_NUM_25 // PWM pin

// Motor B pins (Right)
#define BIN1 GPIO_NUM_12
#define BIN2 GPIO_NUM_13
#define PWMB GPIO_NUM_14 // PWM pin

#define STBY GPIO_NUM_33 // Standby

// PWM Config (ESP32 LEDC)
const uint8_t PWM_CH_A = 0x00;
const uint8_t PWM_CH_B = 0x01;
const uint32_t PWM_FREQ = 16000;    // 16 kHz
const uint8_t PWM_RESO = 12;        // 12-bit -> 0-4095
const int32_t PWM_MAX_INPUT = 4095; // logical input range (0..4095)

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

const uint32_t DELAY_MS = 250; // 250 milliseconds

void init_Motors();
void movement_Control(uint8_t motor, int16_t speed);
void start_WiFi();
void http_Controller();

void init_Motors()
{

  pinMode(STBY, OUTPUT);   // Motor Driver STBY pin
  digitalWrite(STBY, LOW); // Disable until ready

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  // PWM setup (LEDC)
  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RESO);
  ledcAttachPin(PWMA, PWM_CH_A);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RESO);
  ledcAttachPin(PWMB, PWM_CH_B);

  // Enable motor driver
  digitalWrite(STBY, HIGH);

  vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
  start_WiFi();
}

void movement_Control(uint8_t motorIndex, int16_t motorSpeed)
{
  // Validate motor index (0 = left, 1 = right)
  if (motorIndex > 1)
  {
    Serial.printf("movement_Control: invalid motor %u\n", motorIndex);
    return;
  }

  // Clamp speed to allowed range
  if (motorSpeed > (int16_t)PWM_MAX_INPUT)
    motorSpeed = (int16_t)PWM_MAX_INPUT;
  if (motorSpeed < -(int16_t)PWM_MAX_INPUT)
    motorSpeed = -(int16_t)PWM_MAX_INPUT;

  // Get absolute value for magnitude
  int32_t absSpeed = (motorSpeed < 0) ? -(int32_t)motorSpeed : (int32_t)motorSpeed;

  // Calculate PWM duty cycle (scaled to resolution)
  const uint32_t maxDuty = (1UL << PWM_RESO) - 1; // e.g. 4095 for 12 bits
  uint32_t dutyCycle = (uint32_t)absSpeed * maxDuty / (uint32_t)PWM_MAX_INPUT;
  if (dutyCycle > maxDuty)
    dutyCycle = maxDuty;

  // If speed is zero, coast (both LOW)
  if (absSpeed == 0)
  {
    if (motorIndex == 0)
    {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW); // coast
      ledcWrite(PWM_CH_A, 0);
    }
    else
    {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW); // coast
      ledcWrite(PWM_CH_B, 0);
    }
    return;
  }

  // Set direction and speed
  if (motorIndex == 0)
  {
    // Left motor
    digitalWrite(AIN1, motorSpeed > 0 ? LOW : HIGH);
    digitalWrite(AIN2, motorSpeed > 0 ? HIGH : LOW);
    ledcWrite(PWM_CH_A, dutyCycle);
  }
  else
  {
    // Right motor
    digitalWrite(BIN1, motorSpeed > 0 ? HIGH : LOW);
    digitalWrite(BIN2, motorSpeed > 0 ? LOW : HIGH);
    ledcWrite(PWM_CH_B, dutyCycle);
  }
}

void start_WiFi()
{
  WiFi.disconnect(true);
  vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
  WiFi.mode(WIFI_OFF);
  vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
  WiFi.setSleep(WIFI_PS_NONE);
  vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
  WiFi.mode(WIFI_AP);
  vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
  WiFi.softAPConfig(AP_Addr, AP_Gate, AP_Mask);
  vTaskDelay(pdMS_TO_TICKS(DELAY_MS));

  bool ok = WiFi.softAP(SSID, PASSWORD, AP_CHANNEL, false, AP_MAX_CONNECTIONS);
  Serial.printf("SoftAP Started: %s, IP: %s\n", ok ? "OK" : "FAIL", WiFi.softAPIP().toString().c_str());

  // Start HTTP handlers
  vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
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

  // Serve index.html at root
  server.on("/", HTTP_GET, [](PsychicRequest *request) -> esp_err_t
            {
              String htmlContent = readFileToString("/index.html");
              if (htmlContent.length() == 0)
              {
                // 404 if file not found
                return request->reply("404 Not Found");
              }
              return request->reply(htmlContent.c_str()); });

  // WebSocket: expects messages like "leftSpeed,rightSpeed\n"
  wsHandler.onFrame([](PsychicWebSocketRequest *wsRequest, httpd_ws_frame *wsFrame) -> esp_err_t
                    {
    if (!wsFrame || wsFrame->len <= 0) return wsRequest->reply("ERR");

    // Copy payload to buffer, ensure NUL-terminated
    int payloadLength = std::min((int)wsFrame->len, 127);
    char payloadBuffer[128];
    memcpy(payloadBuffer, wsFrame->payload, payloadLength);
    payloadBuffer[payloadLength] = '\0';

    // Parse left and right speed values
    char *leftToken = strtok(payloadBuffer, ",");
    if (!leftToken) return wsRequest->reply("ERR");
    long leftSpeed = strtol(leftToken, nullptr, 10);

    char *rightToken = strtok(NULL, ",");
    if (!rightToken) return wsRequest->reply("ERR");
    long rightSpeed = strtol(rightToken, nullptr, 10);

    // Clamp values and control motors
    leftSpeed = std::max(std::min(leftSpeed, (long)MAX_INPUT), -(long)MAX_INPUT);
    rightSpeed = std::max(std::min(rightSpeed, (long)MAX_INPUT), -(long)MAX_INPUT);

    movement_Control(0, (int16_t)leftSpeed);
    movement_Control(1, (int16_t)rightSpeed);

    return wsRequest->reply("OK"); });

  server.on("/ws", &wsHandler);

  Serial.println("HTTP server started");
}

void setup()
{
  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  vTaskDelay(pdMS_TO_TICKS(DELAY_MS));

  // mount LittleFS (must succeed before serving files)
  if (!LittleFS.begin())
  {
    Serial.println("LittleFS mount failed! index.html will not be served.");
  }
  else
  {
    Serial.println("LittleFS mounted.");
  }

  vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
  init_Motors();
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
}
