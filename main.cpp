#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

const char* ssid = "ESP32-Flight_Control_System";
const char* password = "192837465";

const short int servo_1_pin = 1;
const short int servo_1_pin = 2;

Servo servo_1;
Servo servo_2;

AsyncWebServer server(80);
AsyncWebSocket websocket("/websocket")

void setup() {
    Serial.begin(115200);

    ESP32PWM::allocateTimer(0);
    servo_1.setPeriodHertz(50);
    servo_1.attach(servo_1_pin, 500, 2400);
    servo_1.write(90);

    servo_2.setPeriodHertz(50);
    servo_2.attach(servo_2_pin, 500, 2400);
    servo_2.write(90);

    WiFi.softAP(ssid, password);

    websocket.onEvent(onWebsocketEvent);
    server.addHandler(&websocket);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", index_html);
    });

    server.begin();
}

void loop() {
    websocket.cleanupClients();
}