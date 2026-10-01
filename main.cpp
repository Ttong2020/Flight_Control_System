#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

const char* ssid = "ESP32-Flight_Control_System";
const char* password = "192837465";

const short int servo_1_pin = 18;
const short int servo_2_pin = 19;
const short int esc_pin = 21;

const char html_code[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang = "en-GB">
    <head>
        <title>
        RC Plane Flight Control System
        </title>
    </head>
    <body>
        <h1 style = "text-align: center;">
        Flight Control System
        </h1>
        <button onclick = "servo_1_command('left')">
        Left
        </button>
        <button onclick = "servo_1_command('right')">
        Right
        </button>
        <button onclick = "servo_2_command('up')">
        Up
        </button>
        <button onclick = "servo_2_command('down')">
        Down
        </button>
        <button onclick = "reset_servos_command()">
        Reset
        </button>
        <span id = "span_servo_1_angle">
        90
        </span>
        <span id = "span_servo_2_angle">
        90
        </span>
        <label for = "slider_throttle_percentage">
        Throttle:
        </label>
        <input type = "range" id = "slider_throttle_percentage" min = "0" max = "100" value = "0" oninput = "throttle_command(this.value)">
        <span id = "span_throttle_percentage">
        0
        </span>
        <script>
            let socket = new WebSocket("ws://" + window.location.hostname + "/websocket");

            function servo_1_command(movement) {
                let new_servo_1_angle = 90;
                if (movement === "left") {
                    new_servo_1_angle = 45;
                } else if (movement === "right") {
                    new_servo_1_angle = 135;
                }
                if (socket.readyState === WebSocket.OPEN) {
                    socket.send(JSON.stringify({
                        new_servo_1_angle: new_servo_1_angle
                    }));
                    document.getElementById("span_servo_1_angle").innerText = new_servo_1_angle;
                }
            }

            function servo_2_command(movement) {
                let new_servo_2_angle = 90;
                if (movement === "up") {
                    new_servo_2_angle = 45;
                } else if (movement === "down") {
                    new_servo_2_angle = 135;
                }
                if (socket.readyState === WebSocket.OPEN) {
                    socket.send(JSON.stringify({
                        new_servo_2_angle: new_servo_2_angle
                    }));
                    document.getElementById("span_servo_2_angle").innerText = new_servo_2_angle;
                }
            }

            function reset_servos_command() {
                if (socket.readyState === WebSocket.OPEN) {
                    socket.send(JSON.stringify({
                        new_servo_1_angle: 90,
                        new_servo_2_angle: 90
                    }));
                    document.getElementById("span_servo_1_angle").innerText = 90;
                    document.getElementById("span_servo_2_angle").innerText = 90;
                }
            }
            
            function throttle_command(throttle_percentage) {
                let throttle_setting = Math.round((throttle_percentage / 100) * 180);
                if (socket.readyState === WebSocket.OPEN) {
                    socket.send(JSON.stringify({
                        new_throttle_setting: throttle_setting
                    }));
                    document.getElementById("span_throttle_percentage").innerText = throttle_percentage;
                }
            }
        </script>
    </body>
</html>
)rawliteral";

Servo servo_1;
Servo servo_2;
Servo esc;

AsyncWebServer server(80);
AsyncWebSocket websocket("/websocket");

void understand_websocket_message(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, data, len);
        if (!error) {
            if (doc.containsKey("new_servo_1_angle")) {
                int new_servo_1_angle = constrain((int) doc["new_servo_1_angle"], 0, 180);
                servo_1.write(new_servo_1_angle);
            }
            if (doc.containsKey("new_servo_2_angle")) {
                int new_servo_2_angle = constrain((int) doc["new_servo_2_angle"], 0, 180);
                servo_2.write(new_servo_2_angle);
            }
            if (doc.containsKey("new_throttle_setting")) {
                int new_throttle_setting = constrain((int) doc["new_throttle_setting"], 0, 180);
                esc.write(new_throttle_setting);
            }
        }
    }
}

void triggered_on_websocket_event(AsyncWebSocket *server, AsyncWebSocketClient * client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        understand_websocket_message(arg, data, len);
    }
}

void setup() {
    Serial.begin(115200);

    ESP32PWM::allocateTimer(0);
    servo_1.setPeriodHertz(50);
    servo_1.attach(servo_1_pin, 500, 2400);
    servo_1.write(90);

    ESP32PWM::allocateTimer(1);
    servo_2.setPeriodHertz(50);
    servo_2.attach(servo_2_pin, 500, 2400);
    servo_2.write(90);

    ESP32PWM::allocateTimer(2);
    esc.setPeriodHertz(50);
    esc.attach(esc_pin, 1000, 2000);
    esc.write(0);

    WiFi.softAP(ssid, password);

    websocket.onEvent(triggered_on_websocket_event);
    server.addHandler(&websocket);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", html_code);
    });

    server.begin();
}

void loop() {
    websocket.cleanupClients();
}