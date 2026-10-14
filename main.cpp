#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

const char* ssid = "ESP32-Flight_Control_System";
const char* password = "192837465";

const short int servo_1_pin = 25;
const short int servo_2_pin = 26;
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
        <button onclick = "servo_command('left')">
        Left
        </button>
        <button onclick = "servo_command('right')">
        Right
        </button>
        <button onclick = "servo_command('up')">
        Up
        </button>
        <button onclick = "servo_command('down')">
        Down
        </button>
        <button onclick = "servo_command('reset')">
        Reset
        </button>
        <span id = "span_servo_1_angle">
        90 degree(s)
        </span>
        <span id = "span_servo_2_angle">
        90 degree(s)
        </span>
        <label for = "slider_throttle_percentage">
        Throttle:
        </label>
        <input type = "range" id = "slider_throttle_percentage" min = "0" max = "180" value = "90" oninput = "throttle_command(this.value)">
        <span id = "span_throttle_percentage">
        0%
        </span>
        <br>
        <span id = "span_joystick_status">
        Joystick Status: Disconnected
        </span>
        <script>
            let socket = new WebSocket("ws://" + window.location.hostname + "/websocket");
            let last_servo_1_angle = 90;
            let last_servo_2_angle = 90;
            let last_throttle_setting = 0;
            let previous_time = 0;
            let time_per_frame = 50;

            function servo_command(movement) {
                let data_to_send = {};
                let new_servo_1_angle = 90;
                let new_servo_2_angle = 90;
                if (movement === "left") {
                    new_servo_1_angle = 45
                    data_to_send.new_servo_1_angle = new_servo_1_angle;
                } else if (movement === "right") {
                    new_servo_1_angle = 135
                    data_to_send.new_servo_1_angle = new_servo_1_angle;
                } else if (movement === "up") {
                    new_servo_2_angle = 45
                    data_to_send.new_servo_2_angle = new_servo_2_angle;
                } else if (movement === "down") {
                    new_servo_2_angle = 135
                    data_to_send.new_servo_2_angle = new_servo_2_angle;
                } else if (movement === "reset") {
                    new_servo_1_angle = 90;
                    new_servo_2_angle = 90;
                    data_to_send.new_servo_1_angle = new_servo_1_angle;
                    data_to_send.new_servo_2_angle = new_servo_2_angle;
                }
                if (socket.readyState === WebSocket.OPEN) {
                    socket.send(JSON.stringify(data_to_send));
                    document.getElementById("span_servo_1_angle").innerText = new_servo_1_angle + " degree(s);";
                    document.getElementById("span_servo_2_angle").innerText = new_servo_2_angle + " degree(s);";
                }
            }

            function throttle_command(throttle_percentage) {
                if (socket.readyState === WebSocket.OPEN) {
                    socket.send("90," + throttle_percentage + ",0");
                    document.getElementById("span_throttle_percentage").innerText = throttle_percentage;
                }
            }
            
            function update_joystick_input() {
                let joystick_array = navigator.getGamepads();
                let current_time = Date.now();
                if (joystick_array[0] && (current_time - previous_time) >= time_per_frame) {
                    let joystick = joystick_array[0];
                    if (joystick.connected) {
                        let new_servo_1_angle = Math.round(Math.abs((joystick.axes[0] - 1) / 2) * 40) + 70;
                        let new_servo_2_angle = Math.round(Math.abs((joystick.axes[1] - 1) / 2) * 50) + 65;
                        let new_throttle_setting = Math.round((Math.abs(joystick.axes[3] - 1) / 2) * 180);
                        
                        document.getElementById("span_servo_1_angle").innerText = new_servo_1_angle;
                        document.getElementById("span_servo_2_angle").innerText = new_servo_2_angle;
                        document.getElementById("span_throttle_percentage").innerText = Math.round((Math.abs(joystick.axes[3] - 1) / 2) * 100) + "%";
                        
                        if (socket.readyState === WebSocket.OPEN) {
                            socket.send(new_servo_1_angle + "," + new_servo_2_angle + "," + new_throttle_setting);
                        }
                        
                        previous_time = Date.now()
                    }
                }
                requestAnimationFrame(update_joystick_input);
            }

            window.addEventListener("gamepadconnected", (e) => {
            console.log("Gamepad connected:", e.gamepad.id);
            document.getElementById("span_joystick_status").innerText = "Joystick Status: Connected to " + e.gamepad.id;
            });

            window.addEventListener("gamepaddisconnected", (e) => {
            document.getElementById("span_joystick_status").innerText = "Joystick Status: Disconnected";
            });

            requestAnimationFrame(update_joystick_input);
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
        char buffer[64];
        
        if (len >= sizeof(buffer)) {
            len = sizeof(buffer) - 1;
        }
        
        memcpy(buffer, data, len);
        buffer[len] = '\0';
        
        int new_servo_1_angle, new_servo_2_angle, new_throttle_setting;

        if (sscanf(buffer, "%d,%d,%d", &new_servo_1_angle, &new_servo_2_angle, &new_throttle_setting) == 3) {
            servo_1.write(constrain(new_servo_1_angle, 70, 110));
            servo_2.write(constrain(new_servo_2_angle, 45, 135));
            esc.write(constrain(new_throttle_setting, 0, 180));
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

// let throttle_setting = Math.round((throttle_percentage / 100) * 180);