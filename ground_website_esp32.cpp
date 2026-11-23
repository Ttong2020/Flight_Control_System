#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>


const char* ssid = "ESP32-Flight_Control_System";
const char* password = "192837465";

const uint8_t ground_uart2_rx_pin = 18;
const uint8_t ground_uart2_tx_pin = 19;

HardwareSerial ground_serial(2);
AsyncWebServer webserver(80);
AsyncWebSocket websocket("/websocket");

const char html_code[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang = "en-GB">
    <head>
        <title>
        RC Plane Flight Control System
        </title>
    </head>
    <body>
        <span id = "new_measured_airspeed">
        Airspeed (km/h)
        </span> <br>
        <label id = "label_new_measured_airspeed" for = "new_measured_airspeed">
        0
        </label> <br>
        
        <span id = "new_measured_altitude">
        Altitide (ft)
        </span> <br>
        <label id = "label_new_measured_altitude" for = "new_measured_altitude">
        0
        </label> <br>

        <span id = "new_measured_heading">
        Heading (deg)
        </span> <br>
        <label id = "label_new_measured_heading" for = "new_measured_heading">
        0
        </label> <br>

        <script>
            let websocket = new WebSocket("ws://" + window.location.hostname + "/websocket");
            let new_measured_airspeed = 0;
            let new_measured_altitude = 0;
            let new_measured_heading = 0;
            let new_measured_vertical_speed = 0;
            let new_measured_signal_quality_score = 0;

            function decode_show_message_from_ground_bluetooth(event) {
                let incoming_message = event.data.split(",");
                if (incoming_message.length == 5) {
                    document.getElementById("label_new_measured_airspeed").innerText = incoming_message[0];
                    document.getElementById("label_new_measured_altitude").innerText = incoming_message[1];
                    document.getElementById("label_new_measured_heading").innerText = incoming_message[2];
                }
            }

            websocket.onmessage = decode_show_message_from_ground_bluetooth;
        </script>
    </body>
</html>
)rawliteral";

void setup() {
    WiFi.softAP(ssid, password);
    webserver.addHandler(&websocket);

    webserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", html_code);
    });

    webserver.begin();

    Serial.begin(115200);
    
    ground_serial.begin(115200, SERIAL_8N1, ground_uart2_rx_pin, ground_uart2_tx_pin);
}

void loop() {
    if (ground_serial.available() > 0) {
        int32_t new_measured_airspeed;
        int32_t new_measured_altitude;
        int32_t new_measured_heading;
        int32_t new_measured_vertical_speed;
        int32_t new_measured_signal_quality_score;
        String serial_message = ground_serial.readStringUntil('\n');
        Serial.println(serial_message);
        sscanf(serial_message.c_str(), "%d,%d,%d,%d,%d", &new_measured_airspeed, &new_measured_altitude, &new_measured_heading, &new_measured_vertical_speed, &new_measured_signal_quality_score);
        
        String message_to_send_over_websocket = String(new_measured_airspeed) + "," + String(new_measured_altitude) + "," + String(new_measured_heading) + "," + String(new_measured_vertical_speed) + "," + String(new_measured_signal_quality_score);
        websocket.textAll(message_to_send_over_websocket);
    }

    websocket.cleanupClients();
}