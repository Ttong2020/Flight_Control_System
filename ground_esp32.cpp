#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <map>

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

        <span id = "span_servo_1_angle">
        Aileron: 90 degree(s)
        </span> <br>

        <span id = "span_servo_2_angle">
        Elevator: 90 degree(s)
        </span> <br>

        <span id = "span_throttle_percentage">
        Throttle: 0%
        </span> <br> <br>


        <span id = "span_joystick_status">
        Joystick Status: Disconnected
        </span> <br> <br>


        <input type = "radio" id = "radio_input_Extreme_3D_pro_(Vendor:_046d_Product:_c215)" name = "joystick_options" value = "Extreme 3D pro (Vendor: 046d Product: c215)" onclick = "update_joystick_input(value)">
        <label for = "radio_input_Extreme_3D_pro_(Vendor:_046d_Product:_c215)">
        Extreme 3D pro (Vendor: 046d Product: c215)
        </label> <br>
        
        <input type = "radio" id = "radio_input_DualSense_Wireless_Controller_(STANDARD_GAMEPAD)" name = "joystick_options" value = "DualSense Wireless Controller (STANDARD GAMEPAD)" onclick = "update_joystick_input(value)">
        <label for = "radio_input_DualSense_Wireless_Controller_(STANDARD_GAMEPAD)">
        DualSense Wireless Controller (STANDARD GAMEPAD)
        </label>

        <script>
            let socket = new WebSocket("ws://" + window.location.hostname + "/websocket");
            let last_servo_1_angle = 90;
            let last_servo_2_angle = 90;
            let last_throttle_setting = 0;
            let previous_time = 0;
            let time_per_frame = 50;
            let animation_frame_id = null;
            let target_airspeed = null;
            let target_altitude = null;
            let target_heading = null;

            function update_joystick_input(joystick_id) {
                if (animation_frame_id) {
                    cancelAnimationFrame(animation_frame_id);
                }
                
                let joystick_array = navigator.getGamepads();
                let current_time = Date.now();
                let joystick = Array.from(joystick_array).find(joystick_array => joystick_array && joystick_array.id === joystick_id);

                if (joystick && joystick.connected && (current_time - previous_time) >= time_per_frame) {
                    let joystick_servo_angle_throttle_setting_2D_array = [["Extreme 3D pro (Vendor: 046d Product: c215)", Math.round(Math.abs((joystick.axes[0] - 1) / 2) * 40) + 70, Math.round(Math.abs((joystick.axes[1] - 1) / 2) * 50) + 65, Math.round(Math.abs(joystick.axes[6] - 1) / 2 * 180)],
                                                                          ["DualSense Wireless Controller (STANDARD GAMEPAD)", Math.round(Math.abs((joystick.axes[0] - 1) / 2) * 40) + 70, Math.round(Math.abs((joystick.axes[1] - 1) / 2) * 50) + 65, Math.round(joystick.buttons[7].value * 180)]]
                    
                    
                    document.getElementById("span_joystick_status").innerText = "Joystick Status: Connected to " + joystick_id;

                    document.getElementById("span_servo_1_angle").innerText = "Aileron: " + joystick_servo_angle_throttle_setting_2D_array[joystick_servo_angle_throttle_setting_2D_array.findIndex(row => row[0] === joystick_id)][1] + " degree(s)";
                    document.getElementById("span_servo_2_angle").innerText = "Elevator: " + joystick_servo_angle_throttle_setting_2D_array[joystick_servo_angle_throttle_setting_2D_array.findIndex(row => row[0] === joystick_id)][2] + " degree(s)";
                    document.getElementById("span_throttle_percentage").innerText = "Throttle: " + Math.round(joystick_servo_angle_throttle_setting_2D_array[joystick_servo_angle_throttle_setting_2D_array.findIndex(row => row[0] === joystick_id)][3] / 1.8) + " %";
                    
                    if (socket.readyState === WebSocket.OPEN) {
                        socket.send(joystick_servo_angle_throttle_setting_2D_array[joystick_servo_angle_throttle_setting_2D_array.findIndex(row => row[0] === joystick_id)][1] + "," + joystick_servo_angle_throttle_setting_2D_array[joystick_servo_angle_throttle_setting_2D_array.findIndex(row => row[0] === joystick_id)][2] + "," + joystick_servo_angle_throttle_setting_2D_array[joystick_servo_angle_throttle_setting_2D_array.findIndex(row => row[0] === joystick_id)][3] + "," + target_airspeed + "," + target_altitude + "," + target_heading);
                        previous_time = Date.now()
                    }
                    
                }
                
                animation_frame_id = requestAnimationFrame(() => update_joystick_input(joystick_id));
            }

            window.addEventListener("gamepadconnected", (e) => {
            });

            window.addEventListener("gamepaddisconnected", (e) => {
            document.getElementById("span_joystick_status").innerText = "Joystick Status: Disconnected";
            });

        </script>
    </body>
</html>
)rawliteral";

void normal_operation() {
    encoded_message = encode_message(new_aileron_servo_angle, new_elevator_servo_angle, new_throttle_servo_angle, new_target_airspeed, new_target_altitude, new_target_heading, new_target_vertical_speed, is_airspeed_autopilot, is_altitude_autopilot, is_heading_autopilot);
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    if (esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR) != ESP_OK) {
        Serial.println("Error: failed to set up Long Range Protocol.");
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error: failed to initialise ESP-NOW.");
    } else {
        Serial.println("Success: Successfully initialised ESP-NOW Long Range Protocol.");
    }

    normal_operation();
}

void loop() {
    websocket.cleanupClients();
}

/* let throttle_setting = Math.round((throttle_percentage / 100) * 180);


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
                    socket.send("90,90," + throttle_percentage);
                    document.getElementById("span_throttle_percentage").innerText = throttle_percentage;
                }
            }
            */


            /*
                    
                    
                    
            requestAnimationFrame(update_joystick_input(e.gamepad.id));
                }*/