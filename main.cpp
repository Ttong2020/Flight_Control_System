#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <map>

const char* ssid = "ESP32_Flight_Control_System";
const char* password = "192837465";

const uint8_t servo_1_pin = 25;
const uint8_t servo_2_pin = 26;
const uint8_t esc_pin = 21;

std::map<std::string, std::array<float, 5>> kp_ki_kd_sum_of_past_integrals_previous_error = {{"airspeed", {1.0f, 1.0f, 1.0f, 0.0f, 0.0f}},
                                                                                             {"altitude", {1.0f, 1.0f, 1.0f, 0.0f, 0.0f}},
                                                                                             {"heading", {1.0f, 1.0f, 1.0f, 0.0f, 0.0f}}
                                                                                            };
const uint8_t dt = 50;
const uint8_t stall_airspeed = 

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

Servo servo_1;
Servo servo_2;
Servo esc;

AsyncWebServer server(80);
AsyncWebSocket websocket("/websocket");

float pid_controller(std::string autopilot_parameter, float target_value, float measured_value) {
    float current_error = target_value - measured_value;
    float proportional_contribution = kp_ki_kd_sum_of_past_integrals_previous_error[autopilot_parameter][0] * current_error;
    float integral_contribution = kp_ki_kd_sum_of_past_integrals_previous_error[autopilot_parameter][1] * (kp_ki_kd_sum_of_past_integrals_previous_error[autopilot_parameter][3] + (current_error * dt));
    float derivative_contribution = kp_ki_kd_sum_of_past_integrals_previous_error[autopilot_parameter][2] * ((current_error - kp_ki_kd_sum_of_past_integrals_previous_error[autopilot_parameter][4]) / dt);
    float output_control = proportional_contribution + integral_contribution + derivative_contribution;
    kp_ki_kd_sum_of_past_integrals_previous_error[autopilot_parameter][3] = integral_contribution;
    kp_ki_kd_sum_of_past_integrals_previous_error[autopilot_parameter][4] = current_error;

    return(output_control);
}

void understand_websocket_message(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        char buffer[64];
        
        if (len >= sizeof(buffer)) {
            len = sizeof(buffer) - 1;
        }
        
        memcpy(buffer, data, len);
        buffer[len] = '\0';
        
        int new_servo_1_angle, new_servo_2_angle, new_throttle_setting, target_airspeed, target_altitude, target_heading;

        if (sscanf(buffer, "%d,%d,%d,%f,%f,%f", &new_servo_1_angle, &new_servo_2_angle, &new_throttle_setting, &target_airspeed, &target_altitude, &target_heading) == 6) {
            if (measured_airspeed < (stall_airspeed * 1.1)) {

            } else if (measured_airspeed >= (stall_airspeed * 1.1)) {
                if (target_airspeed) {
                    uint8_t new_throttle_setting = Math.round(pid_controller(std::string "airspeed", target_airspeed, measured_airspeed))
                }
                if (target_altitude) {
                    uint8_t new_servo_2_angle = Math.round(pid_controller(std::string "altitude", target_altitude, measured_altitude))
                }
                if (target_heading) {
                    uint8_t new_servo_1_angle = Math.round(pid_controller(std::string "heading", target_heading, measured_heading))
                }
            }
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