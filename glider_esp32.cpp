#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <map>

const uint8_t aileron_servo_pin = 25;
const uint8_t elevator_servo_pin = 26;
const uint8_t throttle_esc_pin = 21;

std::map<std::string, std::array<float, 5>> kp_ki_kd_sum_of_past_integrals_previous_error_array = {{"airspeed", {1.0f, 1.0f, 1.0f, 0.0f, 0.0f}},
                                                                                                   {"altitude", {1.0f, 1.0f, 1.0f, 0.0f, 0.0f}},
                                                                                                   {"heading", {1.0f, 1.0f, 1.0f, 0.0f, 0.0f}}
                                                                                                  };
const uint8_t dt = 50;
const uint8_t stall_airspeed = 

Servo aileron_servo;
Servo elevator_servo;
Servo throttle_servo;

float pid_controller(std::string autopilot_parameter, float target_value, float measured_value) {
    float current_error = target_value - measured_value;
    float proportional_contribution = kp_ki_kd_sum_of_past_integrals_previous_error_array[autopilot_parameter][0] * current_error;
    float integral_contribution = kp_ki_kd_sum_of_past_integrals_previous_error_array[autopilot_parameter][1] * (kp_ki_kd_sum_of_past_integrals_previous_error_array[autopilot_parameter][3] + (current_error * dt));
    float derivative_contribution = kp_ki_kd_sum_of_past_integrals_previous_error_array[autopilot_parameter][2] * ((current_error - kp_ki_kd_sum_of_past_integrals_previous_error_array[autopilot_parameter][4]) / dt);
    float output_control = proportional_contribution + integral_contribution + derivative_contribution;
    kp_ki_kd_sum_of_past_integrals_previous_error_array[autopilot_parameter][3] = integral_contribution;
    kp_ki_kd_sum_of_past_integrals_previous_error_array[autopilot_parameter][4] = current_error;

    return(output_control);
}

void decode_message_from_ground_esp32(const uint8_t *ground_esp32_mac_address, const uint8_t *incomingData, int len) {
    
}

void setup() {
    Serial.begin(115200);

    ESP32PWM::allocateTimer(0);
    aileron_servo.setPeriodHertz(50);
    aileron_servo.attach(aileron_servo_pin, 500, 2400);
    aileron_servo.write(90);

    ESP32PWM::allocateTimer(1);
    elevator_servo.setPeriodHertz(50);
    elevator_servo.attach(elevator_servo_pin, 500, 2400);
    elevator_servo.write(90);

    ESP32PWM::allocateTimer(2);
    throttle_servo.setPeriodHertz(50);
    throttle_servo.attach(throttle_esc_pin, 1000, 2000);
    throttle_servo.write(0);

    WiFi.mode(WIFI_STA);
    if (esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR) != ESP_OK) {
        Serial.println("Error: failed to set up Long Range Protocol.");
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error: failed to initialise ESP-NOW.");
    } else {
        Serial.println("Success: Successfully initialised ESP-NOW Long Range Protocol.");
    }

    esp_now_register_recv_cb(esp_now_recv_cb_t(decode_message_from_ground_esp32));
}

void loop() {
}