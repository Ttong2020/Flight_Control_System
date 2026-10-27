#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include <map>

const uint8_t aileron_servo_pin = 25;
const uint8_t elevator_servo_pin = 26;
const uint8_t throttle_esc_pin = 21;

const uint8_t bytes_expected_for_configuration_array = 56;
const uint8_t bytes_expected_for_normal_operation = 6;

std::map<std::string, std::array<float, 6>> kp_ki_kd_sum_of_past_integrals_previous_error_measured_value_array = {{"airspeed", {1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f}},
                                                                                                                  {"altitude", {1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f}},
                                                                                                                  {"heading", {1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f}},
                                                                                                                  {"vertical speed", {1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f}}
                                                                                                                 };
const float dt = 0.05;
// const uint8_t stall_airspeed;

int16_t scale_factor_value_to_add_number_of_possible_value_new_value_array[7][4] = {{1, 60, 61, 90}, // new_aileron_servo_angle
                                                                                    {1, 45, 91, 90}, // new_elevator_servo_angle
                                                                                    {4, 0, 46, 0}, // new_throttle_servo_angle
                                                                                    {1, 0, 31, 0}, // new_target_airspeed
                                                                                    {1, 0, 151, 0}, // new_target_altitude
                                                                                    {4, 0, 91, 0}, // new_target_heading
                                                                                    {4, -600, 301, 0} // new_target_vertical_speed
                                                                                   };

Servo aileron_servo;
Servo elevator_servo;
Servo throttle_servo;

float pid_controller(std::string autopilot_parameter, int16_t target_value) {
    float current_error = target_value - kp_ki_kd_sum_of_past_integrals_previous_error_measured_value_array[autopilot_parameter][5];
    float proportional_contribution = kp_ki_kd_sum_of_past_integrals_previous_error_measured_value_array[autopilot_parameter][0] * current_error;
    float integral_contribution = kp_ki_kd_sum_of_past_integrals_previous_error_measured_value_array[autopilot_parameter][1] * (kp_ki_kd_sum_of_past_integrals_previous_error_measured_value_array[autopilot_parameter][3] + (current_error * dt));
    float derivative_contribution = kp_ki_kd_sum_of_past_integrals_previous_error_measured_value_array[autopilot_parameter][2] * ((current_error - kp_ki_kd_sum_of_past_integrals_previous_error_measured_value_array[autopilot_parameter][4]) / dt);
    float output_control = proportional_contribution + integral_contribution + derivative_contribution;
    kp_ki_kd_sum_of_past_integrals_previous_error_measured_value_array[autopilot_parameter][3] = integral_contribution;
    kp_ki_kd_sum_of_past_integrals_previous_error_measured_value_array[autopilot_parameter][4] = current_error;

    return(output_control);
}

void decode_message_from_ground_esp32(const uint8_t *ground_esp32_mac_address, const uint8_t *incomingData, int len) {
    if (len == bytes_expected_for_normal_operation) {
        uint64_t total_value_received_base_256 = 0;
        for (uint8_t i = 0; i < bytes_expected_for_normal_operation; i++) {
            uint64_t value_at_byte_index_i = incomingData[i];
            total_value_received_base_256 = total_value_received_base_256 + (value_at_byte_index_i << (8 * i));
        }

        uint8_t flag_bits = total_value_received_base_256 & 0x07;
        bool is_airspeed_autopilot = (total_value_received_base_256 >> 2) & 0x01;
        bool is_altitude_autopilot = (total_value_received_base_256 >> 1) & 0x01;
        bool is_heading_autopilot = (total_value_received_base_256 >> 0) & 0x01;
        total_value_received_base_256 = total_value_received_base_256 >> 3;
        
        uint8_t number_of_parameters = sizeof(scale_factor_value_to_add_number_of_possible_value_new_value_array) / sizeof(scale_factor_value_to_add_number_of_possible_value_new_value_array[0]);
        uint64_t multiplication_of_number_of_possible_value = 1;
        for (uint8_t i = 0; i < number_of_parameters; i++) {
            int16_t value_sent_at_index_i = (total_value_received_base_256 / multiplication_of_number_of_possible_value) % scale_factor_value_to_add_number_of_possible_value_new_value_array[i][2];
            scale_factor_value_to_add_number_of_possible_value_new_value_array[i][3] = value_sent_at_index_i * scale_factor_value_to_add_number_of_possible_value_new_value_array[i][0] + scale_factor_value_to_add_number_of_possible_value_new_value_array[i][1];
            multiplication_of_number_of_possible_value = multiplication_of_number_of_possible_value * (uint64_t) scale_factor_value_to_add_number_of_possible_value_new_value_array[i][2];
        }

        if (is_airspeed_autopilot == true) {
            scale_factor_value_to_add_number_of_possible_value_new_value_array[2][3] = pid_controller("airspeed", scale_factor_value_to_add_number_of_possible_value_new_value_array[3][3]);
        }
        if (is_altitude_autopilot == true) {
            scale_factor_value_to_add_number_of_possible_value_new_value_array[1][3] = pid_controller("vertical speed", scale_factor_value_to_add_number_of_possible_value_new_value_array[6][3]) + 90;
        }
        if (is_heading_autopilot == true) {
            scale_factor_value_to_add_number_of_possible_value_new_value_array[0][3] = pid_controller("heading", scale_factor_value_to_add_number_of_possible_value_new_value_array[5][3]) + 90;
        }

        aileron_servo.write(constrain(scale_factor_value_to_add_number_of_possible_value_new_value_array[0][3], 70, 110));
        elevator_servo.write(constrain(scale_factor_value_to_add_number_of_possible_value_new_value_array[1][3], 45, 135));
        throttle_servo.write(constrain(scale_factor_value_to_add_number_of_possible_value_new_value_array[2][3], 0, 180));
    }
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

    esp_now_register_recv_cb(decode_message_from_ground_esp32);
}

void loop() {

}