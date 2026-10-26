#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>

const uint16_t number_of_possible_combination_array[7] = {61, 91, 46, 31, 151, 91, 301};

std::vector<uint8_t> encode_message(const uint8_t new_aileron_servo_angle, const uint8_t new_elevator_servo_angle, const uint8_t new_throttle_servo_angle, const uint8_t new_target_airspeed, const uint8_t new_target_altitude, const uint8_t new_target_heading, const uint8_t new_target_vertical_speed, const bool is_airspeed_autopilot, const bool is_altitude_autopilot, const bool is_heading_autopilot) {
    const uint16_t new_values_to_send_array[7] = {new_aileron_servo_angle, new_elevator_servo_angle, new_throttle_servo_angle, new_target_airspeed, new_target_altitude, new_target_heading, new_target_vertical_speed};

    uint64_t total_value_to_send_base_10 = 0;
    uint64_t multiplication_of_number_of_possible_combination = 1;
    for (uint8_t i = 0; i < 7; i++) {
        total_value_to_send_base_10 = total_value_to_send_base_10 + new_values_to_send_array[i] * multiplication_of_number_of_possible_combination;
        multiplication_of_number_of_possible_combination = multiplication_of_number_of_possible_combination * number_of_possible_combination_array[i];
    }

    total_value_to_send_base_10 = (total_value_to_send_base_10 << 3) | ((is_airspeed_autopilot ? 1 : 0) << 2) | ((is_altitude_autopilot ? 1 : 0) << 1) | ((is_heading_autopilot ? 1 : 0) << 0);

    uint8_t bit_required = 0;
    if (total_value_to_send_base_10 <= 0) {
        bit_required = 1;
    } else {
        bit_required = sizeof(total_value_to_send_base_10) * 8 - __builtin_clzll(total_value_to_send_base_10);
    }
    // By adding 7, if bit_required = 40, then (bit_required + 7) DIV 8 = 5. But if bit_required = 41, then (bit_required + 7) DIV 8 = 6.
    uint8_t minimum_number_of_bytes_required = (bit_required + 7) / 8;

    std::vector<uint8_t> message_to_send_in_byte_array(minimum_number_of_bytes_required);
    for (uint8_t i = 0; i < minimum_number_of_bytes_required; i++) {
        message_to_send_in_byte_array[i] = (total_value_to_send_base_10 >> (8 * (minimum_number_of_bytes_required - i - 1))) & 0xFF;
    }
    
    return message_to_send_in_byte_array;
}

void normal_operation() {
    std::vector<uint8_t> encoded_message_to_send = encode_message(new_aileron_servo_angle, new_elevator_servo_angle, new_throttle_servo_angle, new_target_airspeed, new_target_altitude, new_target_heading, new_target_vertical_speed, is_airspeed_autopilot, is_altitude_autopilot, is_heading_autopilot);
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
}