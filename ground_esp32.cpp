#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <vector>
#include <ps5Controller.h>

uint8_t glider_esp32_mac_address[6] = {0x88, 0xF1, 0x55, 0x31, 0x07, 0x40};
bool is_ps5_controller_connected = false;

const uint8_t minimum_number_of_bytes_required = 6;
std::vector<uint8_t> message_to_send_in_byte_array(minimum_number_of_bytes_required);

const int16_t scale_factor_value_to_add_number_of_possible_value_new_value_array[7][4] = {{1, 60, 61, 90}, // new_aileron_servo_angle
                                                                                          {1, 45, 91, 90}, // new_elevator_servo_angle
                                                                                          {4, 0, 46, 0}, // new_throttle_servo_angle
                                                                                          {1, 0, 31, 0}, // new_target_airspeed
                                                                                          {1, 0, 151, 0}, // new_target_altitude
                                                                                          {4, 0, 91, 0}, // new_target_heading
                                                                                          {4, -600, 301, 0} // new_target_vertical_speed
                                                                                         };

void encode_send_message(const uint8_t new_aileron_servo_angle, const uint8_t new_elevator_servo_angle, const uint8_t new_throttle_servo_angle, const uint8_t new_target_airspeed, const uint8_t new_target_altitude, const uint8_t new_target_heading, const uint8_t new_target_vertical_speed, const bool is_airspeed_autopilot, const bool is_altitude_autopilot, const bool is_heading_autopilot) {
    const uint16_t new_values_to_send_array[7] = {(uint16_t) round((float) (new_aileron_servo_angle - scale_factor_value_to_add_number_of_possible_value_new_value_array[0][1]) / (float) scale_factor_value_to_add_number_of_possible_value_new_value_array[0][0]),
                                                  (uint16_t) round((float) (new_elevator_servo_angle - scale_factor_value_to_add_number_of_possible_value_new_value_array[1][1]) / (float) scale_factor_value_to_add_number_of_possible_value_new_value_array[1][0]),
                                                  (uint16_t) round((float) (new_throttle_servo_angle - scale_factor_value_to_add_number_of_possible_value_new_value_array[2][1]) / (float) scale_factor_value_to_add_number_of_possible_value_new_value_array[2][0]),
                                                  (uint16_t) round((float) (new_target_airspeed - scale_factor_value_to_add_number_of_possible_value_new_value_array[3][1]) / (float) scale_factor_value_to_add_number_of_possible_value_new_value_array[3][0]),
                                                  (uint16_t) round((float) (new_target_altitude - scale_factor_value_to_add_number_of_possible_value_new_value_array[4][1]) / (float) scale_factor_value_to_add_number_of_possible_value_new_value_array[4][0]),
                                                  (uint16_t) round((float) (new_target_heading - scale_factor_value_to_add_number_of_possible_value_new_value_array[5][1]) / (float) scale_factor_value_to_add_number_of_possible_value_new_value_array[5][0]),
                                                  (uint16_t) round((float) (new_target_vertical_speed - scale_factor_value_to_add_number_of_possible_value_new_value_array[6][1]) / (float) scale_factor_value_to_add_number_of_possible_value_new_value_array[6][0])
                                                 };

    uint64_t total_value_to_send_base_256 = 0;
    uint64_t multiplication_of_number_of_possible_combination = 1;
    for (uint8_t i = 0; i < 7; i++) {
        total_value_to_send_base_256 = total_value_to_send_base_256 + new_values_to_send_array[i] * multiplication_of_number_of_possible_combination;
        multiplication_of_number_of_possible_combination = multiplication_of_number_of_possible_combination * (uint64_t) scale_factor_value_to_add_number_of_possible_value_new_value_array[i][2];
    }

    total_value_to_send_base_256 = (total_value_to_send_base_256 << 3) | ((is_airspeed_autopilot ? 1 : 0) << 2) | ((is_altitude_autopilot ? 1 : 0) << 1) | ((is_heading_autopilot ? 1 : 0) << 0);

    for (uint8_t i = 0; i < minimum_number_of_bytes_required; i++) {
        message_to_send_in_byte_array[i] = (total_value_to_send_base_256 >> (8 * i)) & 0xFF;
    }
    
    esp_now_send(glider_esp32_mac_address, message_to_send_in_byte_array.data(), message_to_send_in_byte_array.size());
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

    esp_now_peer_info_t connection_with_glider_esp32_peer_info = {};
    memcpy(connection_with_glider_esp32_peer_info.peer_addr, glider_esp32_mac_address, 6);
    connection_with_glider_esp32_peer_info.channel = 0;
    connection_with_glider_esp32_peer_info.encrypt = false;
    if (esp_now_add_peer(&connection_with_glider_esp32_peer_info) == ESP_OK) {
        Serial.println("Success: Successfuly added glider ESP32 as peer.");
    } else {
        Serial.println("Error: failed to add glider ESP32 as peer.");
    }

    ps5.begin();
    Serial.println("Bluetooth initialised.");
    Serial.print("Waiting for PS5 Controller...");
}

void loop() {
    if (ps5.isConnected() == true) {
        if (is_ps5_controller_connected != true) {
            is_ps5_controller_connected = true;
            Serial.println();
            Serial.println("Success: PS5 Controller successfully connected.");
        }

        delay(50);
        
        uint8_t new_aileron_servo_angle = round((1.0 - ((float) (ps5.lx + 128) / 255)) * 40 + 70);
        uint8_t new_elevator_servo_angle = round((1.0 - ((float) (ps5.ly + 128) / 255)) * 90 + 45);
        uint8_t new_throttle_servo_angle = round(((float) ps5.r2 / 255) * 180);

        encode_send_message(new_aileron_servo_angle, new_elevator_servo_angle, new_throttle_servo_angle, 0, 0, 0, 0, false, false, false);
    } else {
        if (is_ps5_controller_connected != false) {
            is_ps5_controller_connected = false;
            Serial.print("Waiting for PS5 Controller...");
        } else {
            Serial.print(".");
        }
        delay(500);
    }
}