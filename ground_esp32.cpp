#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <vector>
#include <ps5Controller.h>
#include <map>

constexpr uint8_t glider_esp32_mac_address[6] = {0x88, 0xF1, 0x55, 0x31, 0x07, 0x40};
bool is_ps5_controller_connected = false;
uint32_t previous_time_sent_packet_milliseconds = 0;
uint16_t time_delay_between_packet_request_milliseconds = 50;
bool is_configuration_struct_setup = false;

// configuration data to send to glider ESP32.
struct configuration_struct_1_encode_decode_message_info {
    uint8_t packet_num = 10;
    // {value to multiply, value to add, number of possible value, new value}.
    int32_t decode_configuration_array[8][4] = {{1, 60, 61, 90}, // new_aileron_servo_angle
                                                {1, 45, 91, 90}, // new_elevator_servo_angle
                                                {4, 0, 46, 0}, // new_throttle_servo_angle
                                                {1, 0, 31, 0}, // new_target_airspeed
                                                {1, 0, 151, 0}, // new_target_altitude
                                                {4, 0, 91, 0}, // new_target_heading
                                                {4, -600, 301, 0}, // new_target_vertical_speed
                                                {1, 0, 11, 0} // new_signal_quality_score
                                               };
    uint8_t starting_index_for_decode_configuration_array_ground_to_glider = 0;
    uint8_t ending_index_for_decode_configuration_array_ground_to_glider = 6;
    uint8_t starting_index_for_decode_configuration_array_glider_to_ground = 3;
    uint8_t ending_index_for_decode_configuration_array_glider_to_ground = 7;
    uint8_t bytes_required_for_normal_operation_ground_to_glider = 6;
    uint8_t bytes_required_for_normal_operation_glider_to_ground = 4;
};

struct configuration_struct_2_other_info {
    uint8_t packet_num = 20;
    uint8_t minimum_airspeed_before_autopilot_disengage;
    // {kp, ki, kd, sum of past integrals, previous error, target value, measured value}.
    float pid_controller_configuration_array[4][7] = {{1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                      {1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                      {1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                      {1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}
                                                     };
    float dt_seconds = 0.05;
    uint16_t wind_direction_degrees;
    uint8_t wind_speed_metres_per_second;
    uint8_t aileron_servo_pin = 25;
    uint8_t elevator_servo_pin = 26;
    uint8_t throttle_esc_pin = 21;
    uint8_t gps_uart2_rx_pin = 16;
    uint8_t gps_uart2_tx_pin = 17;
};

configuration_struct_1_encode_decode_message_info flight_config_1;
configuration_struct_2_other_info flight_config_2;

void print_measured_flight_data(const bool is_autopilot_engaged) {
    Serial.println();
}

//Need updating: Can view the 3 flags as the same as other parameters
void encode_send_message(const bool is_heading_autopilot, const bool is_altitude_autopilot, const bool is_airspeed_autopilot) {
    std::vector<uint8_t> message_to_send_in_byte_array(flight_config_1.bytes_required_for_normal_operation_ground_to_glider);
    
    for (uint8_t i = flight_config_1.starting_index_for_decode_configuration_array_ground_to_glider; i < flight_config_1.ending_index_for_decode_configuration_array_ground_to_glider + 1; i++) {
        flight_config_1.decode_configuration_array[i][3] = constrain((int32_t) round((float) (flight_config_1.decode_configuration_array[i][3] - flight_config_1.decode_configuration_array[i][1]) / (float) flight_config_1.decode_configuration_array[i][0]), 0, flight_config_1.decode_configuration_array[i][2]);
    }

    uint64_t total_value_to_send_base_256 = 0;
    uint64_t multiplication_of_number_of_possible_combination = 1;
    for (uint8_t i = flight_config_1.starting_index_for_decode_configuration_array_ground_to_glider; i < flight_config_1.ending_index_for_decode_configuration_array_ground_to_glider + 1; i++) {
        total_value_to_send_base_256 = total_value_to_send_base_256 + flight_config_1.decode_configuration_array[i][3] * multiplication_of_number_of_possible_combination;
        multiplication_of_number_of_possible_combination = multiplication_of_number_of_possible_combination * (uint64_t) flight_config_1.decode_configuration_array[i][2];
    }

    total_value_to_send_base_256 = (total_value_to_send_base_256 << 3) | ((is_heading_autopilot ? 1 : 0) << 2) | ((is_altitude_autopilot ? 1 : 0) << 1) | ((is_airspeed_autopilot ? 1 : 0) << 0);

    for (uint8_t i = 0; i < flight_config_1.bytes_required_for_normal_operation_ground_to_glider; i++) {
        message_to_send_in_byte_array[i] = (total_value_to_send_base_256 >> (8 * i)) & 0xFF;
    }
    
    esp_now_send(glider_esp32_mac_address, message_to_send_in_byte_array.data(), message_to_send_in_byte_array.size());
}

// Function to decode message from incoming message, and execute them through servos/motor.
void decode_execute_message_from_glider(const uint8_t *glider_esp32_mac_address, const uint8_t *incoming_message, int len) {
    if (len == flight_config_1.bytes_required_for_normal_operation_glider_to_ground) {
        uint64_t total_value_received_base_256 = 0;
        for (uint8_t i = 0; i < flight_config_1.bytes_required_for_normal_operation_glider_to_ground; i++) {
            uint64_t value_at_byte_index_i = incoming_message[i];
            total_value_received_base_256 = total_value_received_base_256 + (value_at_byte_index_i << (8 * i));
        }

        bool is_autopilot_engaged = (total_value_received_base_256 >> 0) & 0x01;
        total_value_received_base_256 = total_value_received_base_256 >> 1;
        
        uint64_t multiplication_of_number_of_possible_value = 1;
        for (uint8_t i = flight_config_1.starting_index_for_decode_configuration_array_glider_to_ground; i < flight_config_1.ending_index_for_decode_configuration_array_glider_to_ground + 1; i++) {
            int16_t value_sent_at_index_i = (total_value_received_base_256 / multiplication_of_number_of_possible_value) % flight_config_1.decode_configuration_array[i][2];
            flight_config_1.decode_configuration_array[i][3] = value_sent_at_index_i * flight_config_1.decode_configuration_array[i][0] + flight_config_1.decode_configuration_array[i][1];
            multiplication_of_number_of_possible_value = multiplication_of_number_of_possible_value * (uint64_t) flight_config_1.decode_configuration_array[i][2];
        }

        print_measured_flight_data(is_autopilot_engaged);
    } else if (len == 1 && incoming_message[0] == 3) {
        Serial.println("Reply message received: 3");
        is_configuration_struct_setup = true;
        Serial.println("\x1B[32mSuccess: Configuration setup completed.\x1B[0m");
        Serial.print("\x1B[2J\x1B[H");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.print("\x1B[2J\x1B[H");

    WiFi.mode(WIFI_STA);
    if (esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR) != ESP_OK) {
        Serial.print("\x1B[31mError: failed to set up Long Range Protocol.\x1B[0m");
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("\x1B[31mError: failed to initialise ESP-NOW.\x1B[0m");
    } else {
        Serial.println("\x1B[32mSuccess: Successfully initialised ESP-NOW Long Range Protocol.\x1B[0m");
    }

    esp_now_peer_info_t connection_with_glider_esp32_peer_info = {};
    memcpy(connection_with_glider_esp32_peer_info.peer_addr, glider_esp32_mac_address, 6);
    connection_with_glider_esp32_peer_info.channel = 0;
    connection_with_glider_esp32_peer_info.encrypt = false;
    if (esp_now_add_peer(&connection_with_glider_esp32_peer_info) == ESP_OK) {
        Serial.println("\x1B[32mSuccess: Successfuly added glider ESP32 as peer.\x1B[0m");
    } else {
        Serial.println("\x1B[31mError: failed to add glider ESP32 as peer.\x1B[0m");
    }

    ps5.begin();
    Serial.println("Bluetooth initialised.");
    Serial.print("Waiting for PS5 Controller...");

    esp_now_register_recv_cb(decode_execute_message_from_glider);
}

void loop() {
    if (ps5.isConnected() == true && is_ps5_controller_connected != true) {
        is_ps5_controller_connected = true;
        Serial.println();
        Serial.println("\x1B[32mSuccess: PS5 Controller successfully connected.\x1B[0m");
    } else if (ps5.isConnected() == false && is_ps5_controller_connected != false) {
        is_ps5_controller_connected = false;
        Serial.print("Waiting for PS5 Controller...");
    }

    if (is_configuration_struct_setup == false && millis() - previous_time_sent_packet_milliseconds >= time_delay_between_packet_request_milliseconds) {
        previous_time_sent_packet_milliseconds = millis();
        esp_now_send(glider_esp32_mac_address, (uint8_t*) &flight_config_1, sizeof(flight_config_1));
        delay(25);
        esp_now_send(glider_esp32_mac_address, (uint8_t*) &flight_config_2, sizeof(flight_config_2));
    }

    if (is_configuration_struct_setup == true && ps5.isConnected() == true) {
        delay(flight_config_2.dt_seconds * 1000);
        
        flight_config_1.decode_configuration_array[0][3] = round((1.0 - ((float) (ps5.lx + 128) / 255)) * 40 + 70);
        flight_config_1.decode_configuration_array[1][3] = round((1.0 - ((float) (ps5.ly + 128) / 255)) * 90 + 45);
        flight_config_1.decode_configuration_array[2][3] = round(((float) ps5.r2 / 255) * 180);
        flight_config_1.decode_configuration_array[3][3] = 0;
        flight_config_1.decode_configuration_array[4][3] = 0;
        flight_config_1.decode_configuration_array[5][3] = 0;
        flight_config_1.decode_configuration_array[6][3] = 0;
        
        encode_send_message(false, false, false);
    }
}