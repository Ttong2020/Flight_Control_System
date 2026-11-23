#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <vector>

constexpr uint8_t glider_esp32_mac_address[6] = {0x88, 0xF1, 0x55, 0x31, 0x07, 0x40};
uint32_t previous_time_sent_packet_milliseconds = 0;
uint16_t time_delay_between_sending_packets_milliseconds = 50;
bool is_configuration_struct_setup = false;

const uint8_t ground_bluetooth_uart2_rx_pin = 19;
const uint8_t ground_bluetooth_uart2_tx_pin = 18;
const uint8_t ground_website_uart1_rx_pin = 23;
const uint8_t ground_website_uart1_tx_pin = 22;

HardwareSerial ground_bluetooth_serial(2);
HardwareSerial ground_website_serial(1);

// configuration data to send to glider ESP32.
struct configuration_struct_1_encode_decode_message_info {
    uint8_t packet_num = 10;
    // {value to multiply, value to add, number of possible value, new value}.
    int32_t decode_configuration_array[9][4] = {{1, 60, 61, 90}, // new_aileron_servo_angle
                                                {1, 45, 91, 90}, // new_elevator_servo_angle
                                                {1, 30, 121, 90}, // new_rudder_servo_angle
                                                {1, 0, 181, 0}, // new_throttle_servo_angle
                                                {1, 0, 101, 0}, // new_target_airspeed
                                                {1, 0, 601, 0}, // new_target_altitude
                                                {1, 0, 361, 0}, // new_target_heading
                                                {1, -600, 1201, 0}, //new_vertical_speed
                                                {1, 0, 11, 0} // new_signal_quality_score
                                               };
    uint8_t starting_index_for_decode_configuration_array_ground_to_glider = 0;
    uint8_t ending_index_for_decode_configuration_array_ground_to_glider = 3;
    uint8_t starting_index_for_decode_configuration_array_glider_to_ground = 4;
    uint8_t ending_index_for_decode_configuration_array_glider_to_ground = 8;
    uint8_t bytes_required_for_normal_operation_ground_to_glider = 7;
    uint8_t bytes_required_for_normal_operation_glider_to_ground = 5;
};

struct configuration_struct_2_other_info {
    uint8_t packet_num = 20;
    uint8_t minimum_airspeed_before_autopilot_disengage;
    // {kp, ki, kd, sum of past integrals, previous error, target value, measured value}.
    float pid_controller_configuration_array[3][7] = {{1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                      {1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                                      {1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}
                                                     };
    float dt_seconds = 0.05;
    uint16_t wind_direction_degrees;
    uint8_t wind_speed_km_per_hour;
    uint8_t aileron_servo_pin = 25;
    uint8_t elevator_servo_pin = 26;
    uint8_t rudder_servo_pin = 27;
    uint8_t throttle_esc_pin = 21;
    uint8_t gps_uart2_rx_pin = 16;
    uint8_t gps_uart2_tx_pin = 17;
};

configuration_struct_1_encode_decode_message_info flight_config_1;
configuration_struct_2_other_info flight_config_2;

//std::vector<uint8_t> message_to_send_in_byte_array(flight_config_1.bytes_required_for_normal_operation_ground_to_glider);
uint8_t message_to_send_in_byte_array[7];

void encode_send_message(const bool is_airspeed_autopilot, const bool is_altitude_autopilot, const bool is_heading_autopilot) {
    
    for (uint8_t i = flight_config_1.starting_index_for_decode_configuration_array_ground_to_glider; i < flight_config_1.ending_index_for_decode_configuration_array_ground_to_glider + 1; i++) {
        flight_config_1.decode_configuration_array[i][3] = constrain((int32_t) round((float) (flight_config_1.decode_configuration_array[i][3] - flight_config_1.decode_configuration_array[i][1]) / (float) flight_config_1.decode_configuration_array[i][0]), 0, flight_config_1.decode_configuration_array[i][2] - 1);
    }

    uint64_t total_value_to_send_base_256 = 0;
    uint64_t multiplication_of_number_of_possible_combination = 1;
    for (uint8_t i = flight_config_1.starting_index_for_decode_configuration_array_ground_to_glider; i < flight_config_1.ending_index_for_decode_configuration_array_ground_to_glider + 1; i++) {
        total_value_to_send_base_256 = total_value_to_send_base_256 + flight_config_1.decode_configuration_array[i][3] * multiplication_of_number_of_possible_combination;
        multiplication_of_number_of_possible_combination = multiplication_of_number_of_possible_combination * (uint64_t) flight_config_1.decode_configuration_array[i][2];
    }

    total_value_to_send_base_256 = (total_value_to_send_base_256 << 3) | ((is_airspeed_autopilot ? 1 : 0) << 2) | ((is_altitude_autopilot ? 1 : 0) << 1) | ((is_heading_autopilot ? 1 : 0) << 0);

    for (uint8_t i = 0; i < flight_config_1.bytes_required_for_normal_operation_ground_to_glider; i++) {
        message_to_send_in_byte_array[i] = (total_value_to_send_base_256 >> (8 * i)) & 0xFF;
    }
    
    esp_now_send(glider_esp32_mac_address, message_to_send_in_byte_array/*.data()*/, sizeof(message_to_send_in_byte_array)); //changed
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

        ground_website_serial.printf("%d,%d,%d,%d,%d\n", flight_config_1.decode_configuration_array[4][3], flight_config_1.decode_configuration_array[5][3], flight_config_1.decode_configuration_array[6][3], flight_config_1.decode_configuration_array[7][3], flight_config_1.decode_configuration_array[8][3]);

    } else if (len == 1 && incoming_message[0] == 3) {
        is_configuration_struct_setup = true;
        ground_bluetooth_serial.printf("3\n");
    }
}

void setup() {
    WiFi.mode(WIFI_STA);
    if (esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR) != ESP_OK) {
        esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
    }

    if (esp_now_init() != ESP_OK) {
        esp_now_init();
    }

    esp_now_peer_info_t connection_with_glider_esp32_peer_info = {};
    memcpy(connection_with_glider_esp32_peer_info.peer_addr, glider_esp32_mac_address, 6);
    connection_with_glider_esp32_peer_info.channel = 0;
    connection_with_glider_esp32_peer_info.encrypt = false;
    if (esp_now_add_peer(&connection_with_glider_esp32_peer_info) != ESP_OK) {
        esp_now_add_peer(&connection_with_glider_esp32_peer_info);
    }

    ground_bluetooth_serial.begin(115200, SERIAL_8N1, ground_bluetooth_uart2_rx_pin, ground_bluetooth_uart2_tx_pin);
    ground_website_serial.begin(115200, SERIAL_8N1, ground_website_uart1_rx_pin, ground_website_uart1_tx_pin);
    
    esp_now_register_recv_cb(decode_execute_message_from_glider);
    Serial.begin(115200);
}

void loop() {
    if (is_configuration_struct_setup == false && millis() - previous_time_sent_packet_milliseconds >= time_delay_between_sending_packets_milliseconds) {
        previous_time_sent_packet_milliseconds = millis();
        esp_now_send(glider_esp32_mac_address, (uint8_t*) &flight_config_1, sizeof(flight_config_1));
        delay(25);
        esp_now_send(glider_esp32_mac_address, (uint8_t*) &flight_config_2, sizeof(flight_config_2));
    }

    if (ground_bluetooth_serial.available() > 0 && is_configuration_struct_setup == true) {
        int32_t new_aileron_servo_angle;
        int32_t new_elevator_servo_angle;
        int32_t new_rudder_servo_angle;
        int32_t new_throttle_servo_angle;
        int32_t is_resend_configuration_structs;
        String serial_message = ground_bluetooth_serial.readStringUntil('\n');
        sscanf(serial_message.c_str(), "%d,%d,%d,%d,%d", &new_aileron_servo_angle, &new_elevator_servo_angle, &new_rudder_servo_angle, &new_throttle_servo_angle, &is_resend_configuration_structs);

        Serial.print(new_aileron_servo_angle);
        Serial.print(", ");
        Serial.print(new_elevator_servo_angle);
        Serial.print(", ");
        Serial.print(new_rudder_servo_angle);
        Serial.print(", ");
        Serial.println(new_throttle_servo_angle);

        if (is_resend_configuration_structs == true) {
            is_configuration_struct_setup = false;
        } else {
            Serial.println(new_aileron_servo_angle);
            flight_config_1.decode_configuration_array[0][3] = new_aileron_servo_angle;
            flight_config_1.decode_configuration_array[1][3] = new_elevator_servo_angle;
            flight_config_1.decode_configuration_array[2][3] = new_rudder_servo_angle;
            flight_config_1.decode_configuration_array[3][3] = new_throttle_servo_angle;
            flight_config_1.decode_configuration_array[4][3] = 0;
            flight_config_1.decode_configuration_array[5][3] = 0;
            flight_config_1.decode_configuration_array[6][3] = 0;

            Serial.print(flight_config_1.decode_configuration_array[0][3]);
            Serial.print(", ");
            Serial.print(flight_config_1.decode_configuration_array[1][3]);
            Serial.print(", ");
            Serial.print(flight_config_1.decode_configuration_array[2][3]);
            Serial.print(", ");
            Serial.println(flight_config_1.decode_configuration_array[3][3]);
            
            encode_send_message(false, false, false);
        }
    }
}