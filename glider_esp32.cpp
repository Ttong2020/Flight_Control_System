#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include <map>
#include <vector>
#include <TinyGPS++.h>
#include <cmath>

uint8_t ground_esp32_mac_address[6] = {0x20, 0x9B, 0xA9, 0x67, 0xC7, 0x74};
uint32_t previous_time_received_packet_milliseconds = 0;
uint16_t maximum_safe_timeout_milliseconds = 500;
uint32_t previous_time_sent_packet_milliseconds = 0;
uint16_t time_delay_between_packet_request_milliseconds = 1000;
bool is_configuration_struct_1_filled = false;
bool is_configuration_struct_2_filled = false;
bool is_configuration_struct_setup = false;

// Configuration data to be received from ground ESP32.
struct configuration_struct_1_encode_decode_message_info {
    uint8_t packet_num;
    int32_t decode_configuration_array[8][4];
    uint8_t starting_index_for_decode_configuration_array_ground_to_glider;
    uint8_t ending_index_for_decode_configuration_array_ground_to_glider;
    uint8_t starting_index_for_decode_configuration_array_glider_to_ground;
    uint8_t ending_index_for_decode_configuration_array_glider_to_ground;
    uint8_t bytes_required_for_normal_operation_ground_to_glider;
    uint8_t bytes_required_for_normal_operation_glider_to_ground;
};

struct configuration_struct_2_other_info {
    uint8_t packet_num;
    uint8_t minimum_airspeed_before_autopilot_disengage;
    float pid_controller_configuration_array[4][7];
    float dt_seconds;
    uint16_t wind_direction_degrees;
    uint8_t wind_speed_metres_per_second;
    uint8_t aileron_servo_pin;
    uint8_t elevator_servo_pin;
    uint8_t throttle_esc_pin;
    uint8_t gps_uart2_rx_pin;
    uint8_t gps_uart2_tx_pin;
};

configuration_struct_1_encode_decode_message_info flight_config_1;
configuration_struct_2_other_info flight_config_2;

// Declaring servos/motor/gps/gps UART serial port.
Servo aileron_servo;
Servo elevator_servo;
Servo throttle_servo;
TinyGPSPlus gps;
HardwareSerial gps_serial(2);

// Proportional Integral Derivative (PID) Controller loop to enable autopilot parameters to reach target value as quickly and as less overshooting as possible (critically damped).
float pid_controller(uint8_t index_to_calculate) {
    float current_error = flight_config_2.pid_controller_configuration_array[index_to_calculate][5] - flight_config_2.pid_controller_configuration_array[index_to_calculate][6];
    float proportional_contribution = flight_config_2.pid_controller_configuration_array[index_to_calculate][0] * current_error;
    float integral_contribution = flight_config_2.pid_controller_configuration_array[index_to_calculate][1] * (flight_config_2.pid_controller_configuration_array[index_to_calculate][3] + (current_error * flight_config_2.dt_seconds));
    float derivative_contribution = flight_config_2.pid_controller_configuration_array[index_to_calculate][2] * ((current_error - flight_config_2.pid_controller_configuration_array[index_to_calculate][4]) / flight_config_2.dt_seconds);
    float output_control = proportional_contribution + integral_contribution + derivative_contribution;
    flight_config_2.pid_controller_configuration_array[index_to_calculate][3] = integral_contribution;
    flight_config_2.pid_controller_configuration_array[index_to_calculate][4] = current_error;

    return(output_control);
}

void encode_send_message_to_ground(const bool is_autopilot_engaged) {
    std::vector<uint8_t> message_to_send_in_byte_array(flight_config_1.bytes_required_for_normal_operation_glider_to_ground);

    for (uint8_t i = flight_config_1.starting_index_for_decode_configuration_array_glider_to_ground; i < flight_config_1.ending_index_for_decode_configuration_array_glider_to_ground + 1; i++) {
        flight_config_1.decode_configuration_array[i][3] = constrain((int32_t) round((float) (flight_config_1.decode_configuration_array[i][3] - flight_config_1.decode_configuration_array[i][1]) / (float) flight_config_1.decode_configuration_array[i][0]), 0, flight_config_1.decode_configuration_array[i][2]);
    }

    uint64_t total_value_to_send_base_256 = 0;
    uint64_t multiplication_of_number_of_possible_combination = 1;
    for (uint8_t i = flight_config_1.starting_index_for_decode_configuration_array_glider_to_ground; i < flight_config_1.ending_index_for_decode_configuration_array_glider_to_ground + 1; i++) {
        total_value_to_send_base_256 = total_value_to_send_base_256 + flight_config_1.decode_configuration_array[i][3] * multiplication_of_number_of_possible_combination;
        multiplication_of_number_of_possible_combination = multiplication_of_number_of_possible_combination * (uint64_t) flight_config_1.decode_configuration_array[i][2];
    }

    total_value_to_send_base_256 = (total_value_to_send_base_256 << 1) | ((is_autopilot_engaged ? 1 : 0) << 0);

    for (uint8_t i = 0; i < flight_config_1.bytes_required_for_normal_operation_glider_to_ground; i++) {
        message_to_send_in_byte_array[i] = (total_value_to_send_base_256 >> (8 * i)) & 0xFF;
    }
    
    esp_now_send(ground_esp32_mac_address, message_to_send_in_byte_array.data(), message_to_send_in_byte_array.size());
}

// Function to decode message from incoming message, and execute them through servos/motor.
void decode_execute_message_from_ground(const uint8_t* ground_esp32_mac_address, const uint8_t* incoming_message, int len) {
    if (len == flight_config_1.bytes_required_for_normal_operation_ground_to_glider) {
        previous_time_received_packet_milliseconds = millis();

        uint64_t total_value_received_base_256 = 0;
        for (uint8_t i = 0; i < flight_config_1.bytes_required_for_normal_operation_ground_to_glider; i++) {
            uint64_t value_at_byte_index_i = incoming_message[i];
            total_value_received_base_256 = total_value_received_base_256 + (value_at_byte_index_i << (8 * i));
        }

        uint8_t flag_bits = total_value_received_base_256 & 0x07;
        bool is_heading_autopilot = (total_value_received_base_256 >> 2) & 0x01;
        bool is_altitude_autopilot = (total_value_received_base_256 >> 1) & 0x01;
        bool is_airspeed_autopilot = (total_value_received_base_256 >> 0) & 0x01;
        total_value_received_base_256 = total_value_received_base_256 >> 3;
        
        uint64_t multiplication_of_number_of_possible_value = 1;
        for (uint8_t i = flight_config_1.starting_index_for_decode_configuration_array_ground_to_glider; i < flight_config_1.ending_index_for_decode_configuration_array_ground_to_glider + 1; i++) {
            int16_t value_sent_at_index_i = (total_value_received_base_256 / multiplication_of_number_of_possible_value) % flight_config_1.decode_configuration_array[i][2];
            flight_config_1.decode_configuration_array[i][3] = value_sent_at_index_i * flight_config_1.decode_configuration_array[i][0] + flight_config_1.decode_configuration_array[i][1];
            multiplication_of_number_of_possible_value = multiplication_of_number_of_possible_value * (uint64_t) flight_config_1.decode_configuration_array[i][2];
        }

        bool is_autopilot_engaged = false;
        if (flight_config_2.pid_controller_configuration_array[0][6] >= (float) flight_config_2.minimum_airspeed_before_autopilot_disengage) {
            if (is_heading_autopilot == true) {
                flight_config_2.pid_controller_configuration_array[2][5] = flight_config_1.decode_configuration_array[5][3];
                flight_config_1.decode_configuration_array[0][3] = pid_controller(2) + 90;
                is_autopilot_engaged = true;
            }
            if (is_altitude_autopilot == true) {
                flight_config_2.pid_controller_configuration_array[3][5] = flight_config_1.decode_configuration_array[6][3];
                flight_config_1.decode_configuration_array[1][3] = pid_controller(3) + 90;
                is_autopilot_engaged = true;
            }
            if (is_airspeed_autopilot == true) {
                flight_config_2.pid_controller_configuration_array[0][5] = flight_config_1.decode_configuration_array[3][3];
                flight_config_1.decode_configuration_array[2][3] = pid_controller(0);
                is_autopilot_engaged = true;
            }
        }

        aileron_servo.write(constrain(flight_config_1.decode_configuration_array[0][3], 70, 110));
        elevator_servo.write(constrain(flight_config_1.decode_configuration_array[1][3], 45, 135));
        throttle_servo.write(constrain(flight_config_1.decode_configuration_array[2][3], 0, 180));

        encode_send_message_to_ground(is_autopilot_engaged);
    }
}

void decode_configuration_info_message_from_ground(const uint8_t* ground_esp_mac_address, const uint8_t* incoming_message, int len) {
    if (len == sizeof(flight_config_1) && incoming_message[0] == 0x0A && is_configuration_struct_1_filled == false) {
        memcpy(&flight_config_1, incoming_message, sizeof(flight_config_1));
        is_configuration_struct_1_filled = true;
        Serial.println("Packet 1 received.");
    } else if (len == sizeof(flight_config_2) && incoming_message[0] == 0x14 && is_configuration_struct_2_filled == false) {
        memcpy(&flight_config_2, incoming_message, sizeof(flight_config_2));
        is_configuration_struct_2_filled = true;
        Serial.println("Packet 2 received.");
    }
}

void servo_motor_gps_serial_setup() {
    gps_serial.begin(9600, SERIAL_8N1, flight_config_2.gps_uart2_rx_pin, flight_config_2.gps_uart2_tx_pin);
    Serial.println("Success: Successfully initialised GPS UART2 Serial port.");

    ESP32PWM::allocateTimer(0);
    aileron_servo.setPeriodHertz(50);
    aileron_servo.attach(flight_config_2.aileron_servo_pin, 500, 2400);
    aileron_servo.write(90);

    ESP32PWM::allocateTimer(1);
    elevator_servo.setPeriodHertz(50);
    elevator_servo.attach(flight_config_2.elevator_servo_pin, 500, 2400);
    elevator_servo.write(90);

    ESP32PWM::allocateTimer(2);
    throttle_servo.setPeriodHertz(50);
    throttle_servo.attach(flight_config_2.throttle_esc_pin, 1000, 2000);
    throttle_servo.write(0);
}

// This function runs first. It configures the serial monitor, servos, motor, ESP32-NOW Long Range Protocol. It tells ESP-NOW to call the function 'decode_execute_message_from_ground_esp32' whenever a message is received.
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

    esp_now_peer_info_t connection_with_ground_esp_peer_info = {};
    memcpy(connection_with_ground_esp_peer_info.peer_addr, ground_esp32_mac_address, 6);
    connection_with_ground_esp_peer_info.channel = 0;
    connection_with_ground_esp_peer_info.encrypt = false;
    if (esp_now_add_peer(&connection_with_ground_esp_peer_info) == ESP_OK) {
        Serial.println("Success: Successfully added ground ESP32 as peer.");
    } else {
        Serial.println("Error: failed to add ground ESP32 as peer.");
    }

    esp_now_register_recv_cb(decode_configuration_info_message_from_ground);
}

// This function runs after te funtion 'setup'.
void loop() {
    if (is_configuration_struct_setup == false || millis() - previous_time_received_packet_milliseconds > maximum_safe_timeout_milliseconds) {
        aileron_servo.write(90);
        elevator_servo.write(90);
        throttle_servo.write(0);
    }
    
    if (is_configuration_struct_setup == false) {
        if (is_configuration_struct_1_filled == true && is_configuration_struct_2_filled == true) {
            const uint8_t reply_message = 3;
            esp_now_send(ground_esp32_mac_address, &reply_message, 1);
            esp_now_register_recv_cb(decode_execute_message_from_ground);
            maximum_safe_timeout_milliseconds = (uint16_t) (flight_config_2.dt_seconds * 1000) * 10;
            is_configuration_struct_setup = true;
            servo_motor_gps_serial_setup();
            Serial.println("Success message sent.");
        } else if (millis() - previous_time_sent_packet_milliseconds >= time_delay_between_packet_request_milliseconds) {
            if (is_configuration_struct_1_filled == false) {
                const uint8_t reply_message = 1;
                esp_now_send(ground_esp32_mac_address, &reply_message, 1);
                previous_time_sent_packet_milliseconds = millis();
                Serial.println("Packet 1 request.");
            } else if (is_configuration_struct_2_filled == false) {
                const uint8_t reply_message = 2;
                esp_now_send(ground_esp32_mac_address, &reply_message, 1);
                previous_time_sent_packet_milliseconds = millis();
                Serial.println("Packet 2 request.");
            }
        }
    } else if (is_configuration_struct_setup == true) {
        while (gps_serial.available() > 0) {
            gps.encode(gps_serial.read());
        }
        
        // Need fixing: if (gps.location.isValid()) returns false, then program resumes and pid controller loop uses wrong measured value.
        if (gps.location.isValid()) {
            flight_config_2.pid_controller_configuration_array[0][6] = gps.speed.mps() - flight_config_2.wind_speed_metres_per_second * cos((flight_config_2.wind_direction_degrees - gps.course.deg()) * (PI / 180));
            flight_config_2.pid_controller_configuration_array[1][6] = gps.altitude.meters();
            flight_config_2.pid_controller_configuration_array[2][6] = gps.course.deg();
            Serial.print("Ground speed: ");
            Serial.println(gps.speed.mps());
            Serial.print("Altitude: ");
            Serial.println(gps.altitude.meters());
            Serial.print("Course: ");
            Serial.println(gps.course.deg());
        }
    }
}