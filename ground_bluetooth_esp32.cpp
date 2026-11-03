#include <Arduino.h>
#include <ps5Controller.h>
#include <SimpleBLE.h>

const char* ssid = "ESP32-Flight_Control_System";
const char* password = "192837465";

uint32_t previous_time_sent_serial_data_milliseconds = 0;
uint16_t time_delay_between_serial_data_sent_milliseconds = 50;
bool is_ps5_controller_connected = false;
bool is_configuration_struct_setup = false;
bool is_resend_configuration_structs = false;

const uint8_t ground_uart2_rx_pin = 16;
const uint8_t ground_uart2_tx_pin = 17;

uint8_t new_aileron_servo_angle = 90;
uint8_t new_elevator_servo_angle = 90;
uint8_t new_throttle_servo_angle = 0;
int32_t new_measured_airspeed = 0;
int32_t new_measured_altitude = 0;
int32_t new_measured_heading = 0;
int32_t new_measured_vertical_speed = 0;
int32_t new_measured_signal_quality_score = 0;

HardwareSerial ground_serial(2);
SimpleBLE ble_serial;

void setup() {
    Serial.begin(115200);
    Serial.print("\x1B[2J\x1B[H");

    ps5.begin();
    Serial.println("Classic bluetooth initialised. Waiting for PS5 Controller...");

    ground_serial.begin(115200, SERIAL_8N1, ground_uart2_rx_pin, ground_uart2_tx_pin);
    ble_serial.begin("Measured Glider Flight Data");
    Serial.println("BLE initialised. Waiting for BLE connection.");
}

void loop() {
    if (ps5.isConnected() == true && is_ps5_controller_connected != true) {
        is_ps5_controller_connected = true;
        Serial.println("\x1B[32mSuccess: PS5 Controller successfully connected.\x1B[0m");
    } else if (ps5.isConnected() == false && is_ps5_controller_connected != false) {
        is_ps5_controller_connected = false;
        Serial.println("Waiting for PS5 Controller...");
    }

    if (ground_serial.available() > 0 && is_configuration_struct_setup == true) {
        int32_t new_measured_airspeed;
        int32_t new_measured_altitude;
        int32_t new_measured_heading;
        int32_t new_measured_vertical_speed;
        int32_t new_measured_signal_quality_score;
        String serial_message = ground_serial.readStringUntil('\n');
        sscanf(serial_message.c_str(), "%d,%d,%d,%d,%d", &new_measured_airspeed, &new_measured_altitude, &new_measured_heading, &new_measured_vertical_speed, &new_measured_signal_quality_score);
    }

    if (ground_serial.available() > 0 && is_configuration_struct_setup == false) {
        String serial_message = ground_serial.readStringUntil('\n');
        if (serial_message == "3") {
            is_configuration_struct_setup = true;
            is_resend_configuration_structs = false;
        }
    }

    if (ps5.isConnected() == true && is_configuration_struct_setup == true && millis() - previous_time_sent_serial_data_milliseconds >= time_delay_between_serial_data_sent_milliseconds) {
        previous_time_sent_serial_data_milliseconds = millis();

        new_aileron_servo_angle = round((1.0 - ((float) (ps5.lx + 128) / 255)) * 40 + 70);
        new_elevator_servo_angle = round((1.0 - ((float) (ps5.ly + 128) / 255)) * 90 + 45);
        new_throttle_servo_angle = round(((float) ps5.r2 / 255) * 180);
        is_resend_configuration_structs = ps5.share;
        if (is_resend_configuration_structs == true) {
            is_configuration_struct_setup = false;
        }

        ground_serial.printf("%u,%u,%u,%u\n", new_aileron_servo_angle, new_elevator_servo_angle, new_throttle_servo_angle, is_resend_configuration_structs);
    }
}