#include <Arduino.h>
#include <ps5Controller.h>
#include <cmath>

uint32_t previous_time_sent_serial_data_milliseconds = 0;
uint16_t time_delay_between_serial_data_sent_milliseconds = 50;
bool is_configuration_struct_setup = false;

int32_t new_aileron_servo_angle = 90;
int32_t new_elevator_servo_angle = 90;
float resultant_of_y_z_accelerometer;
float ps5_controller_physical_roll_angle;
int32_t new_rudder_servo_angle = 90;
int32_t new_throttle_servo_angle = 0;
int32_t is_resend_configuration_structs;
bool is_ps5_controller_l1_pressed = false;
bool is_ps5_controller_r1_pressed = false;
const float rudder_servo_angle_mode_1_sensitivity_multiplier = 0.333;
const float rudder_servo_angle_mode_2_sensitivity_multiplier = 1;
const float adverse_yaw_aileron_to_rudder_servo_angle_multiplier = 1;
bool is_adverse_yaw_assist = false;
uint8_t rudder_servo_mode = 0;

const uint8_t ground_uart2_rx_pin = 18;
const uint8_t ground_uart2_tx_pin = 19;

HardwareSerial ground_serial(2);

void setup() {
    ps5.begin();

    ground_serial.begin(115200, SERIAL_8N1, ground_uart2_rx_pin, ground_uart2_tx_pin);
}

void loop() {
    if (ground_serial.available() > 0 && is_configuration_struct_setup == false) {
        String serial_message = ground_serial.readStringUntil('\n');
        if (serial_message == "3") {
            is_configuration_struct_setup = true;
        }
    }

    if (ps5.isConnected() == true && is_configuration_struct_setup == true && millis() - previous_time_sent_serial_data_milliseconds >= time_delay_between_serial_data_sent_milliseconds) {
        previous_time_sent_serial_data_milliseconds = millis();
        
        is_ps5_controller_l1_pressed = false;
        is_ps5_controller_r1_pressed = false;
        if (ps5.l1.pressed == true) {
            is_ps5_controller_l1_pressed = true;
        }
        if (ps5.r1.pressed == true) {
            is_ps5_controller_r1_pressed = true;
        }

        if (is_ps5_controller_l1_pressed == false && is_ps5_controller_r1_pressed == true) {
            rudder_servo_mode = 0;
        } else if (is_ps5_controller_l1_pressed == true && is_ps5_controller_r1_pressed == false) {
            rudder_servo_mode = 1;
        } else if (is_ps5_controller_l1_pressed == true && is_ps5_controller_r1_pressed == true) {
            rudder_servo_mode = 2;
        }
        if (ps5.cross.pressed == true) {
            is_adverse_yaw_assist = true;
        } else if (ps5.circle.pressed == true) {
            is_adverse_yaw_assist = false;
        }

        new_aileron_servo_angle = constrain(round(((float) (ps5.rx + 128) / 255) * 40 + 70), 70, 110);
        new_elevator_servo_angle = constrain(round(((float) (ps5.ry + 128) / 255) * 90 + 45), 45, 135);
        resultant_of_y_z_accelerometer = pow(pow(ps5.accelY, 2) + pow(ps5.accelZ, 2), 0.5);
        if (resultant_of_y_z_accelerometer != 0 && rudder_servo_mode != 0) {
            ps5_controller_physical_roll_angle = (atan(ps5.accelX / resultant_of_y_z_accelerometer) * (180.0 / PI)) / -1;
            if (rudder_servo_mode == 1) {
                new_rudder_servo_angle = round(((constrain(ps5_controller_physical_roll_angle * rudder_servo_angle_mode_1_sensitivity_multiplier, -90, 90) + 90) / 180) * 120 + 30);
            } else if (rudder_servo_mode == 2) {
                new_rudder_servo_angle = round(((constrain(ps5_controller_physical_roll_angle * rudder_servo_angle_mode_2_sensitivity_multiplier, -90, 90) + 90) / 180) * 120 + 30);
            }
        } else {
            new_rudder_servo_angle = 90;
        }

        if (is_adverse_yaw_assist == true) {
            new_rudder_servo_angle = constrain(new_rudder_servo_angle + (new_aileron_servo_angle - 90) * adverse_yaw_aileron_to_rudder_servo_angle_multiplier, 30, 150);
        }
        //new_rudder_servo_angle = round((1.0 - ((float) (ps5.rx + 128) / 255)) * 120 + 30);
        new_throttle_servo_angle = round(((float) ps5.l2 / 255) * 180);
        is_resend_configuration_structs = ps5.share.pressed;
        if (is_resend_configuration_structs == true) {
            is_configuration_struct_setup = false;
        }

        ground_serial.printf("%d,%d,%d,%d,%d\n", new_aileron_servo_angle, new_elevator_servo_angle, new_rudder_servo_angle, new_throttle_servo_angle, is_resend_configuration_structs);
    }
}