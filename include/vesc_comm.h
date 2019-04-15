/*
    Copyright 2018 Jan Pomikalek <jan.pomikalek@gmail.com>
*/
#include <Arduino.h>

#ifndef VESC_COMM_H
#define VESC_COMM_H

#define PACKET_MAX_LENGTH 70

typedef enum {
    FAULT_CODE_NONE = 0,
    FAULT_CODE_OVER_VOLTAGE,
    FAULT_CODE_UNDER_VOLTAGE,
    FAULT_CODE_DRV,
    FAULT_CODE_ABS_OVER_CURRENT,
    FAULT_CODE_OVER_TEMP_FET,
    FAULT_CODE_OVER_TEMP_MOTOR
} vesc_comm_fault_code;

void vesc_comm_init(uint32_t baud);
uint8_t vesc_comm_fetch_packet(uint8_t *vesc_packet, uint16_t timeout = 100);
uint8_t vesc_comm_receive_packet(uint8_t *vesc_packet, uint16_t timeout);
bool vesc_comm_is_expected_packet(uint8_t *vesc_packet, uint8_t packet_length);
float vesc_comm_get_temp_mosfet(uint8_t *vesc_packet);
float vesc_comm_get_temp_motor(uint8_t *vesc_packet);
float vesc_comm_get_motor_current(uint8_t *vesc_packet);
float vesc_comm_get_battery_current(uint8_t *vesc_packet);
float vesc_comm_get_duty_cycle(uint8_t *vesc_packet);
int32_t vesc_comm_get_rpm(uint8_t *vesc_packet);
float vesc_comm_get_voltage(uint8_t *vesc_packet);
float vesc_comm_get_amphours_discharged(uint8_t *vesc_packet);
float vesc_comm_get_amphours_charged(uint8_t *vesc_packet);
int32_t vesc_comm_get_tachometer(uint8_t *vesc_packet);
int32_t vesc_comm_get_tachometer_abs(uint8_t *vesc_packet);
// vesc_comm_fault_code vesc_comm_get_fault_code(uint8_t *vesc_packet);

#endif //VESC_COMM_H
