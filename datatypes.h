#ifndef DATATYPES_H
#define DATATYPES_H

#include "ch.h"


typedef enum {
  MOTOR_STATUS_OFF = 0,
  MOTOR_STATUS_DETECTING = 1,
  MOTOR_STATUS_RUNNING = 1,
  MOTOR_STATUS_FULL_BRAKE = 2,
} motor_status_t;

typedef enum {
  MOTOR_CONTROL_MODE_DUTY = 0,
  MOTOR_CONTROL_MODE_SPEED,
  MOTOR_CONTROL_MODE_CURRENT,
  MOTOR_CONTROL_MODE_CURRENT_BRAKE,
  MOTOR_CONTROL_MODE_POS,
  MOTOR_CONTROL_MODE_HANDBRAKE,
  MOTOR_CONTROL_MODE_OPENLOOP,
  MOTOR_CONTROL_MODE_OPENLOOP_PHASE,
  MOTOR_CONTROL_MODE_OPENLOOP_DUTY,
  MOTOR_CONTROL_MODE_OPENLOOP_DUTY_PHASE,
  MOTOR_CONTROL_MODE_NONE,
} motor_control_mode_t;

typedef enum {
  MODULATION_SINE_PWM = 0,
  MODULATION_SVPWM,
} modulation_mode_t;

typedef enum {
  CURRENT_SENSING_TYPE_LOW_SIDE = 0,
  CURRENT_SENSING_TYPE_IN_LINE,
} current_sensing_mode_t;

typedef struct {
    float target_position_deg;  // Position in degrees
    float target_speed_deg_s;     // Speed in deg/s
    uint16_t phases_current_raw[3];
    float phases_current[3];
    float q_target_amperes;
    float d_target_amperes;
    float d;
    float q;
    float q_voltage;
    float d_voltage;
    float voltage_angle_deg;
    float raw_angle_deg;
    float voltage_mag;
    float current_mag;
    float duty;
    uint16_t phase_duty[3];
    
} motor_state_t;

typedef struct {
    uint8_t num_poles_pairs;
    uint16_t pid_frequency;
    float battery_voltage;
    modulation_mode_t modulation_mode;
} motor_config_t;

typedef struct {
    float gain;
    float r_sense;
    uint16_t voltage_raw_offset;
    current_sensing_mode_t current_sensing_mode;
} current_sensing_config_t;

typedef struct {
  motor_status_t motor_status;
  motor_control_mode_t motor_control_mode;
  motor_config_t motor_config;
  current_sensing_config_t current_sensing_config;
  motor_state_t motor_state;
} motor_t;

#endif