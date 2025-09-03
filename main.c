#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "datatypes.h"
#include "shell.h"
#include "shell_cmd.h"
#include <math.h>
#include <stdlib.h>
#include "mb_slave.h"

/*===========================================================================*/
/* Math Utility Macros                                                       */
/*===========================================================================*/
#define RPM_TO_RAD_S(rpm) ((rpm) * 0.104719755f)    // 2π/60 = 0.104719755
#define RAD_S_TO_RPM(rad_s) ((rad_s) * 9.549296586f) // 60/2π = 9.549296586
#define DEG_S_TO_RPM(deg_s) ((deg_s) * 0.174532925f) // 60/360 = 0.174532925
#define DEG_TO_RAD(deg) ((deg) * 0.0174532925f) // π/180 = 0.0174532925
#define RAD_TO_DEG(rad) ((rad) * 57.295779513f) // 180/π = 57.295779513
#define DEG_TO_RAD_S(deg_s) ((deg_s) * 0.0174532925f) // π/180 = 0.0174532925
#define RAD_S_TO_DEG_S(rad_s) ((rad_s) * 57.295779513f) // 180/π = 57.295779513
#define TWO_BY_3 (0.666666667f)      // 2/3 = 0.666666667
#define SQRT_3_BY_2 (0.866025404f)   // √3/2 = 0.866025404
#define SQRT3 (1.732050808f)         // √3 = 1.732050808

// Low pass filter: output = alpha * input + (1-alpha) * previous_output
// alpha = dt / (dt + tau), where tau is time constant
#define LPF(last_output, input, alpha) ((alpha) * (input) + (1.0f - (alpha)) * (last_output))

// Normalize angle to [-180, 180] degrees range using float operations
#define NORM_ANGLE_180(angle) \
  (angle - 360.0f * floorf((angle + 180.0f) / 360.0f))

// Clamp value between min and max
static inline float math_clamp(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

/*===========================================================================*/
/* Motor                                                                     */
/*===========================================================================*/

static motor_t motor;

static motor_t* motor_get_motor(void) {
  return &motor;
}

static void motor_init(void) {
  motor.motor_status = MOTOR_STATUS_OFF;
  motor.motor_control_mode = MOTOR_CONTROL_MODE_NONE;
  motor.motor_config.num_poles_pairs = 7;
  motor.motor_config.pid_frequency = 1000;
  motor.motor_config.battery_voltage = 12;
  motor.motor_config.modulation_mode = MODULATION_SVPWM; // Default to sine PWM
  motor.current_sensing_config.gain = 12.22f;
  motor.current_sensing_config.r_sense = 0.005f;
  motor.current_sensing_config.voltage_raw_offset = 2047;
  motor.current_sensing_config.current_sensing_mode = CURRENT_SENSING_TYPE_LOW_SIDE;
}

static void pwm_set_duty(float duty_a, float duty_b, float duty_c);
static void motor_set_phase_voltages(float cos_theta_rad, float sin_theta_rad, float v_d, float v_q) {
  motor_get_motor()->motor_state.q_voltage = v_d;
  motor_get_motor()->motor_state.d_voltage = v_q;
  motor_get_motor()->motor_state.voltage_mag = sqrtf(v_d * v_d + v_q * v_q);
  motor_get_motor()->motor_state.voltage_angle_deg = NORM_ANGLE_180(RAD_TO_DEG(atan2f(v_q, v_d)));
  switch (motor.motor_config.modulation_mode) {
    case MODULATION_SINE_PWM:
      motor_get_motor()->motor_state.duty = motor_get_motor()->motor_state.voltage_mag / (motor.motor_config.battery_voltage / 2);
      break;
    case MODULATION_SVPWM:
      motor_get_motor()->motor_state.duty = motor_get_motor()->motor_state.voltage_mag / (motor.motor_config.battery_voltage / SQRT3);
      break;
    default:
      break;
  }

  // === SECTION 5: Inverse Park Transform ===
  float v_alpha = v_d * cos_theta_rad - v_q * sin_theta_rad;
  float v_beta = v_d * sin_theta_rad + v_q * cos_theta_rad;
  
  // === SECTION 6: Inverse Clarke Transform & PWM Modulation ===
  float temp_va = v_alpha;
  float temp_vb = -0.5f * v_alpha + SQRT_3_BY_2 * v_beta;
  float temp_vc = -0.5f * v_alpha - SQRT_3_BY_2 * v_beta;

  float mid_point = motor.motor_config.battery_voltage / 2.0f;
  
  // Apply modulation mode
  if (motor.motor_config.modulation_mode == MODULATION_SVPWM) {
    // SVPWM modulation with center alignment
    float Umin = temp_va;
    if (temp_vb < Umin) Umin = temp_vb;
    if (temp_vc < Umin) Umin = temp_vc;
    
    float Umax = temp_va;
    if (temp_vb > Umax) Umax = temp_vb;
    if (temp_vc > Umax) Umax = temp_vc;

    mid_point -= (Umin + Umax) / 2.0f;
  }

  float va = (temp_va + mid_point);
  float vb = (temp_vb + mid_point);
  float vc = (temp_vc + mid_point);
  
  // === SECTION 7: PWM Conversion ===
  float duty_a_pwm = va / motor.motor_config.battery_voltage;
  float duty_b_pwm = vb / motor.motor_config.battery_voltage;
  float duty_c_pwm = vc / motor.motor_config.battery_voltage;
  
  // Set PWM duty cycles directly
  pwm_set_duty(duty_a_pwm, duty_b_pwm, duty_c_pwm);
}

/*===========================================================================*/
/* PID Controller                                                            */
/*===========================================================================*/

// PID Controller
typedef struct {
  // Gains
  float kp;                   // Proportional gain
  float ki;                   // Integral gain
  float kd;                   // Derivative gain
  
  // State variables
  float prev_error;           // Previous error for derivative calculation
  float prev_input;           // Previous input for derivative calculation
  float integral;             // Accumulated integral
  float dt;                   // Sample time (seconds)
  
  // Output limits
  float output_min;           // Minimum output limit
  float output_max;           // Maximum output limit
  
  // Integral limits (anti-windup protection)
  float integral_max;         // Maximum allowed integral value

  // Output
  float output;               // last output value
} pid_controller_t;

static void pid_controller_init(pid_controller_t *pid, float kp, float ki, float kd, float dt, float output_min, float output_max, float integral_max) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->dt = dt;
  pid->output_min = output_min;
  pid->integral_max = integral_max;
  pid->prev_error = 0;
  pid->prev_input = 0;
  pid->integral = 0;
  pid->output_min = output_min;
  pid->output_max = output_max;
  pid->output = 0;
}

static float pid_controller_update(pid_controller_t *pid, float target, float input) {

  float error = target - input;
  pid->integral += error * pid->dt;

  float derivative = -(input - pid->prev_input) / pid->dt;  // Derivative on measurement to avoid derivative kick
  float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

  // Output limits
  if (output > pid->output_max) {
    output = pid->output_max;
  }
  if (output < pid->output_min) {
    output = pid->output_min;
  }

  // Anti-windup protection - limit integral only if output is saturated
  if (output >= pid->output_max || output <= pid->output_min) {
    if (pid->integral > pid->integral_max) {
      pid->integral = pid->integral_max;
    }
    if (pid->integral < -(pid->integral_max)) {
      pid->integral = -(pid->integral_max);
    }
  }

  pid->prev_error = error;
  pid->prev_input = input;
  pid->output = output;  // Store the output for get_output function
  return output;
}

static void pid_controller_reset(pid_controller_t *pid) {
  pid->prev_error = 0;
  pid->prev_input = 0;
  pid->integral = 0;
}

static void pid_controller_set_gains(pid_controller_t *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

static void pid_controller_get_gains(pid_controller_t *pid, float *kp, float *ki, float *kd) {
  *kp = pid->kp;
  *ki = pid->ki;
  *kd = pid->kd;
}

static void pid_controller_set_output_limits(pid_controller_t *pid, float output_min, float output_max) {
  pid->output_min = output_min;
  pid->output_max = output_max;
}

static void pid_controller_set_integral_limits(pid_controller_t *pid, float integral_max) {
  pid->integral_max = integral_max;
}

static float pid_controller_get_output(pid_controller_t *pid) {
  return pid->output;
}

/*===========================================================================*/
/* Current Sensor                                                            */
/*===========================================================================*/
#define CURRENT_SENSING_NUM_PHASES 3


typedef struct {
  float current_phases[CURRENT_SENSING_NUM_PHASES];
  uint16_t voltage_raw[CURRENT_SENSING_NUM_PHASES];
  uint16_t voltage_raw_offset[CURRENT_SENSING_NUM_PHASES];
  uint8_t current_direction[CURRENT_SENSING_NUM_PHASES];
  float gain;
  float r_sense;
  current_sensing_mode_t current_sensing_mode;
} current_sensing_t;

static void current_sensing_init(current_sensing_t *current_sensing, float gain, float r_sense, uint16_t voltage_raw_offset, current_sensing_mode_t current_sensing_mode) {
  for (int i = 0; i < CURRENT_SENSING_NUM_PHASES; i++) {
    current_sensing->current_phases[i] = 0;
    current_sensing->voltage_raw[i] = 0;
    current_sensing->voltage_raw_offset[i] = voltage_raw_offset;
    current_sensing->current_direction[i] = 0;
    current_sensing->current_sensing_mode = current_sensing_mode;
  }
  current_sensing->gain = gain;
  current_sensing->r_sense = r_sense;
}

static void current_sensing_update(current_sensing_t *current_sensing, uint16_t ia, uint16_t ib, uint16_t ic) {
  current_sensing->voltage_raw[0] = ia;
  current_sensing->voltage_raw[1] = ib;
  current_sensing->voltage_raw[2] = ic;

  int16_t current_raw[CURRENT_SENSING_NUM_PHASES];
  for (int i = 0; i < CURRENT_SENSING_NUM_PHASES; i++) {
    // Fix: Cast to signed before subtraction to handle underflow correctly
    current_raw[i] = (int16_t)(current_sensing->voltage_raw[i] - current_sensing->voltage_raw_offset[i]);
  }

  for (int i = 0; i < CURRENT_SENSING_NUM_PHASES; i++) {
    // Fix: Proper current conversion with correct scaling
    // Convert ADC count to voltage difference, then to current
    float voltage_diff = (float)current_raw[i] * (3.3f / 4096.0f);
    current_sensing->current_phases[i] = voltage_diff / (current_sensing->gain * current_sensing->r_sense);
  }
}

static void current_sensing_get_current_phases(current_sensing_t *current_sensing, float *ia, float *ib, float *ic) {
  *ia = current_sensing->current_phases[0];
  *ib = current_sensing->current_phases[1];
  *ic = current_sensing->current_phases[2];
}

/*===========================================================================*/
/* FOC Controller                                                            */
/*===========================================================================*/
typedef void (*foc_set_phase_voltages_func_t)(float cos_theta_rad, float sin_theta_rad, float v_d, float v_q);
static pid_controller_t foc_pid_q_controller;
static pid_controller_t foc_pid_d_controller;
static foc_set_phase_voltages_func_t foc_set_phase_voltages_func;

static void foc_init(uint16_t switching_frequency, uint16_t battery_voltage, foc_set_phase_voltages_func_t set_phase_voltages_func) { 
  pid_controller_init(&foc_pid_q_controller, 2.0f, 0.0f, 0.0f, 1.0 / switching_frequency, -battery_voltage/2, battery_voltage/2, 1);
  pid_controller_init(&foc_pid_d_controller, 0.5f, 0.0f, 0.0f, 1.0 / switching_frequency, -battery_voltage/2, battery_voltage/2, 1);
  foc_set_phase_voltages_func = set_phase_voltages_func;
}



void foc_update(float electrical_angle_degrees, float electrical_rpm, float q_ref_amperes, float ia_current, float ib_current, float ic_current) {
  // Check if function pointer is set
  if (foc_set_phase_voltages_func == NULL) {
      return; // Hardware function not initialized
  }

  (void)electrical_rpm;
  electrical_angle_degrees = NORM_ANGLE_180(electrical_angle_degrees);
  float electrical_angle_rad = DEG_TO_RAD(electrical_angle_degrees);
  // float electrical_rpm_rad_s = RPM_TO_RAD_S(electrical_rpm);  // Unused variable

  // Current values are now passed directly from the current sensor module
  float ia_f = ia_current;
  float ib_f = ib_current;
  float ic_f = ic_current;

  // === SECTION 1: Clarke Transform ===
  float alpha = TWO_BY_3 * (ia_f - 0.5f * ib_f - 0.5f * ic_f);
  float beta = TWO_BY_3 * (SQRT_3_BY_2 * (ib_f - ic_f));

  // === SECTION 2: Park Transform ===
  float cos_theta_rad = cosf(electrical_angle_rad);
  float sin_theta_rad = sinf(electrical_angle_rad);
  float d = alpha * cos_theta_rad + beta * sin_theta_rad;
  float q = -alpha * sin_theta_rad + beta * cos_theta_rad;
  
  motor_get_motor()->motor_state.q = q;
  motor_get_motor()->motor_state.d = d;
  motor_get_motor()->motor_state.current_mag = sqrtf(d * d + q * q);

  // === SECTION 3: PID Controllers ===
  float d_ref = 0;
  float d_pid_output = pid_controller_update(&foc_pid_d_controller, d_ref, d);
  float q_pid_output = pid_controller_update(&foc_pid_q_controller, q_ref_amperes, q);

  foc_set_phase_voltages_func(cos_theta_rad, sin_theta_rad, q_pid_output, d_pid_output);
}

/*===========================================================================*/
/* Motor Control PWM                                                         */
/*===========================================================================*/

#define PWM_CLOCK_FREQ 84000000
#define PWM_FREQ 21000

#define LINE_PWM_PHASE_AH PAL_LINE(GPIOA, GPIOA_ARD_D7)
#define LINE_PWM_PHASE_AL PAL_LINE(GPIOA, GPIOA_ARD_D11)
#define LINE_PWM_PHASE_BH PAL_LINE(GPIOA, GPIOA_ARD_D8)
#define LINE_PWM_PHASE_BL PAL_LINE(GPIOB, GPIOB_ARD_A3)
#define LINE_PWM_PHASE_CH PAL_LINE(GPIOA, GPIOA_ARD_D2)
#define LINE_PWM_PHASE_CL PAL_LINE(GPIOB, GPIOB_PIN1)

static void pwm_init(void) {
  palSetLineMode(LINE_PWM_PHASE_AH, PAL_MODE_ALTERNATE(1));
  palSetLineMode(LINE_PWM_PHASE_BH, PAL_MODE_ALTERNATE(1));
  palSetLineMode(LINE_PWM_PHASE_CH, PAL_MODE_ALTERNATE(1));

  palSetLineMode(LINE_PWM_PHASE_AL, PAL_MODE_ALTERNATE(1));
  palSetLineMode(LINE_PWM_PHASE_BL, PAL_MODE_ALTERNATE(1));
  palSetLineMode(LINE_PWM_PHASE_CL, PAL_MODE_ALTERNATE(1));


  palClearLine(LINE_PWM_PHASE_AH);
  palClearLine(LINE_PWM_PHASE_BH);
  palClearLine(LINE_PWM_PHASE_CH);

  palClearLine(LINE_PWM_PHASE_AL);
  palClearLine(LINE_PWM_PHASE_BL);
  palClearLine(LINE_PWM_PHASE_CL);

  // Enable TIM1 clock
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  // Set to center aligned mode
  TIM1->CR1 &= ~TIM_CR1_CMS_Msk;
  TIM1->CR1 |= TIM_CR1_CMS_1 | TIM_CR1_CMS_0;

  // Set the compare mode to PWM to channel 1
  TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
  TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;

  // Set the compare mode to PWM to channel 2
  TIM1->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
  TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;

  // Set the compare mode to PWM to channel 3
  TIM1->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;
  TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;

  // Set the compare mode to PWM to channel 4
  TIM1->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
  TIM1->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;

  // Set the prescaler to 0 for 84MHz clock
  TIM1->PSC = 0;

  // Set the period to 84MHz / (21 * 2)kHz = 4000 for center aligned mode
  TIM1->ARR = PWM_CLOCK_FREQ / (PWM_FREQ * 2);

  // Initialize the duty cycle to 0
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = TIM1->ARR - 1; 

  // Enable the Output
  TIM1->CCER |=  TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;

  // Enable the complementary output
  TIM1->CCER |= TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;

  // Set the polarity to active high
  TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P);

  // Set the complementary polarity to active high
  TIM1->CCER &= ~(TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP);

  // Configure the TIM1 as master
  TIM1->CR2 &= ~TIM_CR2_MMS_Msk;
  TIM1->CR2 |= TIM_CR2_MMS_2 | TIM_CR2_MMS_1 | TIM_CR2_MMS_0;

  // Enable the main output
  TIM1->BDTR |= TIM_BDTR_MOE;

  // Enable the TIM1 (master timer)
  TIM1->CR1 |= TIM_CR1_CEN;
}

// duty is 0-1
static void pwm_set_duty(float duty_a, float duty_b, float duty_c) {
  uint16_t arr = TIM1->ARR;

  motor_get_motor()->motor_state.phase_duty[0] = duty_a * arr;
  motor_get_motor()->motor_state.phase_duty[1] = duty_b * arr;
  motor_get_motor()->motor_state.phase_duty[2] = duty_c * arr;

  TIM1->CCR1 = duty_a * arr;
  TIM1->CCR2 = duty_b * arr;
  TIM1->CCR3 = duty_c * arr;
}

/*===========================================================================*/
/* ADC Driver                                                                */
/*===========================================================================*/

#define LINE_ADC_PHASE_A  PAL_LINE(GPIOA, GPIOA_ARD_A0)
#define LINE_ADC_PHASE_B  PAL_LINE(GPIOA, GPIOA_ARD_A1)
#define LINE_ADC_PHASE_C  PAL_LINE(GPIOA, GPIOA_ARD_A2)
#define LINE_ADC_POSITION PAL_LINE(GPIOC, GPIOC_ARD_A4)

#define ADC_NUM_CHANNELS 4

#define ADC_CHANNEL_PHASE_A 0
#define ADC_CHANNEL_PHASE_B 1
#define ADC_CHANNEL_PHASE_C 4
#define ADC_CHANNEL_POSITION 11

static void adc_init(void) {
  // Configure the GPIOs for ADC
  palSetLineMode(LINE_ADC_PHASE_A,  PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_ADC_PHASE_B,  PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_ADC_PHASE_C,  PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_ADC_POSITION, PAL_MODE_INPUT_ANALOG);

  // Enable ADC1 clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  // Set ADC prescaler to 2
  ADC->CCR &= ~ADC_CCR_ADCPRE_Msk;
  // ADC->CCR |= ADC_CCR_ADCPRE_;

  // Enable scan mode since we are using multiple channels
  ADC1->CR1 |= ADC_CR1_SCAN;

  
  // Enable ADC Injected Convesion Interrupts
  ADC1->CR1 |= ADC_CR1_JEOCIE;


  // Select TIM4 (PWM Slave) as trigger source on rising edge
  ADC1->CR2 &= ~(ADC_CR2_JEXTSEL_Msk | ADC_CR2_JEXTEN_Msk);
  ADC1->CR2 |= ADC_CR2_JEXTSEL_0;
  ADC1->CR2 |= ADC_CR2_JEXTEN_0;

  // Configure the sampling cycles
  // 100 -> 84 CYCLE

  ADC1->SMPR1 &= ~(ADC_SMPR1_SMP11_Msk);
  ADC1->SMPR1 |= ADC_SMPR1_SMP11_0;

  ADC1->SMPR2 &= ~(ADC_SMPR2_SMP0_Msk | ADC_SMPR2_SMP1_Msk | ADC_SMPR2_SMP4_Msk);
  ADC1->SMPR2 |= ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP1_0 | ADC_SMPR2_SMP4_0;

  // Configure the sequence length to 4 channels
  ADC1->JSQR &= ~ADC_JSQR_JL_Msk;
  ADC1->JSQR |= ADC_JSQR_JL_1 | ADC_JSQR_JL_0;

  // Configure the sequence of injected channels
  ADC1->JSQR &= ~(ADC_JSQR_JSQ1_Msk | ADC_JSQR_JSQ2_Msk | ADC_JSQR_JSQ3_Msk | ADC_JSQR_JSQ4_Msk);
  ADC1->JSQR |= (ADC_CHANNEL_PHASE_A << ADC_JSQR_JSQ1_Pos) |
                (ADC_CHANNEL_PHASE_B << ADC_JSQR_JSQ2_Pos) |
                (ADC_CHANNEL_PHASE_C << ADC_JSQR_JSQ3_Pos) |
                (ADC_CHANNEL_POSITION << ADC_JSQR_JSQ4_Pos);


  NVIC_EnableIRQ(ADC_IRQn);
  NVIC_SetPriority(ADC_IRQn, 2);

  // Enable ADC Conversion
  ADC1->CR2 |= ADC_CR2_ADON;

}

/*===========================================================================*/
/* ADS5600 Encoder Driver                                                    */
/*===========================================================================*/

#define ADS5600_COUNT_PER_REVOLUTION 4096
#define ADS5600_COUNT_TO_DEG(count) (count * 0.087890625f) // 360 / 4096
#define ADS5600_ENCODER_OFFSET_DEGREE 0.0f



/*===========================================================================*/
/* Position Tracker                                                          */
/*===========================================================================*/

// Position Tracker
typedef struct {
  float prev_position; 
  float now_angle; // accumulated angle (deg)
  float now_speed; // deg/s
} position_tracker_t;

static void position_tracker_init(position_tracker_t *position_tracker) {
  position_tracker->prev_position = 0;
  position_tracker->now_angle = 0;
  position_tracker->now_speed = 0;
}

static float position_tracker_update(position_tracker_t *position_tracker, float position_sensor_value, float dt) {
  float now_position = position_sensor_value;
  
  // Calculate raw delta position
  float delta_position = now_position - position_tracker->prev_position;
  
  // Handle wrap-around at 180°/-180° boundary
  if (delta_position > 180.0f) {
    // Crossed from negative to positive (e.g., -179° to +1°)
    delta_position -= 360.0f;
  } else if (delta_position < -180.0f) {
    // Crossed from positive to negative (e.g., +179° to -1°)
    delta_position += 360.0f;
  }
  
  // Accumulate the corrected delta
  position_tracker->now_angle += delta_position;
  position_tracker->prev_position = now_position;
  position_tracker->now_speed = delta_position / dt;
  return position_tracker->now_angle;
}

static float position_tracker_get_angle_deg(position_tracker_t *position_tracker) {
  return position_tracker->now_angle;
}

static float position_tracker_get_speed_deg_s(position_tracker_t *position_tracker) {
  return position_tracker->now_speed;
}

static void position_tracker_reset(position_tracker_t *position_tracker) {
  position_tracker->prev_position = 0;
  position_tracker->now_angle = 0;
  position_tracker->now_speed = 0;
}

/*===========================================================================*/
/* global variables                                                          */
/*===========================================================================*/

#define LINE_DRV_ENABLE             PAL_LINE(GPIOC, GPIOC_PIN8)
#define LINE_DRV_N_FAULT            PAL_LINE(GPIOC, GPIOC_PIN10) 
#define LINE_DRV_N_OCTW             PAL_LINE(GPIOC, GPIOC_PIN11)
#define LINE_DRV_M_OC               PAL_LINE(GPIOC, GPIOC_PIN12)
#define LINE_DRV_OC_ADJ             PAL_LINE(GPIOD, GPIOD_PIN2)
#define LINE_DRV_M_PWM              PAL_LINE(GPIOC, GPIOD_PIN9)

static position_tracker_t position_tracker;

current_sensing_t current_sensing;
// PID Controllers
static pid_controller_t position_controller;
static pid_controller_t speed_controller;
static float speed_controller_last_lpf_output;
static float speed_controller_low_pass_filter_alpha;

/*===========================================================================*/
/* SPEED Controller                                                          */
/*===========================================================================*/

static void speed_controller_init(float kp, float ki, float kd, float dt, float output_min, float output_max, float integral_max, float low_pass_filter_alpha) {
  pid_controller_init(&speed_controller, kp, ki, kd, dt, output_min, output_max, integral_max);
  speed_controller_last_lpf_output = 0;
  speed_controller_low_pass_filter_alpha = low_pass_filter_alpha;
}

static float speed_controller_update(float target, float input) {
  // Apply low pass filter to the input
  float lpf_output = LPF(speed_controller_last_lpf_output, input, speed_controller_low_pass_filter_alpha);
  speed_controller_last_lpf_output = lpf_output;
  float output = pid_controller_update(&speed_controller, target, lpf_output);
  return output;
}

static pid_controller_t* speed_controller_get_pid_controller(void) {
  return &speed_controller;
}

/*===========================================================================*/
/* POSITION Controller                                                       */
/*===========================================================================*/

static void position_controller_init(float kp, float ki, float kd, float dt, float output_min, float output_max, float integral_max) {
  pid_controller_init(&position_controller, kp, ki, kd, dt, output_min, output_max, integral_max);
}

static float position_controller_update(float target, float input) {
  return pid_controller_update(&position_controller, target, input);
}

static pid_controller_t* position_controller_get_pid_controller(void) {
  return &position_controller;
}

/*===========================================================================*/
/* ADC Interrupt Handler                                                     */
/*===========================================================================*/


CH_IRQ_HANDLER(STM32_ADC_HANDLER) {
  if (ADC1->SR & ADC_SR_JEOC) {
    ADC1->SR &= ~ADC_SR_JEOC; // Clear interrupt flag

    uint16_t phase_a_current = ADC1->JDR1 & ADC_JDR1_JDATA_Msk;
    uint16_t phase_b_current = ADC1->JDR2 & ADC_JDR2_JDATA_Msk;
    uint16_t phase_c_current = ADC1->JDR3 & ADC_JDR3_JDATA_Msk;
    uint16_t position = ADC1->JDR4 & ADC_JDR4_JDATA_Msk;

    motor_get_motor()->motor_state.phases_current_raw[0] = phase_a_current;
    motor_get_motor()->motor_state.phases_current_raw[1] = phase_b_current;
    motor_get_motor()->motor_state.phases_current_raw[2] = phase_c_current;

    float now_angle_deg = NORM_ANGLE_180(ADS5600_COUNT_TO_DEG(position));
    motor_get_motor()->motor_state.raw_angle_deg = now_angle_deg;

    // Update position tracker with PWM frequency timing
    position_tracker_update(&position_tracker, now_angle_deg, 1.0f / PWM_FREQ);
    current_sensing_update(&current_sensing, phase_a_current, phase_b_current, phase_c_current);

    float ia_current, ib_current, ic_current;
    current_sensing_get_current_phases(&current_sensing, &ia_current, &ib_current, &ic_current);

    motor_get_motor()->motor_state.phases_current[0] = ia_current;
    motor_get_motor()->motor_state.phases_current[1] = ib_current;
    motor_get_motor()->motor_state.phases_current[2] = ic_current;

    float now_speed_electrical_rpm = DEG_S_TO_RPM(position_tracker_get_speed_deg_s(&position_tracker)) * motor_get_motor()->motor_config.num_poles_pairs;
    float electrical_angle_deg = NORM_ANGLE_180((position_tracker_get_angle_deg(&position_tracker) + ADS5600_ENCODER_OFFSET_DEGREE) * motor_get_motor()->motor_config.num_poles_pairs);

    if(motor_get_motor()->motor_status == MOTOR_STATUS_RUNNING) {
      foc_update(electrical_angle_deg, now_speed_electrical_rpm, motor_get_motor()->motor_state.q_target_amperes, 
              ia_current, ib_current, ic_current);
    }
  }
}

/*===========================================================================*/
/* PID Update Thread                                                         */
/*===========================================================================*/

static THD_WORKING_AREA(pid_update_thread_wa, 256);
static THD_FUNCTION(pid_update_thread, arg) {
  (void)arg;
  chRegSetThreadName("PID Thread");
  // float dt = 1.0f / motor_get_motor()->motor_config.pid_frequency; // Convert to seconds
  
  while (true) {
    switch(motor_get_motor()->motor_control_mode) {
      case MOTOR_CONTROL_MODE_POS:
        float angle_pid_output = position_controller_update(motor_get_motor()->motor_state.target_position_deg, position_tracker_get_angle_deg(&position_tracker));
        float speed_pid_output = speed_controller_update(angle_pid_output, position_tracker_get_speed_deg_s(&position_tracker));
        motor_get_motor()->motor_state.q_target_amperes = speed_pid_output;
        break;
      case MOTOR_CONTROL_MODE_SPEED:
        float speed_pid_output2 = speed_controller_update(motor_get_motor()->motor_state.target_speed_deg_s, position_tracker_get_speed_deg_s(&position_tracker));
        motor_get_motor()->motor_state.q_target_amperes = speed_pid_output2;
        break;
      case MOTOR_CONTROL_MODE_CURRENT:
      case MOTOR_CONTROL_MODE_CURRENT_BRAKE:
      case MOTOR_CONTROL_MODE_HANDBRAKE:
      case MOTOR_CONTROL_MODE_OPENLOOP:
      case MOTOR_CONTROL_MODE_OPENLOOP_PHASE:
      case MOTOR_CONTROL_MODE_OPENLOOP_DUTY:
      case MOTOR_CONTROL_MODE_OPENLOOP_DUTY_PHASE:
      case MOTOR_CONTROL_MODE_NONE:
        pid_controller_reset(position_controller_get_pid_controller());
        pid_controller_reset(speed_controller_get_pid_controller());
        break;
      default:
        break;
    }
    chThdSleepMicroseconds(1000000 / motor_get_motor()->motor_config.pid_frequency);
  }
  
  // Thread function should not return a value
}

static void mc_set_position(float position, float speed, float current) {
  
  if(motor_get_motor()->motor_status != MOTOR_STATUS_RUNNING) {
    pid_controller_reset(position_controller_get_pid_controller());
    pid_controller_reset(speed_controller_get_pid_controller());
  }
  
  motor_get_motor()->motor_control_mode = MOTOR_CONTROL_MODE_POS;
  motor_get_motor()->motor_status = MOTOR_STATUS_RUNNING;
  motor_get_motor()->motor_state.target_position_deg = position;  // Position in degrees
  
  // Set speed limits for position controller output (deg/s)
  pid_controller_set_output_limits(position_controller_get_pid_controller(), -speed, speed);
  
  // Set current limits for speed controller output (amperes)
  pid_controller_set_output_limits(speed_controller_get_pid_controller(), -current, current);
  
  // Also set the d-axis current target to 0 for maximum efficiency
  motor_get_motor()->motor_state.d_target_amperes = 0.0f;
}

static void mc_set_speed(float speed, float current) {
  
  if(motor_get_motor()->motor_status != MOTOR_STATUS_RUNNING) {
    pid_controller_reset(speed_controller_get_pid_controller());
  }
  
  motor_get_motor()->motor_control_mode = MOTOR_CONTROL_MODE_SPEED;
  motor_get_motor()->motor_status = MOTOR_STATUS_RUNNING;
  motor.motor_state.target_speed_deg_s = speed;  // Speed in deg/s
  
  // Set current limits for speed controller output (amperes)
  pid_controller_set_output_limits(speed_controller_get_pid_controller(), -current, current);
  
  // Also set the d-axis current target to 0 for maximum efficiency
  motor_get_motor()->motor_state.d_target_amperes = 0.0f;
}

static void mc_set_current(float current) {
  if(motor_get_motor()->motor_status != MOTOR_STATUS_RUNNING) {
    pid_controller_reset(speed_controller_get_pid_controller());
  }
  
  motor_get_motor()->motor_control_mode = MOTOR_CONTROL_MODE_CURRENT;
  motor_get_motor()->motor_status = MOTOR_STATUS_RUNNING;
  motor_get_motor()->motor_state.q_target_amperes = current;  // Set q-axis current target in amperes
}

static void mc_set_release(void) {
  motor_get_motor()->motor_control_mode = MOTOR_CONTROL_MODE_NONE;
  motor_get_motor()->motor_status = MOTOR_STATUS_OFF;
  motor.motor_state.q_target_amperes = 0.0f;  // Set q-axis current target in amperes
  motor_get_motor()->motor_state.d_target_amperes = 0.0f;  // Set d-axis to 0 for efficiency
  pwm_set_duty(0, 0, 0);
}

/*===========================================================================*/
/* Shell Command Interface                                                   */
/*===========================================================================*/
#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)
char shell_history[SHELL_MAX_HIST_BUFF];
char *shell_completions[SHELL_MAX_COMPLETIONS];
const SerialConfig sd2_config = {
  .speed = 115200,
  .cr1 = 0,
  .cr2 = 0,
  .cr3 = 0,
};

static void cmd_set_position(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 3) {
    chprintf(chp, "Usage: set_position <position_deg> <speed_deg_s> <current_amps>\r\n");
    return;
  }
  float position = atof(argv[0]);
  float speed = atof(argv[1]);
  float current = atof(argv[2]);
  mc_set_position(position, speed, current);
}

static void cmd_set_speed(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 2) {
    chprintf(chp, "Usage: set_speed <speed_deg_s> <current_amps>\r\n");
    return;
  }
  float speed = atof(argv[0]);
  float current = atof(argv[1]);
  mc_set_speed(speed, current);
}

static void cmd_set_current(BaseSequentialStream *chp, int argc, char *argv[]) {
  if (argc != 1) {
    chprintf(chp, "Usage: set_current <current_amps>\r\n");
    return;
  }
  float current = atof(argv[0]);
  mc_set_current(current);
}

static void cmd_set_release(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;  // Unused parameter
  (void)argv;  // Unused parameter
  if (argc != 0) {
    chprintf(chp, "Usage: set_release\r\n");
    return;
  }
  mc_set_release();
}

static void cmd_motor_status(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  chprintf(chp, "Motor status: %d\r\n", motor_get_motor()->motor_status);
  chprintf(chp, "Motor control mode: %d\r\n", motor_get_motor()->motor_control_mode);
  chprintf(chp, "Motor target position: %.2f deg\r\n", motor_get_motor()->motor_state.target_position_deg);
  chprintf(chp, "Motor target speed: %.2f deg/s\r\n", motor_get_motor()->motor_state.target_speed_deg_s);
  chprintf(chp, "Motor target q current: %.2f A\r\n", motor_get_motor()->motor_state.q_target_amperes);
  chprintf(chp, "Motor target d current: %.2f A\r\n", motor_get_motor()->motor_state.d_target_amperes);
  chprintf(chp, "Motor target q voltage: %.2f V\r\n", motor_get_motor()->motor_state.q_voltage);
  chprintf(chp, "Motor target d voltage: %.2f V\r\n", motor_get_motor()->motor_state.d_voltage);
  chprintf(chp, "Motor voltage mag: %.2f V\r\n", motor_get_motor()->motor_state.voltage_mag);
  chprintf(chp, "Motor current mag: %.2f A\r\n", motor_get_motor()->motor_state.current_mag);
  chprintf(chp, "Motor duty: %.2f\r\n", motor_get_motor()->motor_state.duty);
  chprintf(chp, "Motor position: %.2f deg\r\n", position_tracker_get_angle_deg(&position_tracker));
  chprintf(chp, "Motor speed: %.2f deg/s\r\n", position_tracker_get_speed_deg_s(&position_tracker));
  chprintf(chp, "Motor phases current raw: %d, %d, %d\r\n", motor_get_motor()->motor_state.phases_current_raw[0], motor_get_motor()->motor_state.phases_current_raw[1], motor_get_motor()->motor_state.phases_current_raw[2]);
  chprintf(chp, "Motor phases current: %.2f A, %.2f A, %.2f A\r\n", motor_get_motor()->motor_state.phases_current[0], motor_get_motor()->motor_state.phases_current[1], motor_get_motor()->motor_state.phases_current[2]);
  chprintf(chp, "Motor phases duty: %d , %d, %d\r\n", motor_get_motor()->motor_state.phase_duty[0], motor_get_motor()->motor_state.phase_duty[1], motor_get_motor()->motor_state.phase_duty[2]);
}

static void cmd_serial_status(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  
  // Get SD1 (Modbus) buffer status
  size_t sd1_rx_bytes = iqGetFullI(&SD1.iqueue);
  size_t sd1_rx_capacity = qSizeX(&SD1.iqueue);
  size_t sd1_rx_free = iqGetEmptyI(&SD1.iqueue);
  bool sd1_rx_empty = iqIsEmptyI(&SD1.iqueue);
  bool sd1_rx_full = iqIsFullI(&SD1.iqueue);
  
  chprintf(chp, "SD1 (Modbus) RX Buffer:\r\n");
  chprintf(chp, "  Bytes in buffer: %d\r\n", sd1_rx_bytes);
  chprintf(chp, "  Buffer capacity: %d\r\n", sd1_rx_capacity);
  chprintf(chp, "  Free space: %d\r\n", sd1_rx_free);
  chprintf(chp, "  Status: %s%s\r\n", sd1_rx_empty ? "EMPTY " : "", sd1_rx_full ? "FULL" : "OK");
}

static const ShellCommand shell_commands[] = {
  {"set-position", cmd_set_position},
  {"set-speed", cmd_set_speed},
  {"set-current", cmd_set_current},
  {"set-release", cmd_set_release},
  {"motor-status", cmd_motor_status},
  {"serial-status", cmd_serial_status},
  {NULL, NULL},
};

static const ShellConfig shell_cfg = {
  (BaseSequentialStream *)&SD2,
  shell_commands,
  shell_history,
  sizeof(shell_history),
  shell_completions,
};

static THD_WORKING_AREA(shell_wa, SHELL_WA_SIZE);

/*===========================================================================*/
/* Communication Interface                                                   */
/*===========================================================================*/
#define MODBUS_SLAVE_ID 1
#define MODBUS_SERIAL &SD1
#define MODBUS_BAUD_RATE 115200

#define MODBUS_LINE_TX PAL_LINE(GPIOB, GPIOB_ARD_D10)
#define MODBUS_LINE_RX PAL_LINE(GPIOB, GPIOB_PIN7)

static THD_WORKING_AREA(mb_slave_app_thread_wa, 2048);

static modbus_t modbus_instances;
static mutex_t modbus_app_mutex;

const SerialConfig sd1_config = {
  .speed = MODBUS_BAUD_RATE,
  .cr1 = 0,
  .cr2 = 0,
  .cr3 = 0,
};

uint8_t modbus_serial_write_callback(uint8_t *buffer, uint16_t length) {
  uint32_t bytes_written = 0;
  chprintf((BaseSequentialStream *)&SD2, "Modbus serial write callback with length: %d\r\n", length);
  bytes_written = chnWrite(MODBUS_SERIAL, buffer, length);
  return bytes_written;
}

uint16_t modbus_holding_register_read_callback(uint16_t address) {
  return 0;
}

void modbus_holding_register_write_callback(uint16_t address, uint16_t value) {
  return;
}

uint16_t modbus_input_register_read_callback(uint16_t address) {
  return 0;
}

static THD_FUNCTION(mb_slave_app_thread, arg) {
  (void)arg;

  chRegSetThreadName("mb_slave_app_thread");

  event_listener_t el;
  chEvtRegisterMaskWithFlags(MODBUS_SERIAL.event, 
                          &el, EVENT_MASK(0), CHN_INPUT_AVAILABLE);
  
  for(;;) {
      chEvtWaitOne(EVENT_MASK(0));
      chThdSleepMicroseconds(1750);
      
      uint8_t buffer[MODBUS_BUFFER_SIZE];
      msg_t res = chnReadTimeout(MODBUS_SERIAL, buffer, MODBUS_BUFFER_SIZE, TIME_IMMEDIATE);
      if (res != MSG_TIMEOUT) {
          modbus_process_error_t ret = modbus_process_packet(&modbus_instances, buffer, res);
          chnReadTimeout(MODBUS_SERIAL, buffer, MODBUS_BUFFER_SIZE, TIME_IMMEDIATE);
          if (ret != MODBUS_PROCESS_SUCCESS) {
              chprintf((BaseSequentialStream *)&SD2, "Modbus process error: %d\r\n", ret);
          }
      }
  }
  chEvtUnregister(MODBUS_SERIAL.event, &el);
}

void mb_slave_app_init(uint8_t slave_id) {
  sdStart(MODBUS_SERIAL, &sd1_config);
  modbus_init(&modbus_instances, slave_id, 
                     modbus_serial_write_callback, 
                     modbus_holding_register_read_callback,  
                     modbus_holding_register_write_callback, 
                     modbus_input_register_read_callback);

  chThdCreateStatic(mb_slave_app_thread_wa, sizeof(mb_slave_app_thread_wa),
                       NORMALPRIO, mb_slave_app_thread, NULL);
}

int main(void)
{
  halInit();
  chSysInit();
  motor_init();

  pwm_init();
  adc_init();
  

  sdStart(&SD2, &sd2_config);
  shellInit();

  palSetLineMode(LINE_DRV_N_FAULT, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_DRV_N_OCTW, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_DRV_ENABLE, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_M_OC, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_OC_ADJ, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_M_PWM, PAL_MODE_OUTPUT_PUSHPULL);

  mb_slave_app_init(MODBUS_SLAVE_ID);

  position_tracker_init(&position_tracker);
  current_sensing_init(&current_sensing, 
    motor_get_motor()->current_sensing_config.gain, 
    motor_get_motor()->current_sensing_config.r_sense, 
    motor_get_motor()->current_sensing_config.voltage_raw_offset, 
    motor_get_motor()->current_sensing_config.current_sensing_mode
  );
  
  // Initialize controllers with PID thread timing (1000 Hz)
  float pid_dt = 1.0f / motor_get_motor()->motor_config.pid_frequency;
  position_controller_init(0.05f, 0.001f, 0.0f, pid_dt, -1000.0f, 1000.0f, 100.0f);
  speed_controller_init(0.05f, 0.001f, 0.0f, pid_dt, -1000.0f, 1000.0f, 100.0f, 0.1f);
  
  foc_init(PWM_FREQ, motor_get_motor()->motor_config.battery_voltage, motor_set_phase_voltages);

  chThdCreateStatic(pid_update_thread_wa, sizeof(pid_update_thread_wa), NORMALPRIO, pid_update_thread, NULL);

  while (true) {
    thread_t *shell_thd = chThdCreateFromHeap(NULL, SHELL_WA_SIZE, "shell", NORMALPRIO + 1, shellThread, (void *)&shell_cfg);
    chThdWait(shell_thd);
    chThdSleepMilliseconds(500);
  }
}