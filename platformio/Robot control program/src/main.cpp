#include <Arduino.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <Wire.h>
#include <MBED_RP2040_PWM.h>
#include <Servo.h>
#include <MBED_RPi_Pico_ISR_Timer.h>
#include <MBED_RPi_Pico_TimerInterrupt.h>
#include <SimpleKalmanFilter.h>
#include <math.h>
#include <function_declarations.h>

#define PWM_PIN 20
#define DIV_PIN 26
#define MEASUREMENT_INTERVAL_MS 40
#define TRIG_HIGH_TIME_US 10
#define TRIG_PIN 14
#define ECHO_PIN 15
#define BACK_SERVO_PIN 9
#define SIDE_SERVO_PIN 8
#define CLOSED 120
#define OPEN 0

#define USING_MBED_RPI_PICO_TIMER_INTERRUPT true
#define _TIMERINTERRUPT_LOGLEVEL_ 4

Servo back_servo;
Servo side_servo;
LIS3MDL mag;
LSM6 imu;
mbed::PwmOut* fan_pwm = NULL;
MBED_RPI_PICO_Timer timer_take_measurement(1);
MBED_RPI_PICO_Timer timer_trig_low(2);
SimpleKalmanFilter ultrasonic_kalman_filter(1,1,0.1);
SimpleKalmanFilter acc_kalman_filter(1000,1000,0.4);
SimpleKalmanFilter gyro_kalman_filter(1000,1000,0.2);

enum bot_state {
  IDLE_HOVER,
  FORWARD,
  TURBO_BOOST,
  ROTATE,
  GRACEFULLY_STOP,
  EMERGENCY_STOP,
  SLIDING
};

bot_state curr_state = IDLE_HOVER;

const float duty_cycle = 10.0f;
const float freq = 25000.0f;
const float hover_duty_cycle = 80.0f;
const float rotate_duty_cycle = 95.0f;
const float emergency_stop_duty_cycle = 40.0f;
const float gracefully_stop_duty_cycle = 50.0f;
const uint16_t rotate_time_ms = 2000;
const uint16_t turbo_boost_time_ms = 2000;
const uint16_t emergency_stop_timer_ms = 2000;
const uint16_t idle_hover_timer_ms = 2000;
const uint16_t graceful_stop_timer_ms = 2000;
const uint8_t safe_distance = 30;
const uint8_t safe_enough_distance = 35;
const uint8_t acc_dt = 50;
const float acc_dt_s = 0.05;
const int initial_cal_time = 10000;
const float decay = 1.5;
const int imu_timer = 100;

float turbo_boost_duty_cycle = 100.0f;
float forward_duty_cycle = 85.0f;
long back_servo_val = 120;
long side_servo_val = 120;
float dist;
float v_estimate;
float imu_acc_read;
float last_imu_acc_est;
float imu_acc_est;
float filtered_dist = 0;
bool flag_rotate = false;
unsigned long rotate_timer;
unsigned long turbo_boost_timer;
unsigned long emergency_stop_timer;
unsigned long idle_hover_timer;
unsigned long idle_hover_calibration_timer;
unsigned long graceful_stop_timer;
unsigned long acc_dt_timer;
unsigned long cal_timer;
float inaccuracy = 0;

volatile uint16_t ultrasonic_read;
volatile long time_echo;
volatile long time_start;
volatile bool get_dist = false;

// if using default imu calibration values
LIS3MDL::vector<int16_t> m_min = {-32767, -32767, -32767};
LIS3MDL::vector<int16_t> m_max = {+32767, +32767, +32767};
// if calibrating
//LIS3MDL::vector<int16_t> m_min = {0, 0, 0};
//LIS3MDL::vector<int16_t> m_max = {0, 0, 0};


unsigned long imu_poll = 0;
float imu_read;
float v_avg = 0;

void setup() {
  setPWM(fan_pwm, PWM_PIN, freq, duty_cycle);
  back_servo.attach(BACK_SERVO_PIN);
  side_servo.attach(SIDE_SERVO_PIN);
  Serial.begin(9600);
  Wire.begin();
  delay(1000);
  if (!mag.init()) {
    Serial.println("Failed to initialize magnetometer");
    while(1);
  }
  if (!imu.init()) {
    Serial.println("Failed to initialize gyro and accelerometer");
    while(1);
  }
  Serial.println("hello");
  delay(1000);
  mag.enableDefault();
  imu.enableDefault();

  if (timer_take_measurement.attachInterruptInterval(MEASUREMENT_INTERVAL_MS*1000, timer_start_m_handler)) {
    //worked
  }
  if (timer_trig_low.attachInterruptInterval(TRIG_HIGH_TIME_US, timer_trig_low_handler)) {
    timer_trig_low.disableTimer();
  }
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echo_handler, FALLING);
  digitalWrite(TRIG_PIN, LOW);
  curr_state = IDLE_HOVER;
  delay(5000);
  cal_timer = millis()+initial_cal_time;
  initial_calibration();
} 

void loop() {
  if (get_dist) {
    dist = 0.6*dist + 0.4*calc_dist(time_echo);
    filtered_dist = ultrasonic_kalman_filter.updateEstimate(dist);
    get_dist = false;
  }

  switch (curr_state) {
    case IDLE_HOVER:
      hover_nicely();
      if (should_calibrate()) {
        accel_calibrate();
      }
      if (!hover_cooldown_over()) {
      } else if (should_rotate()) {
        engage_rotation_mode();
      } else if (should_turbo_boost()) {
        engage_turbo_drive();
      }
    break;

    case FORWARD: 
      creep_forward();
      if (millis() > acc_dt_timer) {
        update_v_est();
      }
      if (should_emergency_stop()) {
        engage_emergency_stop();
      } else if (should_gracefully_stop()) {
        engage_graceful_stop();
      } else if (too_fast()) {
        engage_sliding_mode();
      } else if (too_slow()) {

      }
    break;

    case TURBO_BOOST:
      boost_like_no_tomorrow();
      if (millis() > acc_dt_timer) {
        update_v_est();
      }
      if (should_emergency_stop()) {
        engage_emergency_stop();
      } else if (enough_turbo()) {
        curr_state = FORWARD;
      } else if (too_fast()) {
        engage_sliding_mode();
      }
    break;

    case ROTATE:
      rotate();
      if (finished_rotate()) {
        engage_graceful_stop();
      }
    break;

    case GRACEFULLY_STOP:
      come_to_a_graceful_stop();
      if (should_emergency_stop()) {
        engage_emergency_stop();
      } else if (should_hover()) {
        engage_hover_mode();
      } else if (stuck()) {
        engage_emergency_stop();
      }
    break;

    case EMERGENCY_STOP:
      stop_fast();
      if (should_hover_post_emergency()) {
        engage_hover_mode();
      }
    break;

    case SLIDING:
      slide();
      if (millis() > acc_dt_timer) {
        update_v_est();
      }
      if (should_emergency_stop()) {
        engage_emergency_stop();
      } else if (should_gracefully_stop()) {
        engage_graceful_stop();
      } else if (too_slow()) {
        curr_state = FORWARD;
      }
    break;
  }

  imu.read();
  imu_read = 0.8*imu_read + 0.2*(imu.g.z+470);
  float rot_est = gyro_kalman_filter.updateEstimate(imu_read);
  imu_acc_read = (0.9*imu_acc_read + 0.1*-(imu.a.y));
  imu_acc_est = acc_kalman_filter.updateEstimate(imu_acc_read);
  imu_acc_est -= inaccuracy;

  Serial.println(filtered_dist);
  delay(10);
}

bool stuck() {
  if (filtered_dist <= safe_enough_distance) {
    return true;
  }
  return false;
}

void initial_calibration() {
  while (millis() < cal_timer) {
    if (millis() > acc_dt_timer) { 
      imu.read();
      imu_acc_read = (0.8*imu_acc_read + 0.2*-(imu.a.y));
      imu_acc_est = acc_kalman_filter.updateEstimate(imu_acc_read);
      acc_dt_timer = millis() + acc_dt;
      //Serial.println(imu_acc_est);
    }
    if (abs(imu_acc_est - last_imu_acc_est) < 200) {
      inaccuracy = imu_acc_est;
    }
    last_imu_acc_est = imu_acc_est;
  }
}

void accel_calibrate() {
  if (millis() > acc_dt_timer) { 
    imu.read();
    imu_acc_read = (0.8*imu_acc_read + 0.2*-(imu.a.y));
    imu_acc_est = acc_kalman_filter.updateEstimate(imu_acc_read);
    acc_dt_timer = millis() + acc_dt;
    if (abs(imu_acc_est - last_imu_acc_est) < 200) {
      inaccuracy = imu_acc_est;
    }
    last_imu_acc_est = imu_acc_est;
  }
}

bool too_slow() {
  if (v_estimate < 15) {
    if (forward_duty_cycle < 85) forward_duty_cycle++;
    if (turbo_boost_duty_cycle < 100) turbo_boost_duty_cycle++;
    return true;
  }
  return false;
}

bool should_calibrate() {
  if (millis() > idle_hover_calibration_timer) {
    return true;
  }
  return false;
}

bool too_fast() {
  if (v_estimate >=15) {
    if (forward_duty_cycle >= 82) forward_duty_cycle--;
    if (turbo_boost_duty_cycle > 90) turbo_boost_duty_cycle--;
    return true;
  }
  return false;
}

// v_estimate is more like a movement average, not actual velocity, but should be used to prevent the robot from accelerating way too fast in the case when friction happens to be exceptionally low.
// which happens sometimes.. like when the battery is fully charged.
void update_v_est() {
  v_estimate = v_estimate + abs(imu_acc_est-last_imu_acc_est)*(acc_dt_s)/10;
  if (v_estimate > 0) {
    v_estimate -= decay;
  } else {
    v_estimate += decay;
  }
  last_imu_acc_est = imu_acc_est;
  acc_dt_timer = millis() + acc_dt;
  Serial.println(v_estimate);
}

// scrapped as magnetometer seems broken
void get_mag_data() {
  mag.read();
  int32_t temp[2] = {mag.m.x, mag.m.y};
  float heading = atan2(temp[1], temp[0]) * 180 / PI;
  float mag_dec = 9.93;
  heading += mag_dec;
  if (heading < 0) heading += 360;
  //Serial.println(heading);
}

void slide() {
  setPWM(fan_pwm, PWM_PIN, freq, hover_duty_cycle);
  back_servo.write(OPEN);
  side_servo.write(CLOSED);
}

void stop_fast() {
  setPWM(fan_pwm, PWM_PIN, freq, emergency_stop_duty_cycle);
  back_servo.write(CLOSED);
  side_servo.write(CLOSED);
}

void come_to_a_graceful_stop() {
  setPWM(fan_pwm, PWM_PIN, freq, gracefully_stop_duty_cycle);
  back_servo.write(CLOSED);
  side_servo.write(CLOSED);
}

void rotate() {
  setPWM(fan_pwm, PWM_PIN, freq, rotate_duty_cycle);
  back_servo.write(CLOSED);
  side_servo.write(OPEN);
}

void boost_like_no_tomorrow() {
  setPWM(fan_pwm, PWM_PIN, freq, turbo_boost_duty_cycle);
  back_servo.write(OPEN);
  side_servo.write(CLOSED);
}

void creep_forward() {
  setPWM(fan_pwm, PWM_PIN, freq, forward_duty_cycle);
  back_servo.write(OPEN);
  side_servo.write(CLOSED);
}

void engage_hover_mode() {
  idle_hover_timer = millis() + idle_hover_timer_ms;
  idle_hover_calibration_timer = millis() + idle_hover_timer_ms/4;
  curr_state = IDLE_HOVER;
}

void hover_nicely() {
  setPWM(fan_pwm, PWM_PIN, freq, hover_duty_cycle);
  back_servo.write(CLOSED);
  side_servo.write(CLOSED);
}

void engage_sliding_mode() {
  curr_state = SLIDING;
  last_imu_acc_est = 0;
}

void engage_graceful_stop() {
  graceful_stop_timer = millis() + graceful_stop_timer_ms;
  curr_state = GRACEFULLY_STOP;
}

void engage_emergency_stop() {
  emergency_stop_timer = millis() + emergency_stop_timer_ms;
  curr_state = EMERGENCY_STOP;
}

void engage_rotation_mode() {
  rotate_timer = millis() + rotate_time_ms;
  curr_state = ROTATE;
}

void engage_turbo_drive() {
  turbo_boost_timer = millis() + rotate_time_ms;
  v_estimate = 0;
  curr_state = TURBO_BOOST;
}

bool should_turbo_boost() {
  if ((filtered_dist >= safe_enough_distance)) {
    return true;
  }
  return false;
}

bool should_hover() {
  if (millis() > graceful_stop_timer) {
    return true;
  }
  return false;
}

bool hover_cooldown_over() {
  if (millis() > idle_hover_timer) {
    return true;
  }
  return false;
}

bool should_hover_post_emergency() {
  if (millis() > emergency_stop_timer) {
    return true;
  }
  return false;
}

bool enough_turbo() {
  if (millis() > turbo_boost_timer) {
    return true;
  }
  return false;
}

bool should_gracefully_stop() {
  if (filtered_dist < safe_enough_distance) {
    return true;
  }
  return false;
}

bool should_emergency_stop() {
  if (dist < safe_distance) {
    return true;
  }
  return false;
}

bool finished_rotate() {
  if (millis() > rotate_timer && dist > safe_distance) {
    return true;
  } 
  return false;
}

bool should_rotate() {
  if (dist < safe_distance) return true;
  return false;
}

void echo_handler() {
  time_echo = micros() - time_start;
  get_dist = true;
}

void timer_trig_low_handler(uint alarm_num) {
  TIMER_ISR_START(alarm_num);
  digitalWrite(TRIG_PIN, LOW);
  timer_trig_low.disableTimer();
  time_start = micros()+3000;
  TIMER_ISR_END(alarm_num);
}

void timer_start_m_handler(uint alarm_num) {
  TIMER_ISR_START(alarm_num);
  digitalWrite(TRIG_PIN, HIGH);
  timer_trig_low.enableTimer();
  TIMER_ISR_END(alarm_num);
}

void calibrate_mag() {
  unsigned long cal_time = millis() + 10000;
  while (millis() < cal_time) {
    mag.read();
    m_min.x = min(m_min.x, mag.m.x);
    m_min.y = min(m_min.y, mag.m.y);
    m_min.z = min(m_min.z, mag.m.z);
    m_max.x = max(m_max.x, mag.m.x);
    m_max.y = max(m_max.y, mag.m.y);
    m_max.z = max(m_max.z, mag.m.z);
    Serial.println(millis());
    Serial.println(cal_time);
    delay(50);
  }
}

long calc_dist(long fly_time) {
  return (long)((fly_time/2)/29.1);
}