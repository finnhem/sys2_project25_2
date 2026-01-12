#include <Servo.h>
#include <Arduino.h>

#include <SignalSource.h>
#include <math.h>


// Global Variables
float MAX_POSITION = 1042.0;
float OFFSET = MAX_POSITION / 2;
static unsigned long last_time = 0;

const int16_t TORQUE_OFFSET = -6;

// Global constants for the PID controller
//const float Kp = 0.4;    // Proportional component
//const float Ki = 0.0;    // Integral component
//const float Kd = 0.0;   // Derivative component

// System parameters
const float T_krit = 0.4;  // Critical period duration (K_p=0.45 -> T=[0.23, 0.279]s)
const float K_R_krit = 1.58;  // Critical gain 0.07

// PID parameters according to table
const float Kp = 0.6 * K_R_krit;  // Proportional component
const float Ti = 0.5 * T_krit;    // Reset time
const float Td = 0.125 * T_krit;  // Derivative time

// Conversion for discrete implementation
const float Ki = Kp / Ti;         // Integral component
const float Kd = Kp * Td;        // Derivative component

// Sampling time
float dt = 0.005;

// Error deadband in encoder counts to prevent jitter around setpoint
static const int16_t ERROR_DEADBAND_COUNTS = 4;


// State variables for controller
static int16_t prev_error = 0;       // Previous error for derivative
static float integral_sum = 0.0;     // Accumulated integral component
static float prev_D = 0.0;           // Previous filtered D-term for LPF



//Resets the controller state when an experiment is started.
void reset(){
  prev_error = 0;
  integral_sum = 0.0;
  last_time = micros();
}

/**
 * Rescales the position value from the motor.
 *
 * @param position The original position value to be rescaled.
 * @return The rescaled position value, adjusted to remain within a range of 0 to 360.
 */
 float rescalepos(int16_t position) {
  if (MAX_POSITION == 0) return 0.0f;                // guard (minimal)
  const float scale = 360.0f / (float)MAX_POSITION;
  float scaled = (position - (float)OFFSET) * scale + 180.0f;
  scaled = fmodf(scaled, 360.0f);                   // reduce magnitude
  if (scaled < 0.0f) scaled += 360.0f;              // map to [0,360)
  return scaled;
}

/**
 * Calculates the error between the desired setpoint and the current position.
 *
 * @param setpoint The desired target value.
 * @param currentpos The current position value.
 * @return The error value, normalized to account for smallest angle to target value from current value.
 */
int16_t calculate_error(int16_t setpoint, int16_t currentpos){
  // Calculate dynamic dt
  unsigned long current_time = micros();
  if (last_time != 0) {
    float new_dt = (float)(current_time - last_time) / 1000000.0;
    // Sanity check to prevent division by zero or extreme spikes
    if (new_dt > 0.0001 && new_dt < 0.1) {
      dt = new_dt;
    }
  }
  last_time = current_time;

  // MAX_POSITION is the full rotation in encoder units
  const int16_t max_pos = (int16_t)MAX_POSITION;
  const int16_t half_pos = max_pos / 2;
  
  // Normalize both values to 0 to MAX_POSITION
  int16_t sp = setpoint % max_pos;
  if (sp < 0) sp += max_pos;
  
  int16_t cp = currentpos % max_pos;
  if (cp < 0) cp += max_pos;
  
  // Compute raw difference
  int16_t error = sp - cp;

  // Normalize to the smallest distance in [-half_pos, half_pos]
  if (error > half_pos) {
    error -= max_pos;
  } else if (error < -half_pos) {
    error += max_pos;
  }

  // Deadband: ignore tiny errors (encoder counts)
  if (abs(error) <= ERROR_DEADBAND_COUNTS) {
    error = 0;
  }

  return error;
}

/**
 * Calculates the derivative of the error for use in a PID controller.
 *
 * @param error The current error value.
 * @return The change in error (delta) since the last call.
 */
int16_t error_derivative(int16_t error){
  // Calculate change in error (delta)
  int16_t diff = error - prev_error;

  // Handle wrap-around
  int16_t max_pos = (int16_t)MAX_POSITION;
  int16_t half_pos = max_pos / 2;
  if (diff > half_pos) {
    diff -= max_pos;
  } else if (diff < -half_pos) {
    diff += max_pos;
  }
  
  // Save current error for next call
  prev_error = error;
  //return 0;
  return (int16_t)((float)diff / dt);
}

/**
 * Calculates the integral of the error for use in a PID controller.
 *
 * @param error The current error value.
 * @return The accumulated error over time (integral).
 */
int16_t error_integral(int16_t error){
  // Accumulate error (integration with dt)
  integral_sum += (float)error * dt;
  
  // Anti-Windup: Limiting the integral component
  // Based on torque limits (-20 to 20)
  if (Ki > 0.0001) {  // Only limit if Ki is active
    const float integral_limit = 20.0 / Ki;  // Saturation at max torque
    if (integral_sum > integral_limit) {
      integral_sum = integral_limit;
    } else if (integral_sum < -integral_limit) {
      integral_sum = -integral_limit;
    }
  }

  return (int16_t)integral_sum;
}


/**
 * Implements a basic controller using proportional, integral, and derivative (PID) control.
 *
 * @param error The current error value.
 * @param error_i The integral of the error.
 * @param error_d The derivative of the error.
 * @param measured_disturbance The measured disturbance value.
 * @return The calculated control torque.
 */
int16_t controller(int16_t error, int16_t error_i, int16_t error_d, int16_t measured_disturbance){
  // PID control law (discrete-time):
  // u(k) = Kp * e(k) + Ki * integral_sum + Kd * (de/dt)

  const float Kp = 0.095;
  const float Ki = 0.26;
  const float Kd = 0.06;

  // P-component
  float P = Kp * (float)error;
  //P = 0.7f * P;
  
  // I-component (error_i already contains the accumulated integration)
  float I = Ki * (float)error_i;
  //I = 1.10f * I;
  //I = 0.0f;
  //I = 0.0*I;


  // D-component (error_d is the derivative)
  float D = Kd * (float)error_d;
  // Low-pass filter on D-term to reduce noise
  const float alpha = 0.15f; // Smoothing factor (0 < alpha < 1)
  D = alpha * D + (1.0f - alpha) * prev_D;

  // Deadband for D-term to prevent jitter
  if (abs(error) < ERROR_DEADBAND_COUNTS)
    D = 0;

  prev_D = D;
  //D = 0.01f * D;
  //D = 0.0f;
  
  // Total control variable with offset (tau_offset from Task 2)
  float torque = P + I + D;
  // Disturbance compensation (Task 4.1 - TYPE 1):
  // Feedforward compensation: Disturbance is subtracted directly from the control signal
  // Note: Dead time in communication (Arduino <-> Measuring device) can lead to
  // the measured disturbance arriving with a delay. For fast
  // disturbances, the compensation becomes inaccurate or unstable.
  torque -= (float)measured_disturbance;
  
  // Deadzone Compensation
  // Negative torque needs -6 start, Positive needs +1 start
  if (torque<-1.0) {
    torque -= 5.0; 
  }

  // Limiting to actuator limits (-20 to 20)
  if (torque > 20.0) {
    torque = 20.0;
  } else if (torque < -20.0) {
    torque = -20.0;
  }

  return (int16_t)torque;
}

/**
 * Placeholder for the main program loop. Do not change this.
 */
void loop(){
  start_loop();
}

/**
 * Sets up the controller by registering student-defined functions and initializing signals.
 * Do not change this.
 */
void setup(){
  register_student_fcns((studentFcns){
    .calculate_position=rescalepos,
    .calculate_error=calculate_error,
    .calculate_error_integral=error_integral,
    .calculate_error_derivative=error_derivative,
    .calculate_control=controller,
    .reset=reset});
  setup_signal();
}
