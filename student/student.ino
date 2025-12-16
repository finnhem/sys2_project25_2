#include <SignalSource.h>
#include <math.h>


// Global Variables
//TODO

//Resets the controller state when an experiment is started.
void reset(){
  //TODO
}

/**
 * Rescales the position value from the motor.
 *
 * @param position The original position value to be rescaled.
 * @return The rescaled position value, adjusted to remain within a range of 0 to 360.
 */
float rescalepos(int16_t position){  //rescaling function
  //TODO
  return 0;
}

/**
 * Calculates the error between the desired setpoint and the current position.
 *
 * @param setpoint The desired target value.
 * @param currentpos The current position value.
 * @return The error value, normalized to account for smallest angle to target value from current value.
 */
int16_t calculate_error(int16_t setpoint, int16_t currentpos){
  //TODO
  return 0;
}

/**
 * Calculates the derivative of the error for use in a PID controller.
 *
 * @param error The current error value.
 * @return The change in error (delta) since the last call.
 */
int16_t error_derivative(int16_t error){
  //TODO
  return 0;
}

/**
 * Calculates the integral of the error for use in a PID controller.
 *
 * @param error The current error value.
 * @return The accumulated error over time (integral).
 */
int16_t error_integral(int16_t error){
  //TODO
  return 0;
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
  //TODO
  return 0;
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
