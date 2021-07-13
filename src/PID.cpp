#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  // Initialize PID coefficients
  Kp = Kp_;
  Ki = Ki_;
  // Kd = Kd_;

  // Initialize coeffiecent errors
  // d_error = 0;
  i_error = 0;
  p_error = 0;
}

void PID::UpdateError(double cte) {
  // Update PID errors based on cte.

  // set the most recent cross track error
  p_error = cte;

  // update the sum of all cross track errors so far
  i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your total error calc here!
}

double PID::GetSteeringValue() {
  
  // calculate the proportional value
  double P = -Kp * p_error;

  // calculate the intergral value
  double I = -Ki * i_error;

  return P + I;
}