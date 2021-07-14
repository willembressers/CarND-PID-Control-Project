#include <cmath>
#include <iostream>
#include <fstream>
#include "Twiddle.h"

Twiddle::Twiddle() {}

Twiddle::~Twiddle() {}

using namespace std;

void Twiddle::Init(double Kp, double Ki, double Kd, bool status) {
  p = {Kp, Ki, Kd};
  dp = {1, 1, 1};
  n = 0;
  state = INIT;
  i = 0;
  total_err = 0.0;
  best_err = 1.0;
  sum = 0.0;
  it = 0;
  running = status;
}

void Twiddle::incrementCount(double cte) {
  // increment the total count
  total_err += fabs(cte);
  n++;
}

bool Twiddle::getStatus() {
  return running;
}

std::vector<double> Twiddle::updateParams(double tol) {
  double err = total_err / n;
  n = 0;
  total_err = 0;
  sum = dp[0] + dp[1] + dp[2];

  bool next_p = false;
  
  if (sum > tol) {
    std::cout << "Iteration " << ++it << ", best error = " << best_err << ", p = [" << p[0] << "," << p[1] << "," << p[2] << "]" << std::endl;

    switch (state) {

      case INIT:
        p[i] += dp[i];
        state = INCREMENT;
        break;

      case INCREMENT:
        if (err < best_err) {
          best_err = err;
          dp[i] *= 1.1;
          next_p = true;

        } else {
          p[i] -= 2 * dp[i];
          state = DECREMENT;
        }
        break;

      case DECREMENT:
        if (err < best_err) {
          best_err = err;
          dp[i] *= 1.1;
        } else {      
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
        
        state = INCREMENT;
        next_p = true;
        break;

    }

    if (next_p && (++i == 3)) {
      i = 0;
    }
  }

  return p;
}

void Twiddle::log() {
  ofstream foutput; 
  ifstream finput;
  finput.open ("twiddle.log");
  foutput.open ("twiddle.log",ios::app); 

  if(finput.is_open()) {
    foutput << "Kp:" << p[0] << ", Ki:" << p[1] << ", Kd:" << p[2] << ", sum:" << sum << std::endl;
  }

  finput.close();
  foutput.close(); 
}