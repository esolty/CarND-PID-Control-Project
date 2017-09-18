#include "PID.h"
#include "math.h"
#include <fstream>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // coefficients
  PID::Kp = Kp; 
  PID::Ki = Ki;
  PID::Kd = Kd;

  // errors
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  // twiddle counter
  count = 0;
  P_twiddle = true;
  I_twiddle = false;
  D_twiddle = false;

  //current_time = 0;
  //previous_time = 0;
  num_iterations = 0;
  all_iterations = 0;
  stored_error = 0;
  min_iterations = 10;
  current_iter_num_of_restarts = 0;

  // twiddle values
  startloop = true;
  dKp = 0.2 * Kp; 
  dKi = 0;//0.1 * Ki; 
  dKd = 0.2 * Kd;
  Kp_best = 0;
  Ki_best = 0;
  Kd_best = 0;
  controller_term = Kp;
  delta = dKp;
  sum_delta = 0;
  best_err = 1000000;
  best_err_num_of_restarts = 1000000;
  TwiddleEnable = true;
  last_speed_change_time = 5000;
}

void PID::UpdateError(double cte, double dt) {
  // updating error using time or not 
  //d_error = (cte - p_error)/ dt; 
  d_error = cte - p_error;
  p_error = cte;
  //i_error += cte * dt;
  i_error = i_error + cte;
}

double PID::TotalError() {

  double steer = -Kp * p_error -Kd * d_error - Ki * i_error;

  // limit steering value to 1 and -1
  if(steer > 1) {
    steer = 1;
  }
  if(steer < -1) {
    steer = -1;
  }
  return steer;
}

void PID::Total_CTE_per_Twiddle_Iteration(double cte)
{
  if(num_iterations >= min_iterations)
    stored_error += fabs(cte);
  else {
    stored_error = 0.0;
  }
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws)
{
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

double PID::speed_control(double speed, double maxspeed) {
  double throttle_value = 0.3;
/*
  // accerlate quickly
  if (maxspeed > 15) {
    if (speed/maxspeed < 0.75){
      throttle_value = 1.0;
    }
  }
*/

  // maintain speed
  if(speed >= maxspeed+0.5) {
    throttle_value = 0;
  }
  else if(speed >= maxspeed-0.5 && speed < maxspeed) {
    throttle_value = 0.095;
  }
  else if(speed >= maxspeed-1.25 && speed < maxspeed-0.5) {
    throttle_value = 0.18;
  }
  else if(speed <= maxspeed+0.5 && speed > maxspeed) {
    throttle_value = 0.085;
  }
  return throttle_value;
}

void PID::Twiddle() {

  all_iterations += num_iterations;
  double tolerance = 0.001;
  double err = stored_error / (max_iterations - min_iterations);
  sum_delta = dKp+dKi+dKd;

  // increase speed and restart when tolerance reached, Ki not used currently
  if (P_twiddle == true && sum_delta < tolerance) { count = 10; }
  else if (I_twiddle == true && sum_delta < tolerance) { count = 10; }
  else if (D_twiddle == true && sum_delta < tolerance) { count = 10; }

  // if adding positive delta is better make delta larger by 1.1 
  if (count == 1) {
    if (current_iter_num_of_restarts < best_err_num_of_restarts) {
      count = 0;
      best_err = err;
      best_err_num_of_restarts = current_iter_num_of_restarts;
      delta *= 1.1;
      startloop = false;
      if (P_twiddle == true) { Kp_best = controller_term; }
      if (I_twiddle == true) { Ki_best = controller_term; }
      if (D_twiddle == true) { Kd_best = controller_term; }
    }
    if (err < best_err && current_iter_num_of_restarts == best_err_num_of_restarts) {
      count = 0;
      best_err = err;
      delta *= 1.1;
      startloop = false;
      if (P_twiddle == true) { Kp_best = controller_term; }
      if (I_twiddle == true) { Ki_best = controller_term; }
      if (D_twiddle == true) { Kd_best = controller_term; }
    }
  } 

  // if adding negative delta is better make delta larger by 1.1
  if (count == 2) {
    if (current_iter_num_of_restarts < best_err_num_of_restarts) {
      count = 0;
      best_err = err;
      best_err_num_of_restarts = current_iter_num_of_restarts;
      delta *= 1.1;
      startloop = false;
      if (P_twiddle == true) { Kp_best = controller_term; }
      if (I_twiddle == true) { Ki_best = controller_term; }
      if (D_twiddle == true) { Kd_best = controller_term; }
    }
    else if (err < best_err && current_iter_num_of_restarts == best_err_num_of_restarts) {
      count = 0;
      best_err = err;
      delta *= 1.1;
      startloop = false; 
      if (P_twiddle == true) { Kp_best = controller_term; }
      if (I_twiddle == true) { Ki_best = controller_term; }
      if (D_twiddle == true) { Kd_best = controller_term; }
    }
  }

  // if adding negative not better make dKd smaller by 0.9
  if (count == 2) {
    if (current_iter_num_of_restarts > best_err_num_of_restarts) {
      delta *= 0.9;
      count = 0;
      startloop = false;
      if (P_twiddle == true) { controller_term = Kp_best; }
      if (I_twiddle == true) { controller_term = Ki_best; }
      if (D_twiddle == true) { controller_term = Kd_best; }
      }    
    else if (err > best_err && current_iter_num_of_restarts == best_err_num_of_restarts) {
      delta *= 0.9;
      count = 0;
      startloop = false;
      if (P_twiddle == true) { controller_term = Kp_best; }
      if (I_twiddle == true) { controller_term = Ki_best; }
      if (D_twiddle == true) { controller_term = Kd_best; }
      }
  }
 
  // if adding positive dKd not better make dKd negative and subtract
  if (count == 1) {
    if (current_iter_num_of_restarts > best_err_num_of_restarts) {
      controller_term -= 2* delta;
      count = 2;
    }
    else if (err > best_err && current_iter_num_of_restarts == best_err_num_of_restarts) {
      controller_term -= 2* delta;
      count = 2;
    }
  }

  if (startloop == false) {
    startloop = true;
    if (P_twiddle == true) { P_twiddle = false; D_twiddle = true; Kp = Kp_best; dKp = delta; controller_term = Kd_best; delta = dKd; }
    else if (I_twiddle == true) { I_twiddle = false; D_twiddle = true; Ki = Ki_best; dKi = delta; controller_term = Kd_best; delta = dKd; }
    else if (D_twiddle == true) { D_twiddle = false; P_twiddle = true; Kd = Kd_best; dKd = delta; controller_term = Kp_best; delta = dKp; }
  }

  // add dKd
  if (startloop == true) {
    if (count == 0) {
      count = 1;
      controller_term += delta;
    }
  }

  if (P_twiddle == true){Kp = controller_term; dKp = delta;}
  if (I_twiddle == true){Ki = controller_term; dKi = delta;}
  if (D_twiddle == true){Kd = controller_term; dKd = delta;}
}
 
void PID::RecordCycle(double maxspeed) {
  // headers mph; Tot_iters; CE; CE_Rests; BE; BE_rests; Kp_best; Ki_best; Kd_best; Kp; Ki; Kd; dKp; dKi; dKd; p_error; i_error; d_error;
  std::ofstream myfile;
  myfile.open("storage.txt", std::ios_base::app);
  myfile << maxspeed << "; ";
  myfile << all_iterations << "; ";
  myfile << stored_error/fabs(num_iterations-(min_iterations-2)) << "; ";
  myfile << current_iter_num_of_restarts << "; ";
  myfile << best_err << "; ";
  myfile << best_err_num_of_restarts << "; ";
  myfile << Kp_best << "; ";
  myfile << Ki_best << "; ";
  myfile << Kd_best << "; ";
  myfile << Kp << "; ";
  myfile << Ki << "; ";
  myfile << Kd << "; ";
  myfile << dKp << "; ";
  myfile << dKi << "; ";
  myfile << dKd << "; ";
  myfile << p_error << "; ";
  myfile << i_error << "; ";
  myfile << d_error << "\n";
  myfile.close();
}
