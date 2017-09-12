#ifndef PID_H
#define PID_H
#include <uWS/uWS.h>
#include <iostream>

class PID {
public:

  double Kp;
  double Ki;
  double Kd;

  double p_error;
  double i_error;
  double d_error;

  double speed;
  double maxspeed;


  /*
  * Twiddle values
  */
  bool startloop;
  int num_iterations;
  int all_iterations;
  double stored_error;
  int min_iterations;
  int max_iterations;
  bool P_twiddle;
  bool I_twiddle;
  bool D_twiddle;
  int count;
  double delta;
  double controller_term;
  double dKp;
  double dKi;
  double dKd;
  double best_err;
  double Kp_best;
  double Ki_best;
  double Kd_best;
  int best_err_num_of_restarts;
  int current_iter_num_of_restarts;
  bool TwiddleEnable;
  double current_time;
  double previous_time;
  double sum_delta;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double dt);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Twiddle algorithm
  */
  void Twiddle();

  /*
  * Speed control
  */
 
  double speed_control(double speed, double maxspeed);

  /*
  * Stores the cumulative error
  */
  void Total_CTE_per_Twiddle_Iteration(double cte) ;

  /*
  * Restarts the simulator
  */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);

};

#endif /* PID_H */
