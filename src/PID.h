#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle Params
  */
  const int start_step = 300;         // starts measuring error for twiddle
  const int end_step = 800;           // ending measuring error for twiddle  
  const double twiddle_w_tol = 0.2;   // Twiddle weights tolerance
  const int max_iters = 100;           // Maximum number of iterations
  const int num_params = 3;

  
  double p[3];
  double dp[3];
  double best_p[3];
  double best_error;
  double twiddle_error;
  
  int steps;
  int twiddle_pass;
  int twiddle_iter;

  bool use_twiddle = false;
  bool is_retrial = false;

  // server
  uWS::WebSocket<uWS::SERVER> server;
  
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
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
   /*
  * Initializes twiddle parameters.
  */
  void InitializeTwiddle();

 /*
  * Optimizes tau values (Kp, Ki, Kd).
  */
  void Twiddle();

 /*
  * Restarts simulator.
  */
  void RestartSim(uWS::WebSocket<uWS::SERVER> ws);

 /*
  * Stores server info.
  */
  void SetServer(uWS::WebSocket<uWS::SERVER> ws);

};

#endif /* PID_H */