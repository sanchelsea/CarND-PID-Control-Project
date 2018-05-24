#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() 
{
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) 
{
        this->p_error = 0.0;
        this->d_error = 0.0;
        this->i_error = 0.0;
		p[0] = Kp;
        p[1] = Ki;
        p[2] = Kd;
		twiddle_error = 0;
        steps = 0;
}

void PID::UpdateError(double cte) {
    
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;   
}

double PID::TotalError() 
{
    return (p[0] * p_error) + (p[2] * d_error) +( p[1] * i_error);
}

void PrintParams(double best_p[])
{
	for(int i =0; i<3;i++)
	{
		cout<< "\tParam : "<< best_p[i];
	}
	cout<< endl;
}

//Referred https://github.com/bguisard/CarND-PID-Control-Project for Twiddle implementation using sim restart.

void PID::Twiddle(){

  
  if (use_twiddle) 
  {
	  if (steps == start_step) {
      twiddle_error = pow(p_error, 2);
    } 
    if (steps > start_step) {
      twiddle_error += pow(p_error, 2);
    }
    if (steps > end_step) {
      cout << "\ntotal error = " << twiddle_error << endl; 

    if (best_error == -1) 
	{
		best_error = twiddle_error + 1;
	}

    double sum_dp = accumulate(begin(dp), end(dp), 0.0, plus<double>());

    if ((sum_dp > twiddle_w_tol) & (twiddle_iter < max_iters)) {

        //Debugging steps
        cout << " Iteration " << twiddle_iter << " best error = " << best_error << endl;

        // resets parameter we are optimizing (replaces the for loop)
        if (twiddle_pass >= num_params) twiddle_pass = 0;

        p[twiddle_pass] += dp[twiddle_pass];
        cout << "Optimizing Parameter " << twiddle_pass << endl;

        if (is_retrial) {
          if (twiddle_error < best_error) {
            cout << "Now it improves. Moving to next parameter" << endl;
            best_error = twiddle_error;
            best_p[0] = p[0];
            best_p[1] = p[1];
            best_p[2] = p[2];
            dp[twiddle_pass] *= 1.1;
            twiddle_pass++;
            twiddle_iter++;
            RestartSim(server);
            is_retrial = false;
          } else {
            cout << "Still not improving. Trying smaller dp" << endl;
            p[twiddle_pass] = best_p[twiddle_pass];
            dp[twiddle_pass] *= 0.9;
            p[twiddle_pass] += dp[twiddle_pass];
            twiddle_iter++;
            RestartSim(server);
          }
        }
        else {
          if (twiddle_error < best_error) {
            best_error = twiddle_error;
            cout << "New Error is better than previous best error. Moving to next parameter" << endl;
            dp[twiddle_pass] *= 1.1;
            best_p[0] = p[0];
            best_p[1] = p[1];
            best_p[2] = p[2];
            twiddle_pass++;
            twiddle_iter++;
            RestartSim(server);
          } 
          else {
            p[twiddle_pass] -= 2 * dp[twiddle_pass];
            cout << "New Error is worse than previous best trying in the opposite direction" << endl;          
            is_retrial = true;
            twiddle_iter++;
            RestartSim(server);
          }
        }
      }
      else {
        cout << "\n\n***** Twiddle best parameters *****" << endl;
        cout << " Number of iterations: " << twiddle_iter << endl;
        cout << "Kp = " << best_p[0] << endl;
        cout << "Ki = " << best_p[1] << endl;
        cout << "Kd = " << best_p[2] << endl;
        p[0] = best_p[0];
        p[1] = best_p[1];
        p[2] = best_p[2];
        use_twiddle = false;
      }
    }
    steps++;
  }
}

// Initializes Twiddle parameters
void PID::InitializeTwiddle() {
  // Initializes optimizer parameters
  for (int i = 0; i < num_params; i++) {
    //if (p[i] == 0) 
    dp[i] = 1;
    //else dp[i] = 0.1 * p[i];
  }

  // sets optimizer flag to true
  use_twiddle = true;

  // sets optimizing parameter to 0
  twiddle_pass = 0;

  // Sets best_error to -1 so twiddle knows it's a first run
  best_error = -1;

  // Reset twiddle iteraction counter
  twiddle_iter = 0;

  cout << "\nTwiddle Initialized" << endl;
}

 /*
  * Restarts the simulator and reinitializes PID parameters
  * @param ws the Websocket server info
  */
void PID::RestartSim(uWS::WebSocket<uWS::SERVER> ws) {
  PrintParams(best_p);
  PrintParams(p);
  Init(p[0], p[1], p[2]);
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

 /*
  * Stores server info.
  * @param ws the Websocket server info
  */
void PID::SetServer(uWS::WebSocket<uWS::SERVER> ws) {
  server = ws;
}