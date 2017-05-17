#include "PID.h"
#include <iostream>
#include <ctime>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_i, double Ki_i, double Kd_i, double i_err_max) {

   p_error = 0.0f;
   i_error = 0.0f;
   d_error = 0.0f;
   total_error = 0.0f;
   last_output = 0.1f;
   last_timestamp = clock();
   Kp = Kp_i;
   Ki = Ki_i;
   Kd = Kd_i;
   i_error_max = i_err_max;
   cout << "Kp= " << Kp << endl;
   cout << "Ki= " << Ki << endl;
   cout << "Kd= " << Kd << endl;
}

void PID::UpdateError(double cte) {
  int current_timestamp = clock();
  dt = (current_timestamp - last_timestamp) / double(CLOCKS_PER_SEC); //delta time in seconds
  last_timestamp = current_timestamp;
  cout << "dt= " << dt << endl;
  double last_cte = p_error;
  d_error = (cte-last_cte)/dt; //derivative p_error contains last cte
  i_error += cte*dt; // integral
  p_error = cte; // proportional

  if (i_error > i_error_max){
    i_error = i_error_max;
  }
  else if (i_error < -i_error_max){
    i_error = i_error_max;
  }
  // cout << "pe= " << p_error << endl;
  // cout << "ie= " << i_error << endl;
  // cout << "de= " << d_error << endl;

}

void PID::TotalError(double cte) {
  total_error += fabs(cte)*dt;
  cout << "total error: " << total_error <<endl;
}

double PID::control_out(void) {
  double p_out = -Kp*p_error;
  double i_out = -Ki*i_error;
  double d_out = -Kd*d_error;

  cout << "P= " << p_out << endl;
  cout << "I= " << i_out << endl;
  cout << "D " << d_out << endl;




  double output = p_out + i_out + d_out;

  //try smoothing the output
  //double scale = 1.8;
  //if (output > last_output*scale && output > 0.0 && last_output > 0.0) {output = last_output*scale;}
  //if (output < last_output*scale && output < 0.0 && last_output < 0.0) {output = last_output*scale;}

  //last_output = output;
  cout << "Out: " << output << endl;
  return output;
}
