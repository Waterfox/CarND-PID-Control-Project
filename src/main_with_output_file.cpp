#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream>
#include <sstream>
#include <stdlib.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

//This program requires a output file name when running, it will not warn you!
// ./pid output_file.txt
int main(int argc, char* argv[])
{
  uWS::Hub h;

  std::string out_file_name_ = argv[1];
  std::ofstream out_file_(out_file_name_.c_str(), std::ofstream::out);
  out_file_ << "steering angle" << "\t";
  out_file_ << "CTE" << "\n";

  //PID pid;
  // TODO: Initialize the pid variable.

  //create a PID instance for steering
  PID pid;
  // 35MPH-45MPH (speed_setpoint below)
  double Kp = 0.1150; //0.0950
  double Ki = 0.05;
  double Kd = 0.0020; //0.0025
  double i_err_max = 1.0;

  //55 MPH - drives course but touches rail at end of course
  /*
  double Kp = 0.1050; //0.0950
  double Ki = 0.05;
  double Kd = 0.00220; //0.0025
  double i_err_max = 0.70;
  */

  //create a PID instance for throttle
  PID pid_th;
  double Kp_th = 1.0;
  double Ki_th = 0.5;
  double Kd_th = 0.0002;
  double i_err_max_th = 2.0;


  //std::cout << "nacho";

  pid.Init(Kp,Ki,Kd,i_err_max);
  pid_th.Init(Kp_th,Ki_th,Kd_th,i_err_max_th);

  h.onMessage([&pid, &pid_th, &out_file_](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          pid.UpdateError(cte);
          pid.TotalError(cte);

          steer_value = pid.control_out();

          //set the desired speed
          double speed_setpoint = 40.0;

          // slow the vehicle if we start to steer hard
          if (fabs(steer_value) > 1.0) {
            speed_setpoint = speed_setpoint*0.6;
          }

          //update the throttle error
          pid_th.UpdateError(speed-speed_setpoint);
          //calculate the throttle value
          throttle_value = pid_th.control_out();

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          out_file_ << steer_value << "\t";
          out_file_ << cte << "\n";

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }
}
