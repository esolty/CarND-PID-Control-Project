#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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

// set intial maxspeed
double maxspeed = 10.0;
bool speed_change = false;
bool beforefirsttwiddle = true;

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.

  // for max speed 10 mph 0.9912540623535,
  //pid.Init(0.1,0.0,0.9912540623535);
  pid.Init(0.08019,0.0,0.5);

  pid.max_iterations = 1000;
  pid.TwiddleEnable = true;

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          double final_maxspeed = 10.0;
          double speed_increment = 5.0; // mph
          double time_increment = 25.0; // s
          pid.current_time = clock(); // get the current time

          // increase maxspeed at time increment by speed increment
          if (maxspeed < final_maxspeed) {
            if (((pid.current_time/1000)/time_increment) > (maxspeed - speed_increment)/speed_increment) {
              maxspeed += speed_increment;
              speed_change = true;
            }
          }

         // manual tunning on first speed change
         // based on past twiddle performance
         if (speed_change) {
           if (maxspeed == 15.0){pid.Kp = 0.11; pid.Ki = 0.0; pid.Kd=0.5;} //pid.Kd=1.41002920092124?
           if (maxspeed == 20.0){pid.Kp = 0.14; pid.Ki = 0.0; pid.Kd=0.5;}
           if (maxspeed == 25.0){pid.Kp = 0.15; pid.Ki = 0.0; pid.Kd=0.5;}
           if (maxspeed == 30.0){pid.Kp = 0.16; pid.Ki = 0.0; pid.Kd=0.5;}
           if (maxspeed == 35.0){pid.Kp = 0.2; pid.Ki = 0.0; pid.Kd=0.5;}
           if (maxspeed == 40.0){pid.Kp = 0.2; pid.Ki = 0.0; pid.Kd=0.5;}
           if (maxspeed == 45.0){pid.Kp = 0.22; pid.Ki = 0.0; pid.Kd=0.5;}
           if (maxspeed == 50.0){pid.Kp = 0.2; pid.Ki = 0.0; pid.Kd=0.5;}
           speed_change = false;
         }

         double throttle_value = pid.speed_control(speed,maxspeed);
         pid.TwiddleEnable = true;

         //pid.current_time = clock(); // get the current time
         double dt = (pid.current_time - pid.previous_time) / CLOCKS_PER_SEC;          
         pid.previous_time = pid.current_time;

         pid.UpdateError(cte, dt); 
         steer_value = pid.TotalError();


         // starting twiddle loop
         if(pid.TwiddleEnable && (maxspeed == final_maxspeed)) {
           pid.num_iterations += 1;
           pid.Total_CTE_per_Twiddle_Iteration(cte);

           // sum restarts and keep going
           if(fabs(cte) > 2.5) {
             pid.Restart(ws);
             pid.current_iter_num_of_restarts += 1;
             //pid.num_iterations = 0;
             pid.stored_error = 0;
             pid.p_error = 0;
             pid.i_error = 0;
             pid.d_error = 0;
             }
          
           // save results by run/cycle or when tolerance is met
           if (pid.num_iterations >= pid.max_iterations || pid.count == 10) {

             // set first lap best error to 1st run error
             if (beforefirsttwiddle) {
               pid.best_err = pid.stored_error/(pid.max_iterations-pid.min_iterations);
               pid.best_err_num_of_restarts = pid.current_iter_num_of_restarts;
               beforefirsttwiddle = false;
             }
             // record results 
             // headers mph; Tot_iters; CE; CE_Rests; BE; BE_rests; Kp_best; Ki_best; Kd_best; Kp; Ki; Kd; dKp; dKi; dKd; p_error; i_error; d_error;
             std::ofstream myfile;
             myfile.open("storage.txt", std::ios_base::app);
             myfile << maxspeed << "; ";
             myfile << pid.all_iterations << "; ";
             myfile << pid.stored_error/fabs(pid.num_iterations-(pid.min_iterations-2)) << "; ";
             myfile << pid.current_iter_num_of_restarts << "; ";
             myfile << pid.best_err << "; ";
             myfile << pid.best_err_num_of_restarts << "; ";
             myfile << pid.Kp_best << "; ";
             myfile << pid.Ki_best << "; ";
             myfile << pid.Kd_best << "; ";
             myfile << pid.Kp << "; ";
             myfile << pid.Ki << "; ";
             myfile << pid.Kd << "; ";
             myfile << pid.dKp << "; ";
             myfile << pid.dKi << "; ";
             myfile << pid.dKd << "; ";
             myfile << pid.p_error << "; ";
             myfile << pid.i_error << "; ";
             myfile << pid.d_error << "\n";
             myfile.close();

             // run twiddle
             pid.Twiddle();
             pid.current_iter_num_of_restarts = 0;
             pid.num_iterations = 0;
           }
           
           // increase final speed if twiddle tolerance met
           if (pid.count == 10) {
             pid.count = 0;
             final_maxspeed += speed_increment;
             pid.num_iterations = 0;
             pid.stored_error = 0;
             pid.p_error = 0;
             pid.i_error = 0;
             pid.d_error = 0;
             beforefirsttwiddle = true;
           }
         }

    json msgJson;
    msgJson["steering_angle"] = steer_value;
    msgJson["throttle"] = throttle_value;
    msgJson["Kp"] = pid.Kp;
    msgJson["dKp"] = pid.dKp;
    msgJson["Kd"] = pid.Kd;
    msgJson["dKd"] = pid.dKd;
    msgJson["delta"] = pid.delta;
    msgJson["iter"] = pid.num_iterations;
    msgJson["run"] = pid.all_iterations/pid.max_iterations;
    //msgJson["E"] = pid.stored_error/fabs(pid.num_iterations-(pid.min_iterations-2));
    msgJson["BE"] = pid.best_err;
    //msgJson["c"] = pid.count;
    msgJson["c"] = pid.controller_term;
    //msgJson["iter_restarts"] = pid.current_iter_num_of_restarts;
    //msgJson["BE_restarts"] = pid.best_err_num_of_restarts;

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
}
