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
double maxspeed = 7.5;
bool speed_change = false;
bool beforefirsttwiddle = true;
double Kp_7_5mph = 0.885055;
double Kd_7_5mph = 2.11636;
bool restarted = false;
double restarted_Kp = 0; 
double restarted_Kd = 0; 

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.

  // for max speed 10 mph 0.9912540623535,
  //pid.Init(0.1,0.0,0.9912540623535);
  //pid.Init(0.08019,0.0,0.5);  0.240192, 0.0, 0.915982
  //pid.Init(0.240192, 0.0, 0.91598);
  //pid.Init(0.10018,0.0,0.897308);
      

  pid.Init(Kp_7_5mph, 0.0, Kd_7_5mph);

  pid.max_iterations = 1000;
  pid.TwiddleEnable = false;

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

          double final_maxspeed = 7.5;
          double speed_increment = 2.5; // mph
          double time_increment = 25.0; // s
          pid.current_time = clock(); // get the current time

          if (maxspeed < final_maxspeed) {
            if ((pid.current_time - pid.last_speed_change_time)/1000 > time_increment) {
              maxspeed += speed_increment;
              speed_change = true;
              pid.last_speed_change_time = clock();
            }
          }

         // start values based on past twiddle performance
         if (speed_change) {
           if (maxspeed == 10.0){pid.Kp = 0.531033; pid.Ki = 0.0; pid.Kd=1.08372;}
           if (maxspeed == 12.5){pid.Kp = 0.849653; pid.Ki = 0.0; pid.Kd=5.18981;}
           if (maxspeed == 15.0){pid.Kp = 0.849653; pid.Ki = 0.0; pid.Kd=5.18981;} 
           if (maxspeed == 17.5){pid.Kp = 0.14; pid.Ki = 0.0; pid.Kd=2.5;}
           if (maxspeed == 20.0){pid.Kp = 0.14; pid.Ki = 0.0; pid.Kd=2.5;}
           if (maxspeed == 25.0){pid.Kp = 0.15; pid.Ki = 0.0; pid.Kd=4.5;}
           if (maxspeed == 30.0){pid.Kp = 0.16; pid.Ki = 0.0; pid.Kd=8.5;}
           if (maxspeed == 35.0){pid.Kp = 0.2; pid.Ki = 0.0; pid.Kd=10.5;}
           if (maxspeed == 40.0){pid.Kp = 0.2; pid.Ki = 0.0; pid.Kd=12.5;}
           if (maxspeed == 45.0){pid.Kp = 0.22; pid.Ki = 0.0; pid.Kd=15.5;}
           if (maxspeed == 50.0){pid.Kp = 0.2; pid.Ki = 0.0; pid.Kd=20.0;}
           if (maxspeed == final_maxspeed && restarted_Kp == 0) {restarted_Kp = pid.Kp; restarted_Kd = pid.Kd;}
           speed_change = false;
         }          

         // restart if error too high, twiddle if too many restarts
         if(pid.TwiddleEnable && fabs(cte) > 2.5) {
           pid.Restart(ws);
           pid.num_iterations -= 1;
           pid.current_iter_num_of_restarts += 1;

           if (maxspeed == final_maxspeed && pid.current_iter_num_of_restarts > pid.best_err_num_of_restarts) {
             // record results 
             pid.RecordCycle(maxspeed);
             pid.Twiddle();
             pid.current_iter_num_of_restarts = 0;
             pid.num_iterations = 0;
             pid.last_speed_change_time = pid.current_time + 5000;
             restarted_Kp = pid.Kp;
             restarted_Kd = pid.Kd;
           }

           pid.stored_error = 0;
           pid.p_error = 0;
           pid.i_error = 0;
           pid.d_error = 0;

           if (final_maxspeed != 7.5) {
             maxspeed = 7.5;
             pid.Kp = Kp_7_5mph;
             pid.Ki = 0.0;
             pid.Kd = Kd_7_5mph;
             pid.last_speed_change_time = pid.current_time;
             restarted = true;
           }
         }                  

         // starting twiddle loop
         if(pid.TwiddleEnable && (maxspeed == final_maxspeed)) {
           if (restarted) {
             pid.Kp = restarted_Kp;
             pid.Kd = restarted_Kd;
             restarted = false;
           }
           else {
             pid.num_iterations += 1;
             pid.Total_CTE_per_Twiddle_Iteration(cte);
           }             
         
           if (pid.num_iterations >= pid.max_iterations) {

             // set first lap best error to 1st run error
             if (beforefirsttwiddle) {
               pid.best_err = pid.stored_error/(pid.max_iterations-pid.min_iterations);
               pid.best_err_num_of_restarts = pid.current_iter_num_of_restarts;
               pid.Kp_best = pid.Kp;
               pid.Kd_best = pid.Kd;
               beforefirsttwiddle = false;
             }
             // record results 
             pid.RecordCycle(maxspeed);
             // run twiddle
             pid.Twiddle();
             pid.current_iter_num_of_restarts = 0;
             pid.num_iterations = 0;
             restarted_Kp = pid.Kp;
             restarted_Kd = pid.Kd;
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
           pid.last_speed_change_time = pid.current_time + 5000;
           pid.dKp = 0.4;
           pid.dKd = 0.4;
         }

       }

    //pid.current_time = clock(); // get the current time
    double dt = (pid.current_time - pid.previous_time) / CLOCKS_PER_SEC;          
    pid.previous_time = pid.current_time;

    double throttle_value = pid.speed_control(speed,maxspeed);
    pid.UpdateError(cte, dt); 
    steer_value = pid.TotalError();

    json msgJson;
    msgJson["steering_angle"] = steer_value;
    msgJson["throttle"] = throttle_value;
    msgJson["Kp"] = pid.Kp;
    //msgJson["dKp"] = pid.dKp;
    msgJson["Kd"] = pid.Kd;
    //msgJson["dKd"] = pid.dKd;
    //msgJson["delta"] = pid.delta;
    msgJson["iter"] = pid.num_iterations;
    msgJson["run"] = pid.all_iterations/pid.max_iterations;
    //msgJson["E"] = pid.stored_error/fabs(pid.num_iterations-(pid.min_iterations-2));
    //msgJson["BE"] = pid.best_err;
    msgJson["maxspeed"] = maxspeed;
    //msgJson["c"] = pid.count;
    //msgJson["c"] = pid.controller_term;
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
