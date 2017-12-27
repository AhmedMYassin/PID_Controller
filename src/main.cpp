#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

//#define TUNING_MODE

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

PID pid;

int main()
{
  uWS::Hub h;


  // TODO: Initialize the pid variable.

#ifdef TUNING_MODE

  pid.istuned = false;
  pid.Init(0.1,0,2);

#else

  pid.istuned = true;
  // After Tuning Process
  //pid.Init(0.44,0.03,2.75);

  // These parameters give smooth motion however it's not the best accuracy
  pid.Init(0.3,0.01,2);

#endif

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
          double throttle_value;
          bool out_of_track = false;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          if(pid.steps > 80 && (speed < 2 || cte > 12.0))
          {
           	 out_of_track = true;
          }

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          if(pid.steps < 35)
          {
        	  throttle_value = -(0.1 * (speed - 25));
        	  json msgJson;
        	  msgJson["steering_angle"] = 0;
        	  msgJson["throttle"] = throttle_value;
        	  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
        	  //std::cout << msg << std::endl;
        	  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          else if(pid.steps >= 35 && pid.steps < 800 && !out_of_track)
          {
        	  pid.sum_cte += cte;
        	  pid.abs_sum_cte += fabs(cte);
        	  steer_value = - (pid.Kp * cte) - (pid.Kd * (cte - pid.prev_cte)) - (pid.Ki * pid.sum_cte);
        	  throttle_value = -(0.1 * (speed - 25)) - (1.2 * (cte - pid.prev_cte));
        	  pid.prev_cte = cte;

        	  json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle_value;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              //std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          else
          {
        	  if(out_of_track)
        	  {
        		  pid.UpdateError(10000);
             	  std::cout << "Out Of Track" << std::endl;
              }
        	  else
        	  {
        		  pid.UpdateError((pid.abs_sum_cte/(pid.steps - 35)));
        		  std::cout << "Average Error: " << (pid.abs_sum_cte/(pid.steps - 35)) << " Best Error: " << pid.best_err << std::endl;
        	  }

        	  pid.steps = 0;
        	  pid.sum_cte = 0;
        	  pid.abs_sum_cte = 0;
        	  pid.prev_cte = 0;

        	  if(pid.TotalError() < 0.5)
        	  {
        		  pid.istuned = true;
        		  pid.Kp = pid.Kp_best;
        		  pid.Ki = pid.Ki_best;
        		  pid.Kd = pid.Kd_best;
        		  pid.steps = 50;
        	  }
        	  std::string reset_msg = "42[\"reset\",{}]";
        	  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
          }

          if(!pid.istuned)
          {
        	  pid.steps++;
          }

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

    if(pid.istuned)
    {
    	std::cout << "Model is trained" << std::endl;
    }

    std::cout << "P: " << pid.Kp  << " I: " << pid.Ki  << " D: " << pid.Kd  << std::endl;
	std::cout << "P_error: " << pid.p_error << " I_error: " << pid.i_error << " D_error: " << pid.d_error << std::endl;

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
