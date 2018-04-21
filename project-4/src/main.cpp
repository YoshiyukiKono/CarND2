#include <uWS/uWS.h>
#include <iostream>
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

int main()
{
  uWS::Hub h;

  //PID pid;
  SteeringControler pid;
  ThrottleControler ctrl_throttle;
  // TODO: Initialize the pid variable.
  
  // Adapted from Q&A https://www.youtube.com/watch?v=YamBuzDjrs8&index=4&list=PLAwxTw4SYaPnfR7TzRZN-uxlxGbqxhtm2
  //const double K_P = -0.5;
  //const double K_I = 0;
  //const double K_D = -0.5;
  
  const double TOL = 0.1;//5;//2;

  const bool FLG_TWIDDLE = false;//true;//true;
  
  if (FLG_TWIDDLE == true) {
    //pid.Init(K_P, K_I, K_D);
    //pid.InitTwiddle(1.1, 0.0001, 5, 0.1);
    //pid.InitTwiddle(0.1, 0.0001, 5, 0.1);
    //pid.InitTwiddle(0.5, 0.0001, 5, 0.1);
    pid.InitTwiddle(0.2, 0.0001, 3, 0.1);
    //pid.InitTwiddle(-1, 0.0001, -0.5, 0.1);
    //pid.InitTwiddle(K_P, K_I, K_D, TOL);
  } else {
    pid.Init(0.16, 0.0001, 3);
    //pid.Init(0.16, 0.0001, 3);
    //pid.Init(0.5, 0.0001, 5);
    //pid.Init(K_P, K_I, K_D);
  }
  ////////////////////////////////////////////////
  //ctrl_throttle.Init(0.5, 0.00001,0.3);//1); It did NOT workd
  //ctrl_throttle.Init(0.75, 0.00001,0.3);//1); It worked so so.
  //ctrl_throttle.Init(0.8, 0.00001,0.3);//1); It worked so so.
  //ctrl_throttle.Init(0.8, 0.0,0.3); // Drive 2 round BEST?
  ctrl_throttle.Init(0.8, 0.0,0.3);
  //ctrl_throttle.Init(0.75, 0.0,0.3); // Better
  //ctrl_throttle.Init(0.8, 0.0,1.2); //DO NOT WORK in 2nd round
  //ctrl_throttle.Init(0.8, 0.0,1);//1); Somehow
  //ctrl_throttle.Init(0.8, 0.0,0.8);//1);DO NOT
  
  //ctrl_throttle.Init(0.8, 0.0,0.3);//1); It somehow work 
  //ctrl_throttle.Init(1, 0,1); It did NOT work.


  //h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
  h.onMessage([&pid, &ctrl_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          // DEBUG
          //std::cout << "CTE: " << cte << " Speed: " << speed << std::endl;
          double throttle_value;

          if (speed < 0.1 & FLG_TWIDDLE) {
std::cout << "Will Restart -> CTE: " << cte << " Speed: " << speed << std::endl;
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          pid.UpdateError(cte);
          ctrl_throttle.UpdateError(fabs(cte));
          
          if (pid.IsReset()) {
//std::cout << "[RESET] CTE:" << cte << " Kp:" << pid.Kp << " Ki:" << pid.Ki << " Kd:" << pid.Kd << std::endl;
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          steer_value = pid.steer_value;
          throttle_value = ctrl_throttle.throttle_value;

          // DEBUG
          //std::cout << "Kp:" << pid.Kp << " Ki:" << pid.Ki << "Kd:" << pid.Kd << std::endl;

          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;//throttle_value;//0.3;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

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
