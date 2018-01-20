#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

//#define OPTIMIZE

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

#ifdef OPTIMIZE
void print_weights(const MPC& mpc, const int& steps)
{
  std::cerr << "Current weights are:\n";
  std::cerr << "[cte_start + t] = "                             << weights[0] << "\n";
  std::cerr << "[epsi_start + t] = "                            << weights[1] << "\n";
  std::cerr << "[v_start + t] - v_ref = "                       << weights[2] << "\n";
  std::cerr << "[delta_start + t] = "                           << weights[3] << "\n";
  std::cerr << "[a_start + t] = "                               << weights[4] << "\n";
  std::cerr << "[delta_start + t + 1] - [delta_start + t] = "   << weights[5] << "\n";
  std::cerr << "[a_start + t + 1] - [a_start + t] = "           << weights[6] << "\n";
  std::cerr << "[delta_start + t] * [v_start + t] = "           << weights[7] << "\n";
  std::cerr << "steps = "                                       << steps << "\n";
  std::cerr << "v_ref = "                                       << v_ref << std::endl;
}
#endif

int main()
{
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
#ifdef OPTIMIZE
  std::cerr << "T = " << N*dt << "s, N = " << N << ", dt = " << dt << "\n";
  size_t steps = 300;
  size_t counter = steps;
  double sum_cte = 0;
  double avg_speed = 0;
  double prev_MSE = 999999999;
#endif

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px           = j[1]["x"];
          double py           = j[1]["y"];
          double psi          = j[1]["psi"];
          double v            = j[1]["speed"];
#ifdef OPTIMIZE
          avg_speed          += v;
#endif
          v                  *= 0.44704; // now m/s from mph
          double steer_angle  = -static_cast<double>(j[1]["steering_angle"]);
          double a            = j[1]["throttle"];

          // coord conversion to car's reference
          for(size_t i=0; i<ptsx.size(); ++i)
          {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            ptsx[i] = dx*cos(psi) + dy*sin(psi);
            ptsy[i] = -dx*sin(psi) + dy*cos(psi);
          }
          double* p_xvec = ptsx.data();
          double* p_yvec = ptsy.data();
          Eigen::Map<Eigen::VectorXd> ptsx_veh(p_xvec, ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsy_veh(p_yvec, ptsx.size());
          // fit 3rd-deg poly
          Eigen::VectorXd coeffs = polyfit(ptsx_veh, ptsy_veh, 3);

          // State with delay
          double cte = coeffs[0];
#ifdef OPTIMIZE
          sum_cte += abs(cte);
#endif
          //std::cerr << "CTE^2: " << cte*cte << "\n";
          double epsi = -atan(coeffs[1]);
          Eigen::VectorXd state(6);
          psi   = v*steer_angle/Lf*delay;
          double x_   = v*cos(psi)*delay;
          double y_   = v*sin(psi)*delay;
          v     += a*delay;
          cte   += v*sin(epsi)*delay;
          epsi  += v*steer_angle/Lf*delay;


          state << x_, y_, psi, v, cte, epsi;




#ifdef OPTIMIZE
  ++counter;
  if(counter >= steps)
  {
    // RESET
    avg_speed /= counter;
    double current_MSE = sum_cte/static_cast<double>(counter);
    std::cerr << "MSE CTE: " << current_MSE << std::endl;
    printf("AVG speed: %3.0f mph.\n", avg_speed);
    std::string msg = "42[\"reset\",{}]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    print_weights(mpc, steps);
    //
    printf("MSE diff: %+3.2f %% \n", 100*(current_MSE - prev_MSE)/prev_MSE);
    int idx;
    std::cout << "Which value would you like to change?\n";
    std::cout << "0 : \t[cte_start + t]\n"
        << "1 : \t[epsi_start + t]\n"
        << "2 : \t[v_start + t] - v_ref\n"
        << "3 : \t[delta_start + t]\n"
        << "4 : \t[a_start + t]\n"
        << "5 : \t[delta_start + t + 1] - [delta_start + t]\n"
        << "6 : \t[a_start + t + 1] - [a_start + t]\n"
        << "7 : \t[delta_start + t] * [v_start + t]\n"
        << "8 : \tsteps\n"
        << "9 : \tv_ref\n"
        << ": ";
    std::cin >> idx;
    while (idx < 0 || idx > 9)
    {
      std::cout << "Wrong index. Try again!" << ": ";
      std::cin >> idx;
    }
    double old_val;
    if (idx < 8)
      old_val = weights[idx];
    if(idx == 8)
      old_val = steps;
    if(idx == 9)
      old_val = v_ref;
    std::cout << "Old value: " << old_val << "\n";
    double val;
    std::cout << "New value: ";
    std::cin >> val;
    while (val < 0.0)
    {
      std::cout << "Invalid. Try again!" << ": ";
      std::cin >> val;
    }
    if (idx < 8)
      weights[idx] = val;
    if(idx == 8)
      steps = static_cast<int>(val);
    if(idx == 9)
      v_ref = val;
    prev_MSE = current_MSE;
    sum_cte = 0;
    counter = 0;
    avg_speed = 0;
  }
#endif
          vector<double> mpc_result = mpc.Solve(state, coeffs);
          //for(auto it = mpc_result.cbegin(), end = mpc_result.cend(); it != end; ++it)
          //  std::cerr << *it << "\n";

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value    = mpc_result[0] / deg2rad(25);
          double throttle_value = mpc_result[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"]       = throttle_value;



          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc.x;
          msgJson["mpc_y"] = mpc.y;

          //Display the waypoints/reference line
          vector<double> next_x_vals = ptsx;
          vector<double> next_y_vals = ptsy;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
