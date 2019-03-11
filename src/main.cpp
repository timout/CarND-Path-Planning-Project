#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

/* ---------*/

#include "spline.h"
#include "planner.hpp"

Planner planner;


const int path_size = 50;

const int spacing[] = { 45, 90, 135 };

/* ---------*/

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals; 
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int prev_path_size = previous_path_x.size();
          int points_to_add = path_size - prev_path_size;

          // copy old path
          for(int i = 0; i < prev_path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          vector<double> ptsx;
          vector<double> ptsy;
        
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if ( prev_path_size < 2 ) { //use current values
            double x_prev = car_x - cos(car_yaw);
            double y_prev = car_y - sin(car_yaw);
            
            ptsx.push_back(x_prev);
            ptsy.push_back(y_prev);

          } else { // use previous values
              ref_x = previous_path_x[prev_path_size-1];
              ref_y = previous_path_y[prev_path_size-1];

              double x_prev = previous_path_x[prev_path_size-2];
              double y_prev = previous_path_y[prev_path_size-2];

              ref_yaw = atan2(ref_y - y_prev, ref_x - x_prev);
              
              ptsx.push_back(x_prev);
              ptsy.push_back(y_prev);
              
            }
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);
          
            // Calculate the rest of the path
            vector<double> frenet_coordinates = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);

            // Call planner to get next d coordinate
            int next_d = planner.execute(frenet_coordinates, sensor_fusion, car_s);
          
            // Generate point to draw spline through them
            for ( int sp : spacing ) {
              vector <double> next_wp = getXY(car_s + sp, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(next_wp[0]); 
              ptsy.push_back(next_wp[1]);
            }
          
            // Convert points to car local 
            for (int i = 0; i < ptsx.size(); i++) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
              
              ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }

            // create spline and set points  
            tk::spline spln;
            spln.set_points(ptsx, ptsy);
              
            // Set horizon
            double target_x = 30;
            double target_y = spln(target_x);
            double target_dist = sqrt( target_x * target_x + target_y * target_y);
              
            double x_add_on = 0;
            for(int i = 0; i < points_to_add; i++) {
              double point_velocity = planner.get_next_velocity(); 
              
              // Calculate points along a new path
              double N = ( target_dist / ( 0.02 * point_velocity ) );
              double x_point = x_add_on + (target_x)/N;
              double y_point = spln(x_point);
              
              x_add_on = x_point;
              
              // Convert points to car local 
              double x_ref = x_point;
              double y_ref = y_point;
              
              x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
              y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
              
              x_point += ref_x;
              y_point += ref_y;
              
              //add path point
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
            // ----------------------------------

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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