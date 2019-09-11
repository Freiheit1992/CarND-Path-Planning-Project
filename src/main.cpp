#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

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
  // vector<vector<double>> lane_x, lane_y;
  // vector<tk::spline> lanes;
  // vector<double> map_waypoints_x2;
  // vector<double> map_waypoints_y2;
  // vector<double> map_waypoints_x3;
  // vector<double> map_waypoints_y3;
  // for (int i=0; i<3; i++)
  // {
  //   vector<double> temp_x, temp_y;
  //   for(int j=0; j<map_waypoints_x.size(); j++)
  //   {
  //     temp_x.emplace_back(map_waypoints_x[j] - (2 + 4*i)*map_waypoints_dx[j]);
  //     temp_y.emplace_back(map_waypoints_y[j] - (2 + 4*i)*map_waypoints_dy[j]);
  //   }

  //   lane_x.emplace_back(temp_x);
  //   lane_y.emplace_back(temp_y);
  // }
  // std::cout<<lane_x.size()<<", "<<lane_x[0].size()<<", "<<lane_x[1].size()<<std::endl;
  // std::cout<<lanes.size()<<lanes[0](0)<<", "<<lanes[1](0);
  double tar_speed = 0.2; // m/s
  int tar_lane = 1;
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &tar_speed, &tar_lane]
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
          double pos_x, pos_y, angle;
          vector<double> ptsx, ptsy;
          int path_size = previous_path_x.size();

          if (path_size > 0)
            car_s = end_path_s;
          int curr_lane = int(car_d / 4);
          // std::cout<<curr_lane<<std::endl;
          vector<double> nearest_dist(5, 9999);
          vector<vector<double>> v_dist;
          for (int i=0; i<5; i++){
            vector<double> temp;
            for (int j=0; j<2; j++){
              temp.push_back(-1);
            }
            v_dist.push_back(temp);
          }
          for(int i=0; i<sensor_fusion.size(); i++)
          {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double v = sqrt(vx*vx + vy*vy);
            double env_x = sensor_fusion[i][1];
            double env_y = sensor_fusion[i][2];

            double est_x = env_x + vx*0.02*path_size;
            double est_y = env_y + vy*0.02*path_size;
            double env_s = getFrenet(est_x, est_y, atan2(vy, vx),
                          map_waypoints_x, map_waypoints_y)[0];
            double env_d = getFrenet(est_x, est_y, atan2(vy, vx),
                          map_waypoints_x, map_waypoints_y)[1];
            int env_lane = env_d/4;
            if (tar_lane - env_lane == 1)
            {
              if (env_s - car_s > 0 && env_s - car_s < nearest_dist[0])
              {
                nearest_dist[0] = env_s - car_s;
                v_dist[0][0] = v;
                v_dist[0][1] = env_s - car_s;
              }
              else if (env_s - car_s <= 0 && car_s-env_s < nearest_dist[1])
              {
                nearest_dist[1] = car_s-env_s;
                v_dist[1][0] = v;
                v_dist[1][1] = car_s-env_s;
              }
            }
            else if (env_lane == tar_lane)
            {
              if (env_s - car_s > 0 && env_s - car_s < nearest_dist[2])
              {
                nearest_dist[2] = env_s - car_s;
                v_dist[2][0] = v;
                v_dist[2][1] = env_s - car_s;
              }
            }
            else if (tar_lane- env_lane== -1)
            {
              if (env_s - car_s > 0 && env_s - car_s < nearest_dist[3])
              {
                nearest_dist[3] = env_s - car_s;
                v_dist[3][0] = v;
                v_dist[3][1] = env_s - car_s;
              }
              else if (env_s - car_s <= 0 && car_s-env_s < nearest_dist[4])
              {
                nearest_dist[4] = car_s-env_s;
                v_dist[4][0] = v;
                v_dist[4][1] = car_s-env_s;
              }
            }
          }

          if (v_dist[2][0] > 0 && v_dist[2][1] < 25 && car_speed > v_dist[2][0])
          {
            if (tar_lane > 0 && v_dist[1][1]>5 && v_dist[0][0] < 0)
              tar_lane -= 1;
            else if (tar_lane < 2 && v_dist[4][1]>5 && v_dist[3][0] < 0)
              tar_lane += 1;
            else if (tar_lane > 0 && v_dist[1][1]>5 && v_dist[0][1] >20 && v_dist[0][0] > std::max(v_dist[2][0], v_dist[3][0]))
              tar_lane -= 1;
            else if (tar_lane < 2 && v_dist[4][1]>5 && v_dist[3][1] >20 && v_dist[3][0] > std::max(v_dist[2][0], v_dist[0][0]))
              tar_lane += 1;
            else if (tar_lane > 0 && v_dist[1][1]>5 && v_dist[0][1] >60)
              tar_lane -= 1;
            else if (tar_lane < 2 && v_dist[4][1]>5 && v_dist[3][1] >60)
              tar_lane += 1;
            else
            {
              tar_speed -= 0.22;
            }
            std::cout<<"curr_lane: " << curr_lane<< ", tar_lane: " << tar_lane << ", left, dist: "<< v_dist[0][1] <<", right, dist: "<< v_dist[3][1]<<std::endl;
          }
          else if (tar_speed < 22.1)
          {
            tar_speed += 0.22;
          }


          for (int i=0; i<path_size; i++)
          {
            next_x_vals.emplace_back(previous_path_x[i]);
            next_y_vals.emplace_back(previous_path_y[i]);
          }
          if (path_size ==0)
          {
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
            ptsx.emplace_back(pos_x);
            ptsy.emplace_back(pos_y);
          }
          else
          {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];
            double pos_x_2 = previous_path_x[path_size-2];
            double pos_y_2 = previous_path_y[path_size-2];
            ptsx.emplace_back(pos_x_2);
            ptsy.emplace_back(pos_y_2);
            ptsx.emplace_back(pos_x);
            ptsy.emplace_back(pos_y);
            angle = atan2(pos_y - pos_y_2, pos_x - pos_x_2);
          }
          auto s_d = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
          auto s0 = s_d[0];
          double d0 = tar_lane*4+2;
          for (int i=0; i<3; i++)
          {
            double s = s0 + (i + 1)*30;
            auto x_y = getXY(s, d0, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(x_y[0]);
            ptsy.push_back(x_y[1]);
          }
          for (int i=0; i<ptsx.size(); i++)
          {
            double relativ_x = ptsx[i] - pos_x;
            double relativ_y = ptsy[i] - pos_y;

            ptsx[i] = cos(angle)*relativ_x + sin(angle)*relativ_y;
            ptsy[i] = cos(angle)*relativ_y - sin(angle)*relativ_x;
          }
          tk::spline s;
          s.set_points(ptsx, ptsy);

          double dist_inc = tar_speed/50; // m
          double x_temp = 0.0;
          double tar_x = 30.;
          double tar_y = s(tar_x);
          double dist_tar = sqrt(tar_x*tar_x + tar_y*tar_y);
          double N = dist_tar/dist_inc;
          // std::cout<<path_size<<", "<<car_yaw<<std::endl;
          for (int i = 0; i < 30-path_size; ++i) //
          {    
            double x_point = x_temp + tar_x/N;
            double y_point = s(x_point);
            x_temp = x_point;

            double abs_x = x_point*cos(angle) - y_point*sin(angle) + pos_x;
            double abs_y = x_point*sin(angle) + y_point*cos(angle) + pos_y;
            // double s = s0 + (i + 1)*dist_inc;
            // auto x_y = getXY(s, d0, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(abs_x);
            next_y_vals.push_back(abs_y);


          }

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