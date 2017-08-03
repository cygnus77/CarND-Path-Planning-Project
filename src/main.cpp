#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "spline.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

#define SPEED_LIMIT 20.0  // m/s = 50 mph
#define TIME_TO_MAX 5.0      // 0 to 50 in 20 sec
#define SAFE_FOLLOWING_DISTANCE 10 // meters
#define MPH2MS  0.447027269

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

void JMT(vector<double>& dst, const vector<double>& start, const vector <double>& end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    double tsq = T * T;
    double tcu = tsq * T;
    MatrixXd Tmat(3,3);
    Tmat << tcu, tsq*tsq, tcu*tsq, 3*tsq, 4*tcu, 5*tsq*tsq, 6*T, 12*tsq, 20*tcu;
    
    double Sf = end[0], Sf_d = end[1], Sf_dd = end[2];
    double Si = start[0], Si_d = start[1], Si_dd = start[2];
    MatrixXd Sfmat(3,1);
    Sfmat << Sf - (Si + Si_d*T + 0.5*Si_dd*T*T), Sf_d - (Si_d + Si_dd*T), Sf_dd - Si_dd;
    
    MatrixXd alpha = Tmat.inverse() * Sfmat;
    
    dst.assign({Si, Si_d, 0.5*Si_dd, alpha(0), alpha(1), alpha(2)});
    
}

double poly_eval(const vector<double>& coeffs, double x)
{
  double result = 0;
  for(int i = 0; i < coeffs.size(); i++)
  {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

double velocity_at(double x1, double y1, double x2, double y2, double t)
{
  double vel = sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) ) / t;
  return vel;
}

vector<double> derivative(const vector<double>& coeffs)
{
  vector<double> d;
  for(int i = 1; i < coeffs.size(); i++) {
    d.push_back(i * coeffs[i]);
  }
  return d;
}

void print_trajectory(const string& msg, const vector<double>& coeffs)
{
  std::cout << msg;
  for(const double& p: coeffs){
    std::cout << "(" << p << "*t^" << (&p - &coeffs[0]) << ") + ";
  }
  std::cout << std::endl;
}

double laneToD(int lane)
{
  return 2.0 + (4.0*lane);
}

class Car
{
public:
  int id;
  double x,y,vx,vy,s,d;
  Car(const std::vector<double>& f){
    id = f[0];
    x = f[1];
    y = f[2];
    vx = f[3];
    vy = f[4];
    s = f[5];
    d = f[6];
  }
  Car() : id(-1) {}

  void print()
  {
    std::cout << "car: " << id << ",x:" << x << ",y:" << y << ",vx:" << vx << ",vy:" << vy << ",s:" << s << ",d:" << d << std::endl;
  }
  bool collidesWithXY(double tr_x, double tr_y)
  {
    return (tr_x >= x - 1 && tr_x <= x + 1 && tr_y >= y - 1 && tr_y <= y + 1);
  }
  bool collidesWithSD(double tr_s, double tr_d)
  {
    return (tr_s >= s - 1 && tr_s <= s + 1 && tr_d >= d - 1 && tr_d <= d + 1);
  }
  double distance_from(double s2)
  {
    double s_dist;
    if(s > s2) {
      s_dist = s - s2;
    } else {
      s_dist = (max_s - s2) + s;
    }
    return s_dist;
  }
  bool in_same_lane(int lane) {
    return abs(d-laneToD(lane))<=2.0;
  }
  double getVS() {
    return sqrt(vx*vx + vy*vy);
  }
  operator bool() const
  {
    return id != -1;
  }
};

#define clip(x) x = x < 1e-5 ? 0 : x

enum class FSM :int {KeepInLane, LookToSwitch, SwitchLeft, SwitchRight};

bool IsSafeToSwitch(int lane, double s, double d, double us, const vector<vector<double> >& sensor_fusion){
  
  // Enumerate vehicles in target lane
  for(const vector<double>& car_info: sensor_fusion) {
    Car car(car_info);
    if(car.in_same_lane(lane)) {
      // Could it collide in a lane change ?
      
      // behind and too close
      if(s - car.s > 0 && s - car.s < SAFE_FOLLOWING_DISTANCE)
        return false;
      // behind but faster
      if(s - car.s > 0 && s - car.s < 2*SAFE_FOLLOWING_DISTANCE && car.getVS() >= us)
        return false;
      // in front and too close
      if(car.s - s > 0 && car.s - s < 5 * SAFE_FOLLOWING_DISTANCE)
        return false;
    }
  }
  return true;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Set trajectory distance and time         
  const int tr_len = 100;
  const double tr_T = tr_len * 0.02;
  vector<double> Scoeffs, Dcoeffs;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  FSM state = FSM::KeepInLane;
  int currentLane = 1;

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {

    
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
            double car_speed_ms = car_speed * MPH2MS;

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

            // Path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            int prev_path_size = previous_path_x.size();

            //std::cout << "telemetry " << prev_path_size << "; car_speed_ms: " << car_speed_ms << "; car(s,d):" << car_s << "," << car_d << "; end(s,d):" << end_path_s << "," << end_path_d << "car_xy: (" << car_x << "," << car_y << ")" << std::endl;

            double s, d, as, ad, us, ud;
            if(prev_path_size == 0)
            {
              s = car_s;
              us = 0;
              as = 0;

              d = car_d;
              ud = 0;
              ad = 0;
            }
            else
            {

              double T = (tr_len - prev_path_size)*0.02;

              s = poly_eval(Scoeffs, T);
              d = poly_eval(Dcoeffs, T);

              // Calculate velocities and accelaration from previous JMT
              us = poly_eval(derivative(Scoeffs), T);
              as = poly_eval(derivative(derivative(Scoeffs)), T);
              ud = poly_eval(derivative(Dcoeffs), T);
              ad = poly_eval(derivative(derivative(Dcoeffs)), T);
            }

            // Trajectory end configuration
            double final_s, final_vs;
            double final_vd = 0;
            double final_d;

            if(state == FSM::LookToSwitch) {
              if(currentLane == 0) {
                if(IsSafeToSwitch(1, s, d, us, sensor_fusion))
                  state = FSM::SwitchRight;
                else
                  state = FSM::KeepInLane;
              }
              else if(currentLane == 1) {
                if(IsSafeToSwitch(0, s, d, us, sensor_fusion))
                  state = FSM::SwitchLeft;
                else if(IsSafeToSwitch(2, s, d, us, sensor_fusion))
                  state = FSM::SwitchRight;
                else
                  state = FSM::KeepInLane;
              }
              else if(currentLane == 2) {
                if(IsSafeToSwitch(1, s, d, us, sensor_fusion))
                  state = FSM::SwitchLeft;
                else
                  state = FSM::KeepInLane;
              }
            }

            if(state == FSM::SwitchLeft || state == FSM::SwitchRight) {
              // Completed switch ??
              int newLane = currentLane + (state==FSM::SwitchRight ? 1 : -1);
              if( abs(laneToD(newLane) - d) < 2.0 ) {
                currentLane = newLane;
                state = FSM::KeepInLane;
              }
            }

            if(state == FSM::KeepInLane) {

              // Collision detection
              double min_car_dist = max_s;
              Car car_front;
              for(const std::vector<double>& carinfo: sensor_fusion) {
                Car car(carinfo);
                if(car.in_same_lane(currentLane))
                {
                  double s_dist = car.distance_from(s);
                  if(s_dist < min_car_dist) {
                    car_front = car;
                    min_car_dist = s_dist;
                  }
                }
              }
              // if(car_front) {
              //   cout << "car_s = " << car_s << ", s = " << s << " ";
              //   cout << "Nearest car: " << car_front.id << " @ " << car_front.s << ", gap=" << min_car_dist << ", velocity: " << car_front.getVS() << endl;
              // }

              // Set trajectory parameters
              double targetSpeed = SPEED_LIMIT;
              double accel;
              if(min_car_dist < 0 || min_car_dist > 3*SAFE_FOLLOWING_DISTANCE) {
                // No car ahead
                // Calculate end configuration for full speed driving
                accel = min((targetSpeed - us)*(SPEED_LIMIT/TIME_TO_MAX), (SPEED_LIMIT/TIME_TO_MAX));

              } else {
                // Car ahead, start following
                // decelaration needed to get to safe distance behind vehicle
                double slack = min_car_dist - SAFE_FOLLOWING_DISTANCE;
                if(slack < 0) // too close, max decelaration, avoid -ve t
                  accel = -SPEED_LIMIT/TIME_TO_MAX;
                else{
                  double t = slack / us;
                  accel = (car_front.getVS() - us) / t;
                }
              }

              final_s = s + us * tr_T + 0.5 * accel * tr_T * tr_T;
              final_vs = us + accel * tr_T;

              // Keep in lane
              final_d = laneToD(currentLane);
              final_vd = (final_d-car_d)/tr_T;

              if(min_car_dist - SAFE_FOLLOWING_DISTANCE < 15)  {
                state = FSM::LookToSwitch;
              }
            }
            else if(state == FSM::SwitchLeft || state == FSM::SwitchRight) {

              int newLane = currentLane + (state==FSM::SwitchRight ? 1 : -1);
              double targetSpeed = SPEED_LIMIT;
              double accel = min((targetSpeed - us)*(SPEED_LIMIT/TIME_TO_MAX), (SPEED_LIMIT/TIME_TO_MAX));
              final_s = s + us * tr_T + 0.5 * accel * tr_T * tr_T;
              final_vs = us + accel * tr_T;
              
              final_d = laneToD(newLane);
              final_vd = (final_d-car_d)/tr_T;
            }
            

            // Trajectory Generation
            vector<double> Si = { s, us, as };
            vector<double> Sf = { final_s, final_vs, 0 };

            vector<double> Di = { d, ud, ad };
            vector<double> Df = { final_d, final_vd, 0 };

            // std::cout << "accel:" << accel << std::endl;
            // std::cout << "{" << s << "," << us << "," << as << "} - {"<< final_s << "," << final_vs << "," << 0 << "}" << std::endl;
            // std::cout << "{" << d << "," << ud << "," << ad << "} - {"<< final_d << "," << final_vd << "," << 0 << "}" << std::endl;
            
            JMT(Scoeffs, Si, Sf, tr_T);
            JMT(Dcoeffs, Di, Df, tr_T);

            // Generate points for spline
            // add up to 3 points from previous trajectory - this lets the simulator ensure continuity
            // add some points from desired trajectory
            //  - for low speeds use only two points, for higher speeds, use 4 points
            std::vector<double> Xpts, Ypts, Tpts;

            if(prev_path_size > 0) {
              for(int i = 0; i < 2; i++) {
                // Add car's current position to trajectory spline generation
                Tpts.push_back(i*0.02);
                Xpts.push_back(previous_path_x[i]);
                Ypts.push_back(previous_path_y[i]);
              }
            } else {
              Tpts.push_back(0);
              Xpts.push_back(car_x);
              Ypts.push_back(car_y);
            }

            // Create spline for twice the path length so there is smoothness to the curve
            for(double t = tr_T/2.0; t <= tr_T; t += tr_T/4.0) {
              double s = poly_eval(Scoeffs, t);
              double d = poly_eval(Dcoeffs, t);
              if(s > max_s) s -= max_s;
              vector<double> xy = getXY(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              Xpts.push_back(xy[0]);
              Ypts.push_back(xy[1]);
              Tpts.push_back(t);
            }

            // for(int i = 0; i < Tpts.size(); i++) {
            //   std::cout << Tpts[i] << ": " << Xpts[i] << "," << Ypts[i] << std::endl;
            // }

            tk::spline Xspline, Yspline;
            Xspline.set_points(Tpts,Xpts);
            Yspline.set_points(Tpts,Ypts);


            // Trjaectory execution
            for(int i = 0; i < tr_len; i++)
            {    
              double x = Xspline(i*0.02);
              double y = Yspline(i*0.02);
              //std::cout << x << "\t" << y << std::endl;
              next_x_vals.push_back(x);
              next_y_vals.push_back(y);
            }

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
















































































