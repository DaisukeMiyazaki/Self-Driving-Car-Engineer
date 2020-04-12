	#include <uWS/uWS.h>
	#include <fstream>
	#include <iostream>
	#include <string>
	#include <vector>
	#include <cmath>
	#include <algorithm>
	#include "Eigen-3.3/Eigen/Core"
	#include "Eigen-3.3/Eigen/QR"
	#include "helpers.h"
	#include "json.hpp"
	#include "spline.h"

	// for convenience
	using nlohmann::json;
	using std::string;
	using std::vector;
	using namespace std;

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
		
	  // start in lane
	  int lane = 1;
	  int cnst = 60;
	  
	  // reference velocity
	  double ref_vel = 0.; //MPH
	  double othref_vel = 0.;
	  
	  h.onMessage([&ref_vel,&cnst,&othref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
	               &map_waypoints_dx,&map_waypoints_dy,&lane]
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

	          int prev_size = previous_path_x.size();	       
		      int edg_right = 2 + 4 * lane + 2;
		      int edg_left = 2 + 4 * lane - 2;
	        	
	          /****** actions list *******/

	          bool accelerate = false;
	          bool deaccelerate = false;
	          bool hvy_deaccelerate = false;
	          bool lanechange = true;
	          bool intransition = false;
	          
	 
	          int next_lane = 1;
	        			
	/******* end **********/
        
	          if (prev_size > 0)
	          {
	          	  car_s = end_path_s;
	          }

	/************************************************************
	*************       sensor fusion      **********************
	************* looking at other cars and take action *********
	************************************************************/
	        	bool sensor_fusion_switch = false;
	        	
	        	vector<vector<double>> smlane_fr;
	        	vector<vector<double>> smlane_bk;
	        	vector<vector<double>> othlane_fr;
	        	vector<vector<double>> othlane_bk;
	        	
	        	for(int i = 0; i < sensor_fusion.size();i++)
	        	{
	        		// Check all the cars d coordinate
	        		double d = sensor_fusion[i][6];
	        		double s = sensor_fusion[i][5];
	        		double car_vx = sensor_fusion[i][3];
		        	double car_vy = sensor_fusion[i][4];
		        	double sf_speed = sqrt(pow(car_vx,2) + pow(car_vy,2));
		        	s += ((double)prev_size * 0.02 * sf_speed);

	/****** focus on cars within the range of 60 meters ********/
	        		
	        		if( fabs(s - car_s) < cnst )
	        		{

	        			if( d <= edg_right && d > edg_left && s >= car_s)
	        			{
	        				
	        				smlane_fr.push_back(sensor_fusion[i]);
	        				
	        			}
	        			else if( d <= edg_right && d > edg_left && s < car_s)
	        			{
	        				
	        				smlane_bk.push_back(sensor_fusion[i]);
	        				
	        			}
	        			else if(( d > edg_right || d < edg_left) && s >= car_s)
	        			{

	        				othlane_fr.push_back(sensor_fusion[i]);
	        			
	        			}
	        			else if(( d > edg_right || d < edg_left ) && s < car_s)
	        			{
	        				
	        				othlane_bk.push_back(sensor_fusion[i]);
	        				
	        			}
	        		}
	        		
	        	} // end of for(int i = 0; i < sensor_fusion.size();i++)
	        	
	        	int cmpt_times = smlane_fr.size() + smlane_bk.size() + othlane_fr.size() + othlane_bk.size();
	        	
	/****** end ******/
	        	if( smlane_fr.size() != 0)
	        	{
	        		sensor_fusion_switch = true;
	        	}
				else
	        	{
	        		sensor_fusion_switch = false;
	        	}

	        	
	        	if(sensor_fusion_switch)
		        {

	               
		/********** Focus on a car on the same lane ******/

		        	/*** specify the car at front ***/
		        	vector<double> car_front;
		        	double min = 10000000000;
		        	int car_front_ind;
		        	for(int i = 0; i < smlane_fr.size() ; i++)
		        	{
		        		double small_s = smlane_fr[i][5];
		        		if(small_s < min)
		        		{
		        			min = small_s;
		        			car_front_ind = i;
		        		}
		        	}
			        
		        	car_front = smlane_fr[car_front_ind];
		        	
		        	
		        	double car_f_s = car_front[5];
		        	double car_f_vx = car_front[3];
		        	double car_f_vy = car_front[4];
		        	double car_f_speed = sqrt(pow(car_f_vx,2) + pow(car_f_vy,2));
		        	double fut_car_f_s = car_f_s + ((double)prev_size * 0.02 * car_f_speed);

		        	double cnst_2 = 15.;
		        	double cnst_3 = 20.;
		        	
		        	double prime_dist = fut_car_f_s - car_s;

		/***********************************/
		/*** when car is at front of me ****/
		/***********************************/
		        	
		        	if(prime_dist > cnst_2 && prime_dist < cnst_3)
		        	{
		        		
		        		othref_vel = car_f_speed;
		        		bool ready2lanechange = true;
	                    int valid_lane = -1;
		        		
		        		if( othlane_fr.size() == 0)
		        		{
		        			if(lane == 0 || lane == 2)
		        			{
		        				valid_lane = 1;
		        			}
		        			else
		        			{
		        				valid_lane = 0;
		        			}

		        		}
		        		else /***  othlane_fr isnt empty ***/
		        		{

		        			bool right_adj = true;
		        			bool left_adj = true;
							
		        			/*** when either of the flag is true, valid lane has a value ***/
		        			
			        		for(int i = 0 ; i < othlane_fr.size() ; i ++ )
			        		{
			        			
			        			/*** see if adjecent lanes are available ***/
			        			/*** if cars at front are aligned, stay in my current lane ***/
			        			double car_othfr_s = othlane_fr[i][5];
			        			double car_othfr_d = othlane_fr[i][6];
			        			
			        			/*** change d value to lane unit ***/
			        			int taboo_d = (int)car_othfr_d / 4;
			        			
			        			if( car_othfr_s < (fut_car_f_s + 5 ))
			        			{
			        				if( lane == 0 )
			        				{
			        					if( taboo_d == lane + 1)
			        					{
			        						ready2lanechange = false;
			        						bool right_adj = false;
		        							bool left_adj = false;
			        						break;
			        					}
			        					else // taboo_d == lane + 2
			        					{
			        						bool left_adj = false;
			        					}
			        					
			        				}
			        				else if( lane == 1 )
			        				{
			        					if( taboo_d == lane + 1 )
			        					{
			        						right_adj = false;
			        					}
			        					else if(taboo_d == lane - 1 )
			        					{
			        						left_adj = false;
			        					}
			        				}
			        				else if( lane == 2 ) 
			        				{
			        					if( taboo_d == lane - 1)
			        					{
			        						ready2lanechange = false;
			        						bool right_adj = false;
		        							bool left_adj = false;
			        						break;
			        					}
			        					else // taboo_d == lane + 2
			        					{
			        						bool right_adj = false;
			        					}
			        					
			        				}
			        			}
			        			else
			        			{
			        				/** DO NOTHING **/
			        			}
			        			
		        			} // end of for(int i = 0 ; i < othlane_fr.size() ; i ++ )
							
		        			/*** depends on the flags, decide which lane to go ***/

							if(left_adj == false && right_adj == false)
			        		{
			        			ready2lanechange = false;
			        		}

			        		else if( left_adj == true && lane >= 1)
			        		{
			        			valid_lane = lane - 1;
			        		}
		        			
		        			else if( right_adj == true && lane <= 1)
		        			{
		        				valid_lane = lane + 1;
		        			}
		        			
		        			else if( right_adj == true && left_adj == true )
		        			{
		        				valid_lane = 0; // leftmost lane is priotized 
		        			}
		        			
		        			/*** end ***/
		        			
		        		} // end of if( othlane_fr.size() == 0) else

		        		
		        		if(ready2lanechange == true && intransition == false)
		        		{

	        				/*** there's some empty lane ahead ****/
	        				/*** observe the back side of empty lane ***/
	        			
	        				for( int k = 0 ; k < othlane_bk.size() ; k++ )
	        				{
	        					double car_othbk_s = othlane_bk[k][5];
	        					double car_othbk_d = othlane_bk[k][6];
	        					int othbk_lane = (int)car_othbk_d / 4;
	        					
	        					
	        					if( othbk_lane == valid_lane )
	        					{
	        						double car_othbk_vx = othlane_bk[k][3];
	        						double car_othbk_vy = othlane_bk[k][4];
	        						double car_othbk_speed = sqrt(pow(car_othbk_vx,2) + pow(car_othbk_vy,2));
	        						
	        						/*** predit about the car's behavior ****
	        						**** the car won't come close soon, lane change ***/
	        						
	        						double fut_car_othbk_s = car_othbk_s + ((double)prev_size * 0.02 * car_othbk_speed);
	        						double margin = 70.;
	        						
	        						if( car_s + margin < fut_car_othbk_s)
	        						{

	        							lanechange = false;
	        							intransition = false;
	        							deaccelerate = true;
                                        break;
	        						}
	        					}
	        					else
	        					{
	        						/*** DO NOTHING ***/
	        					}
	        				} // end of for( int k = 0 ; k < othlane_bk.size() ; k++ )
	        				
	        				if( lanechange )
	        				{
		        				lane = valid_lane;
	        					intransition = true;
	        				}
	        				else
	        				{
	        					/*** DO NOTHING ***/
	        				}
	        			
	        			} // end of if(ready2langechange)
		        		else
		        		{
		        			deaccelerate = true;
		        		}

		        		
		        	} // end of if(prime_dist > cnst_2 && prime_dist < cnst_3 )

		/*** when car is right in front of us, brake storngly ****/
		        	else if( prime_dist <= cnst_2 )
		        	{
		        		othref_vel = car_f_speed;
		        		hvy_deaccelerate = true;
		        	}

		/*************************
		/*** else, accelerate ****
		*************************/
		        	else
		        	{
		        		/** no reference speed from other vehicle **/
		        		othref_vel = 49; 
		        		accelerate = true;
		        	}
		        } // end of sensor_fusion_switch
	        	
		/***** end *****/
		 
		/************************
		/***** take action *****
		************************/

		        if(hvy_deaccelerate)
		        {
		           ref_vel -= 1.;
		        }
		        else if(deaccelerate && ref_vel <= othref_vel)
		        {
		        	ref_vel -= 0.324;
		        }
		        else if(accelerate && ref_vel <= othref_vel)
		        {
		        	ref_vel += 0.224;
		        }
	        	
	 /*** out of sensor fusion ***/
	        	
	        	else if(ref_vel < 49)
	        	{
	        		ref_vel += 0.5;
	        	}
	          
	        	
	        	vector<double> ptsx;
	        	vector<double> ptsy;
	        	
	        	double ref_x = car_x;
	        	double ref_y = car_y;
	        	double ref_yaw = deg2rad(car_yaw);
	        	
	        	// See if previous path is almost empty
	        	if(prev_size < 2)
	        	{
	        		double prev_car_x = car_x - cos(car_yaw);
	        		double prev_car_y = car_y - sin(car_yaw);
	        			
	        		ptsx.push_back(prev_car_x);
	        		ptsx.push_back(car_x);
	        		
	        		ptsy.push_back(prev_car_y);
	        		ptsy.push_back(car_y);

	        	}
	        	// if prevous path has more points 	
	        	else
	        	{
	        		// take last 2 points from the previous path
	        		ref_x = previous_path_x[prev_size - 1];
	        		ref_y = previous_path_y[prev_size - 1];

	        		double ref_x_prev = previous_path_x[prev_size - 2];
	        		double ref_y_prev = previous_path_y[prev_size - 2];
		        		ref_yaw = atan2(ref_y - ref_y_prev,ref_x - ref_x_prev);

	        		// push back the 4 points to the vector

	        		ptsx.push_back(ref_x_prev);
	        		ptsx.push_back(ref_x);

	        		ptsy.push_back(ref_y_prev);
	        		ptsy.push_back(ref_y);

	        	}

	        	vector<double> next_wp0 = getXY(car_s +30,(2 + 4 * lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
	        	vector<double> next_wp1 = getXY(car_s+60,(2 + 4 * lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
	        	vector<double> next_wp2 = getXY(car_s+90,(2 + 4 * lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

	        	ptsx.push_back(next_wp0[0]);
	        	ptsx.push_back(next_wp1[0]);
	        	ptsx.push_back(next_wp2[0]);

	        	ptsy.push_back(next_wp0[1]);
	        	ptsy.push_back(next_wp1[1]);
	        	ptsy.push_back(next_wp2[1]);


	        	for(int i = 0; i< ptsx.size() ; i++ )
	        	{
	        		// shift car 
	        		double shift_x = ptsx[i] - ref_x;
	        		double shift_y = ptsy[i] - ref_y;
	        		
	        		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
	        		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));


	        	}

	        	tk::spline s;

	        	s.set_points(ptsx,ptsy);


	            vector<double> next_x_vals;
	            vector<double> next_y_vals;

	        	for(int i = 0; i < previous_path_x.size();i++)
	        	{
	        		next_x_vals.push_back(previous_path_x[i]);
	        		next_y_vals.push_back(previous_path_y[i]);
	        	}

	        	double target_x =30.0;
	        	double target_y = s(target_x);
	        	double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));

	        	double x_add_on = 0;

	        		for(int i = 1;i <= 50 - previous_path_x.size();i++)
	        		{
	        			double N = (target_dist/(.02*ref_vel/2.24));
	        			double x_point = x_add_on + target_x / N;
	        			double y_point = s(x_point);

	        			x_add_on = x_point;

	        			double x_ref = x_point;
	        			double y_ref = y_point;

	        			x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
	        			y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

	        			x_point += ref_x;
	        			y_point += ref_y;

	        			next_x_vals.push_back(x_point);
	        			next_y_vals.push_back(y_point);
	        		}
	        	
	          /**
	           * TODO: define a path made up of (x,y) points that the car will visit
	           *   sequentially every .02 seconds
	           */

	          //end

			  json msgJson;
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