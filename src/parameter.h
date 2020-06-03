#pragma once
//common
#include"utility.h"
#include<vector>
using namespace std;

 const double pi = 3.141592653;
 const double dt = 1;
 const double g = 9.8;

//map
 const int resolution = 100;//?????50
 const utility::point2D PosMin(0, 0);
 const utility::point2D PosMax(100000, 100000);
 const int size_x = ceil((PosMax.y - PosMin.y) / resolution) + 1;
 const int size_y = ceil((PosMax.y - PosMin.y) / resolution) + 1;
 const int rato = 200;
 const int sparse_resolution = resolution * rato;
 const int sparse_size_x = floor(size_x / rato);
 const int sparse_size_y = floor(size_y / rato);


//target
 const int target_num = 3;


//uav
const int UAV_num = 9;
 const utility::point2D VelMax(200, pi / 6);
 const utility::point2D VelMin(150, -pi / 6);
 const double AccMax = 0.6;
 const double AccMin = -0.6;
 const int search_R = 8000;
 const double communication_R = 50000;//5000
 const double tanTheta = sqrt(1.0 / 3);
 const double forget_time = 1000;

//particle
 const int particle_num = 100;
 const double particle_R = resolution + 2 * VelMax.x * dt;
 const double tao = 1;
 const double w1 = 1;
 const double w2 = 1;
 const double w3 = 1;
 const double weight = 1;
 const double c1 = 2;
 const double c2 = 1;
 const double  w = 0.5;

//output
const string uav_path = "data/uav.txt";
const string target_path = "data/target.txt";
const string traj_Point_path = "data/traj_Point.txt";
const string particle_state_path = "data/particle_state.txt";
const string uav_state_path = "data/uav_state.txt";
const string best_state_path = "data/best_state.txt";

//obstacle
const int radar_num = 1;