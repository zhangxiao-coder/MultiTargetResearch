#pragma once
//common
#include"utility.h"
#include<vector>
using namespace std;

 const double pi = 3.141592653;
 const double dt = 1;
 const double g = 9.8;

//map
 const int resolution = 100;
 const utility::point2D PosMin(0, 0);
 const utility::point2D PosMax(50000, 50000);
 const int size_x = ceil((PosMax.y - PosMin.y) / resolution) + 1;
 const int size_y = ceil((PosMax.y - PosMin.y) / resolution) + 1;
 const int rato = 100;
 const int sparse_resolution = resolution * rato;
 const int sparse_size_x = floor(size_x / rato);
 const int sparse_size_y = floor(size_y / rato);


//target
 const int slow_target_num = 1;
 const int middel_target_num = 1;
 const int fast_target_num = 1;
 const int target_num = slow_target_num + middel_target_num + fast_target_num;

//uav
 const int slow_uav_num = 3;
 const int middel_uav_num = 3;
 const int fast_uav_num = 3;
 const int uav_num = slow_uav_num + middel_uav_num + fast_uav_num;


 const double communication_R = 20000;
 const double forget_time = 1000;

//particle
 const int particle_num = 50;
 const double tao = 1;
 const double w1 = 1;
 const double w2 = 1;
 const double w3 = 0.01;
 const double weight = 1;
 const double c1 = 1;
 const double c2 = 10;
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