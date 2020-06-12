#pragma once
//
// Created by bb on 2020/3/9.
//

#ifndef SRC_UTILITY_H
#define SRC_UTILITY_H
#include"stdafx.h"


#include <vector>
#include <map>
#include <queue>
#include<cmath>
namespace utility {
	
	struct point2D {
	public:
		point2D() {

		};
		point2D(double a, double b) {
			x = a; y = b;
		};

		~point2D() {};
		double distance() {
			return sqrt(x*x+y*y);
		}

		double angle() {
			double pi = 3.141592653;
			return y >= 0 ? acos(x / sqrt(x*x + y * y)) : 2 * pi - acos(x / sqrt(x*x + y * y));
		};

		point2D operator+(const point2D& b) {
			point2D temp;
			temp.x = this->x + b.x;
			temp.y = this->y + b.y;
			return temp;
		}
		point2D operator-(const point2D& b) {
			point2D temp;
			temp.x = this->x - b.x;
			temp.y = this->y - b.y;
			return temp;
		}
		point2D operator-() {
			this->x = -this->x;
			this->y = -this->y;
			return *this;
		}
		point2D operator*(double T) {
			point2D temp;
			temp.x = this->x*T;
			temp.y = this->y*T;
			return temp;
		}
		
		double x;
		double y;
	};

	inline double dist(point2D& a, point2D& b) {
		return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
	}

	struct State {
		point2D position;
		point2D velocity;
		point2D accelerate;
	};

	class TARGET
	{
	public:
		TARGET() {

		};
		~TARGET() {

		};
		State  state;
		double search_r;
		double Vel_limite[2];
		double Acc_limite_x[2];
		double Acc_limite_y[2];
	};

	class particle
	{
	public:
		particle() {

		};
		~particle() {

		};
		State  state;

		double p[3];

		double fitness;
		State Pbest_state;

	};

	class radar {
	public:
		radar() {};
		~radar() {};
		point2D center_point;
		double R;
	};


	class grid
	{
	public:
		grid() {};
		~grid() {};
		std::vector<float> search_time;
		bool occupy;
	};

	class UAV
	{
	public:
		UAV() {
		};
		~UAV() {
		};

		int id;
		State  state;
		double Vel_limite[2];
		double Acc_limite_x[2];
		double Acc_limite_y[2];

		double h;
		double search_r;
		int coverd_area_cnt;
		
		State  Gbest_state;
		State  traj_Point;
		double Gbest_fitness;

		std::queue<State> path_;//states
		std::vector<particle*> swarm;
		std::vector<std::pair<State, float>> target_state;//
		std::map<int, int> Tj;//第一项被跟踪的目标ID，第二项该目标是否被更好的无人机追踪，0,1
		int track_target_num;
		std::vector<State> prev_poses;

		void updatePrevPoses() {
			if (prev_poses.size() < 5) {
				prev_poses.push_back(traj_Point);
			}
			else {
				prev_poses.erase(prev_poses.begin());
				prev_poses.push_back(traj_Point);
			}
		}
		double calculateW3(){
			double res = 1;
			if (prev_poses.size() > 1) {

				for (auto i = 0; i < prev_poses.size() - 1; i++) {
					for (int j = i+1; j < prev_poses.size() ; j++) {
						res = dist(prev_poses[i].position, prev_poses[j].position)>res? dist(prev_poses[i].position, prev_poses[j].position):res;
					}
				}
			}
			return res;
		}
	};

	class nion {
	public:
		nion() {};
		~nion() {};
		std::vector<int> parent;
		std::vector<std::vector<int>> barrel;
		int unionsearch(int root) //查找根结点
		{
			int son, tmp;
			son = root;
			while (root != parent[root]) //寻找根结点
				root = parent[root];
			while (son != root) //路径压缩
			{
				tmp = parent[son];
				parent[son] = root;
				son = tmp;
			}
			return root;
		}

		void join(int root1, int root2) //判断是否连通，不连通就合并
		{
			int x, y;
			x = unionsearch(root1);
			y = unionsearch(root2);
			if (x != y) //如果不连通，就把它们所在的连通分支合并
				parent[y] = x;
		}
		void setup_barrel() {
			barrel.clear();
			for (int i = 0; i<parent.size(); i++) {
				if (i == parent[i]) {
					std::vector<int> temp;
					temp.push_back(i);
					barrel.push_back(temp);
				}
				else {
					for (int j = 0; j<barrel.size(); j++) {
						if (barrel[j].front() == parent[i]) {
							barrel[j].push_back(i);
							break;
						}
					}
				}
			}
		};
	};

	class Obstacle {
	public:
		Obstacle() {};
		~Obstacle() {};
		std::vector<point2D> point_lists;
	};

}



#endif //SRC_UTILITY_H
