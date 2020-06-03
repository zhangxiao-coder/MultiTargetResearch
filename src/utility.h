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

namespace utility {
	
	struct point2D {
	public:
		point2D() {

		};
		point2D(double a, double b) {
			x = a; y = b;
		};

		~point2D() {};

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
		std::vector<int> search_count;
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

		double search_r;

		double particle_r;
		int coverd_area_cnt;

		State  Gbest_state;
		//point2D Gbest_position;
		State  last_Gbest_state;
		State  traj_Point;
		double Gbest_fitness;

		std::queue<std::vector<double>> path_;//states
		std::vector<particle*> swarm;
		std::vector<std::pair<point2D, int>> target_position;//
		std::map<int, int> Tj;//��һ����ٵ�Ŀ��ID���ڶ����Ŀ���Ƿ񱻸��õ����˻�׷�٣�0,1
		int track_target_num;
		bool isConvergenced;//
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
		int unionsearch(int root) //���Ҹ����
		{
			int son, tmp;
			son = root;
			while (root != parent[root]) //Ѱ�Ҹ����
				root = parent[root];
			while (son != root) //·��ѹ��
			{
				tmp = parent[son];
				parent[son] = root;
				son = tmp;
			}
			return root;
		}

		void join(int root1, int root2) //�ж��Ƿ���ͨ������ͨ�ͺϲ�
		{
			int x, y;
			x = unionsearch(root1);
			y = unionsearch(root2);
			if (x != y) //�������ͨ���Ͱ��������ڵ���ͨ��֧�ϲ�
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
}



#endif //SRC_UTILITY_H
