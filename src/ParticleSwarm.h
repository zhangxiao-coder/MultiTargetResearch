#pragma once

#include<iostream>
#include <fstream>
#include <cmath>
#include<ctime>
#include<cstdlib>
#include<set>
#include<time.h>

#include"utility.h"
#include"dubins.h"
#include"parameter.h"

using namespace std;

class ParticleSwarm
{
public:
	ParticleSwarm();
	~ParticleSwarm();
	void run();
	void init();
	void spreadParticles(utility::UAV * uav);
	void updateParticleStates(utility::UAV * uav);
	void updateUAVStatesInDubinsState(utility::UAV * uav);
	void updateArtificalPotentialFieldStateImpl(utility::UAV * uav, utility::State& goal, utility::State&next);
	utility::point2D computeRepulsion(utility::point2D& start, utility::point2D& obstacle);
	double computeEnage(utility::point2D& start, utility::point2D& obstacle,double zero_dist);
	double computeAttEnage(utility::State& start, utility::State& end, double zero_dist);
	utility::point2D computeAttraction(utility::point2D& start, utility::point2D& goal);
	void updateTargetStates();
	void updateUAVStates();
	void informationShare();
	void updateMission();
	double dubinsDistance(utility::State& state1, utility::State& state2, double min_R);
	bool isInBound(utility::State& state);
	bool isInRader(utility::State& state);
	bool isInObstacle(utility::State& state);
	void updateSubMap(utility::UAV * uav);
	float computeUncetanty(utility::State& state, double search_r, int id);
	double sigmod(double x);
	double computeAngle(utility::State& start, utility::State& goal);
	utility::Obstacle rectangle;
private:
	vector<vector<utility::grid*>> global_map;
	vector<vector<utility::grid*>> sparse_map;
	vector<utility::TARGET*> target;
	vector<utility::UAV*> uav_;
	vector<utility::radar*> radar_;

	//output
	ofstream output_uav;
	ofstream output_target;
	ofstream output_traj_Point;

	//others
	utility::nion union_;//Disjoint Set
	vector<vector<double>> target_state;
	vector<bool> tracked;
	HybridAStar::DubinsPath* path;
	int cunt = 0;
};

