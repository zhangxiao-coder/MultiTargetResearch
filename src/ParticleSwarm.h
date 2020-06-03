#pragma once

#include<iostream>
#include <fstream>
#include <cmath>
#include<ctime>
#include<cstdlib>
#include<set>

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
	void updateTargetStates();
	void updateUAVStates();
	void informationShare();
	void updateMission();
	double dubinsDistance(utility::State& state1, utility::State& state2);
	bool isInBound(utility::State& state);
	bool isInRader(utility::State& state);
	void updateSubMap(utility::UAV * uav);
	double sigmod(double x);
private:
	vector<vector<utility::grid*>> global_map;
	vector<vector<utility::grid*>> sparse_map;
	vector<utility::TARGET*> target;
	vector<utility::UAV*> uav;
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
};

