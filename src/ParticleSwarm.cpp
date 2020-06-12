#include"stdafx.h"
#include "ParticleSwarm.h"

ParticleSwarm::ParticleSwarm()
{
	target_state = *(new vector<vector<double>>(uav_num, vector<double>(target_num, 0)));
	tracked = *(new vector<bool>(target_num, false));
	path = new HybridAStar::DubinsPath();
}


ParticleSwarm::~ParticleSwarm()
{
}

double ParticleSwarm::sigmod(double x) {
	return 1.0 / (1 + exp(-x));
}

double ParticleSwarm::computeAngle(utility::State& start, utility::State& goal) {
	double l = dist(goal.position, start.position);
	double theTa1 = acos((goal.position.x - start.position.x) / l);

	return (goal.position.y - start.position.y)>0 ? theTa1 : 2 * pi - theTa1;
}

double ParticleSwarm::dubinsDistance(utility::State& state1, utility::State& state2,double min_R) {
	double l = dist(state1.position, state2.position);
	double theTa1 = acos((state2.position.x - state1.position.x) / l);
	theTa1 = (state2.position.y - state1.position.y)>0 ? theTa1 : 2 * pi - theTa1;
	double theTa2 = state1.velocity.y - theTa1;
	if (theTa2 > pi)
		theTa2 -= 2 * pi;
	else if (theTa2 <-pi)
		theTa2 += 2 * pi;
	double theTi, d;

	if (theTa2<-pi / 2) {
		theTa2 += pi;
		d = sqrt((l - min_R * sin(theTa2))*(l - min_R * sin(theTa2)) + min_R * cos(theTa2)*min_R*cos(theTa2));
		theTi = asin(min_R*cos(theTa2) / d) + asin(min_R / d) + theTa1;
	}
	else if (theTa2<0) {
		theTa2 *= -1;
		d = sqrt((l - min_R * sin(theTa2))*(l - min_R * sin(theTa2)) + min_R * cos(theTa2)*min_R*cos(theTa2));
		theTi = asin(min_R / d) - asin(min_R*cos(theTa2) / d) + theTa1;
	}
	else if (theTa2>pi / 2) {
		theTa2 = pi - theTa2;
		d = sqrt((l - min_R * sin(theTa2))*(l - min_R * sin(theTa2)) + min_R * cos(theTa2)*min_R*cos(theTa2));
		theTi = -asin(min_R / d) - asin(min_R*cos(theTa2) / d) + theTa1;
	}
	else {
		d = sqrt((l - min_R * sin(theTa2))*(l - min_R * sin(theTa2)) + min_R * cos(theTa2)*min_R*cos(theTa2));
		theTi = -asin(min_R / d) + asin(min_R*cos(theTa2) / d) + theTa1;
	}

	if (isnan(theTi))
		return 100000.0;

	double q1[3] = { state1.position.x,state1.position.y,state1.velocity.y };//start point state
	double q2[3] = { state2.position.x,state2.position.y,theTi };//end point state;
	HybridAStar::dubins_init(q1, q2, min_R, path);
	return HybridAStar::dubins_path_length(path);
}

bool ParticleSwarm::isInBound(utility::State& state) {
	if (state.position.x <= PosMin.x || state.position.x >= PosMax.x ||
		state.position.y <= PosMin.y || state.position.y >= PosMax.y)
		return false;
	return true;
}

bool ParticleSwarm::isInObstacle(utility::State& state) {
	return rectangle.point_lists[0].x <= state.position.x && state.position.x <= rectangle.point_lists[2].x &&
		rectangle.point_lists[1].y <= state.position.y && state.position.y <= rectangle.point_lists[3].y;
}

bool ParticleSwarm::isInRader(utility::State& state) {
	for (int i = 0; i < radar_.size(); i++) {
		if (utility::dist(state.position, radar_[i]->center_point) < radar_[i]->R)
			return true;
	}
	return false;
}

void ParticleSwarm::updateSubMap(utility::UAV * uav) {
	//update dense map
	int i_min = max(0, (int)(uav->state.position.y - uav->search_r) / resolution);
	int i_max = min(size_y, (int)(uav->state.position.y + uav->search_r) / resolution);
	int j_min = max(0, (int)(uav->state.position.x - uav->search_r) / resolution);
	int j_max = min(size_x, (int)(uav->state.position.x + uav->search_r) / resolution);
	for (int i = i_min; i < i_max; i++) {
		for (int j = j_min; j < j_max; j++) {
			utility::point2D grid_pose((j + 0.5)*resolution, (i + 0.5)*resolution);
			if (dist(grid_pose, uav->state.position) < uav->search_r)
				global_map[i][j]->search_time[uav->id] = cunt;
		}
	}

	//update sparce map
	i_min = max(0, (int)(uav->state.position.y - uav->search_r) / sparse_resolution);
	i_max = min(sparse_size_y, (int)(uav->state.position.y + uav->search_r) / sparse_resolution);
	j_min = max(0, (int)(uav->state.position.x - uav->search_r) / sparse_resolution);
	j_max = min(sparse_size_x, (int)(uav->state.position.x + uav->search_r) / sparse_resolution);
	for (int i = i_min; i < i_max; i++) {
		for (int j = j_min; j < j_max; j++) {
			utility::point2D grid_pose((j + 0.5)*sparse_resolution, (i + 0.5)*sparse_resolution);
			if (dist(grid_pose, uav->state.position) < sparse_resolution)
				global_map[i][j]->search_time[uav->id] = cunt;
		}
	}
	return;
}

void ParticleSwarm::updateUAVStatesInDubinsState(utility::UAV* uav) {
	double minR = uav->state.velocity.x * uav->state.velocity.x / (uav->Acc_limite_y[1]);//最小转弯半径
	double l = dist(uav->traj_Point.position, uav->state.position);
	double theTa1 = acos((uav->traj_Point.position.x - uav->state.position.x) / l);
	theTa1 = (uav->traj_Point.position.y - uav->state.position.y)>0 ? theTa1 : 2 * pi - theTa1;
	double theTa2 = uav->state.velocity.y - theTa1;
	cout << "theTa2 =" << theTa2 * 180 / pi << endl;
	if (theTa2 > pi)
		theTa2 -= 2 * pi;
	else if (theTa2 <-pi)
		theTa2 += 2 * pi;
	double theTi, d;

	if (theTa2<-pi / 2) {
		theTa2 += pi;
		d = sqrt((l - minR * sin(theTa2))*(l - minR * sin(theTa2)) + minR * cos(theTa2)*minR*cos(theTa2));
		theTi = asin(minR*cos(theTa2) / d) + asin(minR / d) + theTa1;
	}
	else if (theTa2<0) {
		theTa2 *= -1;
		d = sqrt((l - minR * sin(theTa2))*(l - minR * sin(theTa2)) + minR * cos(theTa2)*minR*cos(theTa2));
		theTi = asin(minR / d) - asin(minR*cos(theTa2) / d) + theTa1;
	}
	else if (theTa2>pi / 2) {
		theTa2 = pi - theTa2;
		d = sqrt((l - minR * sin(theTa2))*(l - minR * sin(theTa2)) + minR * cos(theTa2)*minR*cos(theTa2));
		theTi = -asin(minR / d) - asin(minR*cos(theTa2) / d) + theTa1;
	}
	else {
		d = sqrt((l - minR * sin(theTa2))*(l - minR * sin(theTa2)) + minR * cos(theTa2)*minR*cos(theTa2));
		theTi = -asin(minR / d) + asin(minR*cos(theTa2) / d) + theTa1;
	}


	if (isnan(theTa1)) {
		cout << "theTa1 is nan" << endl;
		theTi = theTa2;
	}
	cout << "uav->state.velocity.y = " << uav->state.velocity.y << endl;
	cout << "theTi = " << theTi << endl;
	double q0[3] = { uav->state.position.x,uav->state.position.y,uav->state.velocity.y };//start point state
	double q1[3] = { uav->traj_Point.position.x,uav->traj_Point.position.y,theTi };//end point state;


	HybridAStar::dubins_init(q0, q1, minR, path);
	double velocity = uav->state.velocity.x;
	double t = dt * velocity, total_lenth = HybridAStar::dubins_path_length(path);

	while (!uav->path_.empty())
		uav->path_.pop();

	cout << "total_lenth" << total_lenth << endl;
	while (t <= total_lenth) {
		//cout << "t" << t <<endl;
		double temp_point[3];
		HybridAStar::dubins_path_sample(path, t, temp_point);
		utility::State temp;
		temp.position.x = temp_point[0];
		temp.position.y = temp_point[1];
		temp.velocity.x = velocity;
		temp.velocity.y = temp_point[2];
		uav->path_.push(temp);
		t += velocity * dt;
	}
	return;
}

void ParticleSwarm::updateArtificalPotentialFieldStateImpl(utility::UAV * uav, utility::State& goal, utility::State&next) {
	double k_att = 1,k_ref = 10000000.0;

	//compute Attraction
	utility::point2D F_att = computeAttraction(uav->state.position, goal.position);

	//compute Repulsion
	utility::point2D obstacle_point;
	if (uav->state.position.x > rectangle.point_lists[2].x)
		obstacle_point.x =rectangle.point_lists[2].x;
	else if (uav->state.position.x < rectangle.point_lists[0].x)
		obstacle_point.x = rectangle.point_lists[0].x;
	else
		obstacle_point.x = uav->state.position.x;
	if (uav->state.position.y > rectangle.point_lists[3].y)
		obstacle_point.y = rectangle.point_lists[3].y;
	else if (uav->state.position.y < rectangle.point_lists[1].y)
		obstacle_point.y = rectangle.point_lists[0].y;
	else
		obstacle_point.y = uav->state.position.y;
	utility::point2D F_rep = computeRepulsion(uav->state.position, obstacle_point);

	//compute Radar
	utility::point2D Radar_point;
	Radar_point = radar_[0]->center_point + (uav->state.position - radar_[0]->center_point)*(radar_[0]->R / (uav->state.position - radar_[0]->center_point).distance());
	F_rep = F_rep + computeRepulsion(uav->state.position, Radar_point);

	//sum
	utility::point2D F = F_att* k_att + F_rep* k_ref;
	double d_angle = F.angle() - uav->state.velocity.y;
	if (d_angle > pi)
		d_angle -= 2 * pi;
	else if (d_angle <-pi)
		d_angle += 2 * pi;
	utility::point2D F_body;
	double dur_x[2] = { max(uav->Vel_limite[0],uav->state.velocity.x + uav->Acc_limite_x[0] * dt) ,min(uav->Vel_limite[1],uav->state.velocity.x + uav->Acc_limite_x[1] * dt) };
	double dur_y[2] = { uav->Acc_limite_y[0]*dt, uav->Acc_limite_y[1] * dt };
	if (abs(d_angle) < 0.01) {//pingxing 
		F_body.x = (dur_x[1] - uav->state.velocity.x) / dt;
		F_body.y = 0;
	}
	else {
		double x = abs(dur_y[0] / tan(d_angle));
		if (abs(d_angle)>=atan(dur_y[1]/ dur_x[0])) {//不相交
			F_body.x = (dur_x[0] - uav->state.velocity.x) / dt;
			F_body.y = d_angle > 0 ? dur_y[1] : dur_y[0];
		}
		else if (x >= dur_x[1]) {//相交于竖线
			F_body.x = (dur_x[1] - uav->state.velocity.x) / dt;
			F_body.y = dur_x[1] * tan(d_angle);
		}
		else {//相交于横线
			F_body.x = (x - uav->state.velocity.x) / dt;
			F_body.y = d_angle > 0 ? dur_y[1] : dur_y[0];
		}
	}
	F.x = F_body.x * cos(uav->state.velocity.y) - F_body.y * sin(uav->state.velocity.y);
	F.y = F_body.x * sin(uav->state.velocity.y) + F_body.y * cos(uav->state.velocity.y);

	next.position.x = uav->state.position.x + uav->state.velocity.x* cos(uav->state.velocity.y)*dt + 0.5*F.x * dt*dt;
	next.position.y = uav->state.position.y + uav->state.velocity.x* sin(uav->state.velocity.y)*dt + 0.5*F.y * dt*dt;
	double tmep[2] = { uav->state.velocity.x* cos(uav->state.velocity.y) + dt * F.x ,uav->state.velocity.x* sin(uav->state.velocity.y) + dt * F.y };
	next.velocity.x = sqrt(tmep[0] * tmep[0] + tmep[1] * tmep[1]);
	next.velocity.y = tmep[1] > 0 ? acos(tmep[0] / next.velocity.x) : 2 * pi - acos(tmep[0] / next.velocity.x);
	return;
}

utility::point2D ParticleSwarm::computeRepulsion(utility::point2D& start, utility::point2D& obstacle) {
	utility::point2D F_rep = start- obstacle;

	if (F_rep.distance() > 10000)
		F_rep = F_rep * 0;
	else if (F_rep.distance() > 0.0000001) 
		F_rep = F_rep * (1 / pow(F_rep.distance(), 3));
		
	return F_rep;
};

utility::point2D ParticleSwarm::computeAttraction(utility::point2D& start, utility::point2D& goal) {
	utility::point2D F_att = goal - start;
	return F_att;
};

void ParticleSwarm::spreadParticles(utility::UAV * uav) {
	srand((unsigned)time(NULL));
	int j = 0;

	while (j<particle_num) {
		utility::particle* temp_particle = uav->swarm[j];
		double length = uav->search_r+resolution  + (uav->state.velocity.x*dt+ 1000) * (rand() % 1000) / 1000.0;
		double theta = 2 * pi*((rand() % 1000) / 1000.0);
		temp_particle->state.position.x = length * cos(theta) + uav->state.position.x;
		temp_particle->state.position.y = length * sin(theta) + uav->state.position.y;
		temp_particle->state.velocity.x = uav->Vel_limite[0] + (uav->Vel_limite[1] - uav->Vel_limite[0]) * ((rand() % 1000) / 1000.0);//线速度
		temp_particle->state.velocity.y = 2 * pi*((rand() % 1000) / 1000.0);//角度
		temp_particle->state.accelerate.x = 0;//线加速度
		temp_particle->state.accelerate.y = 0;//角速度
		if (isInObstacle(temp_particle->state)) //边界判断
			continue;
		if (!isInBound(temp_particle->state))
			continue;
		if (isInRader(temp_particle->state))
			continue;
		temp_particle->Pbest_state = temp_particle->state;//局部最优
		temp_particle->fitness = 0;
		j++;
	}
	uav->Gbest_state = uav->swarm.front()->Pbest_state;
	uav->Gbest_fitness = 0;
	return;
}

void ParticleSwarm::updateParticleStates(utility::UAV * uav) {//更新粒子
	uav->Gbest_fitness = 0;
	//w3 = 1/uav->calculateW3();
	int best_x = 0, best_y = 0;
	double best_grid_fit = 0, temp_grid_fit = 0;
	utility::point2D best_grid(0, 0);
	for (int i = 0; i<sparse_size_y; i++) {
		for (int j = 0; j<sparse_size_x; j++) {
			utility::State* temp = new utility::State();
			temp->position.x = (i + 0.5)*sparse_resolution;
			temp->position.y = (j + 0.5)*sparse_resolution;
			temp_grid_fit = (cunt-sparse_map[i][j]->search_time[uav->id]) + 1.0/((temp->position - uav->state.position).distance());
			if (temp_grid_fit>best_grid_fit) {
				best_grid_fit = temp_grid_fit;
				best_grid.x = (j + 0.5)*sparse_resolution;
				best_grid.y = (i + 0.5)*sparse_resolution;
			}
		}
	}
	ofstream particle_state, uav_state, best_state;
	particle_state.open(particle_state_path, ios::app | ios::binary);
	uav_state.open(uav_state_path, ios::app | ios::binary);
	best_state.open(best_state_path, ios::app | ios::binary);
	for (int i = 0; i<50; i++) {
		for (int j = 0; j < particle_num; j++) {
			utility::particle & temp_part = *(uav->swarm[j]);
			//速度更新
			double r1 = rand() % 1000 / 1000.0;
			double r2 = rand() % 1000 / 1000.0;
			double line_X = temp_part.state.velocity.x * cos(temp_part.state.velocity.y);//速度在XY向的分量
			double line_Y = temp_part.state.velocity.x * sin(temp_part.state.velocity.y);
			line_X = weight * line_X + c1 * r1 *(temp_part.Pbest_state.position.x - temp_part.state.position.x) + c2 * r2 *(uav->Gbest_state.position.x - temp_part.state.position.x);
			line_Y = weight * line_Y + c1 * r1 *(temp_part.Pbest_state.position.y - temp_part.state.position.y) + c2 * r2 *(uav->Gbest_state.position.y - temp_part.state.position.y);
			
			temp_part.state.velocity.x = sqrt(line_X * line_X + line_Y * line_Y);//粒子的速度更新公式
			if (line_Y >= 0)
				temp_part.state.velocity.y = acos(line_X / temp_part.state.velocity.x);
			else
				temp_part.state.velocity.y = 2 * pi - acos(line_X / temp_part.state.velocity.x);

			//速度限制
			if (temp_part.state.velocity.x > uav->Vel_limite[1])
				temp_part.state.velocity.x = uav->Vel_limite[1];
			else if (temp_part.state.velocity.x < uav->Vel_limite[0])
				temp_part.state.velocity.x = uav->Vel_limite[0];
			//粒子的位置更新公式
			temp_part.state.position.x += temp_part.state.velocity.x * dt*cos(temp_part.state.velocity.y);
			temp_part.state.position.y += temp_part.state.velocity.x * dt*sin(temp_part.state.velocity.y);
			//边界判断
			if (temp_part.state.position.x < PosMin.x) {
				temp_part.state.position.x = PosMin.x;
				temp_part.state.velocity.y = pi * (0.5 - (rand() % 1000) / 1000.0);//角度
			}
			else if (temp_part.state.position.x > PosMax.x) {
				temp_part.state.position.x = PosMax.x;
				temp_part.state.velocity.y = pi * (0.5 + (rand() % 1000) / 1000.0);//角度
			}
			if (temp_part.state.position.y < PosMin.y) {
				temp_part.state.position.y = PosMin.y;
				temp_part.state.velocity.y = pi * ((rand() % 1000) / 1000.0);//角度
			}
			else if (temp_part.state.position.y > PosMax.y) {
				temp_part.state.position.y = PosMax.y;
				temp_part.state.velocity.y = pi * (1 + (rand() % 1000) / 1000.0);//角度
			}

			//概率更新
			int x = temp_part.state.position.x / resolution;
			int y = temp_part.state.position.y / resolution;
			int t = cunt-global_map[y][x]->search_time[uav->id];
			//cout<<"size_x*size_y-uav->coverd_area_cnt="<<size_x*size_y-uav->coverd_area_cnt<<endl;
			//cout<<"1 - exp(-tao *t )="<<1 - exp(-tao *t )<<endl;
			double  num = (size_x*size_y - uav->coverd_area_cnt) / (double)(size_x * size_y);
			for (int k = 0; k < target_num; k++) {
				temp_part.p[k] = sigmod(t) / num;
				//cout << "num=" << num << "; temp_part.p[k] = " << temp_part.p[k] <<"  "<< 1 - exp(-tao * t) <<" ; t="<<t<< endl;
			}

			//适应值更新
			//fitness1
			double temp_fitness_1 = 0, temp_fitness_2 = 0, temp_fitness_3 = 0;
			utility::point2D obstacle;
			if (temp_part.state.position.x > rectangle.point_lists[2].x)
				obstacle.x = rectangle.point_lists[2].x;
			else if (temp_part.state.position.x < rectangle.point_lists[0].x)
				obstacle.x = rectangle.point_lists[0].x;
			else
				obstacle.x = temp_part.state.position.x;

			if (temp_part.state.position.y > rectangle.point_lists[3].y)
				obstacle.y = rectangle.point_lists[3].y;
			else if (temp_part.state.position.y < rectangle.point_lists[1].y)
				obstacle.y = rectangle.point_lists[1].y;
			else
				obstacle.y = temp_part.state.position.y;
			utility::point2D F_rep = computeRepulsion(temp_part.state.position, obstacle);
			//compute Radar
			utility::point2D Radar_point;
			Radar_point = radar_[0]->center_point + (uav->state.position - radar_[0]->center_point)*(radar_[0]->R / (uav->state.position - radar_[0]->center_point).distance());
			F_rep = F_rep + computeRepulsion(uav->state.position, Radar_point);
			for (int uav_it = 0; uav_it < uav_num; uav_it++) {
				if (uav_it != uav->id) {
					utility::point2D temp = computeRepulsion(temp_part.state.position, uav_[uav_it]->traj_Point.position);
					F_rep = F_rep + temp*0.01;
				}
			}
			for (int k = 0; k < target_num; k++) {
				double min_R = uav->state.velocity.x*uav->state.velocity.x / uav->Acc_limite_y[1];
				double Dik = dubinsDistance(uav->state, temp_part.state, min_R) / uav->search_r + F_rep.distance()*10000;//无人机k与粒子i的距离
				double Pij = temp_part.p[k];
				double dik = (1 - sigmod(Dik));
				int Tj = uav->Tj[k];//目标k是否未被与之更匹配的无人机跟踪
									//temp_fitness_1 += Tj * (Pij  + Dik +Cjk);
				//cout << "; dist="<< dubinsDistance(uav->state, temp_part.state, min_R) / uav->search_r <<"; F_rep="<< F_rep.distance() * 10000 << endl;
				temp_fitness_1 += (Pij * dik)*  Tj;
				//cout<<"1"<<endl;
			}
			//fitness2
			if (t >= forget_time)
				temp_fitness_2 = w * 1.0 / num;
			else
				temp_fitness_2 = (1 - w)*t / (forget_time * num);
			//cout << "temp_fitness_1 = " << temp_fitness_1 <<";temp_fitness_2 = "<<temp_fitness_2<<endl;

			//fitness3
			double angle1 = acos((best_grid.x - uav->state.position.x) / dist(uav->state.position, best_grid));
			angle1 = (best_grid.y - uav->state.position.y)>0 ? angle1 : 2 * pi - angle1;//0~2*pi
			double angle2 = acos((temp_part.state.position.x - uav->state.position.x) / dist(uav->state.position, temp_part.state.position));
			angle2 = (temp_part.state.position.y - uav->state.position.y)>0 ? angle2 : 2 * pi - angle2;//0~2*pi
			double angle = abs(angle2 - angle1) < pi ? abs(angle2 - angle1) : 2 * pi - abs(angle2 - angle1);
			temp_fitness_3 = 1 - sigmod(angle);
			//cout << "temp_fitness_1 = " << temp_fitness_1 <<";temp_fitness_2 = "<<temp_fitness_2<< ";temp_fitness_3 = " << temp_fitness_3 << endl;
			double temp_fitness = w1 * temp_fitness_1 + w2 * temp_fitness_2 + w3 * temp_fitness_3;//适应值
			if (isInRader(temp_part.state))
				temp_fitness = -1;
			if(isInObstacle(temp_part.state))
				temp_fitness = -1;
			if (temp_fitness >= temp_part.fitness) {//局部最优位置判断
				temp_part.fitness = temp_fitness;
				temp_part.Pbest_state = temp_part.state;
			}

			if (temp_fitness >= uav->Gbest_fitness) {//全局最优位置判断
													 //cout << "更新temp_fitness" << temp_fitness << " current particle" << j << endl;
				uav->Gbest_fitness = temp_fitness;
				uav->Gbest_state = temp_part.state;
			}
			
			if (uav->id == 3)
				particle_state << temp_part.state.position.x << " " << temp_part.state.position.y << " ";
		}
		if (uav->id == 3) {
			particle_state << endl;
			uav_state << uav->state.position.x << " " << uav->state.position.y << endl;
			best_state << uav->Gbest_state.position.x << " " << uav->Gbest_state.position.y << endl;
		}
	}
	//cout << "Gbest_fitness = " << uav->Gbest_fitness << endl;
	uav->traj_Point = uav->Gbest_state;
	uav->updatePrevPoses();
	return;
}

void ParticleSwarm::init() {
	//obstacle
	srand((int(time(NULL))));
	for (int i = 0; i < radar_num; i++) {
		utility::radar* temp_rader = new utility::radar();
		temp_rader->center_point.x = 30000;
		temp_rader->center_point.y = 30000;
		temp_rader->R = 5000;
		radar_.push_back(temp_rader);
	}
	{
		utility::point2D a, b, c, d;
		a.x = 15000;
		a.y = 15000;
		rectangle.point_lists.push_back(a);
		b.x = a.x;
		b.y = a.y - 5000;
		rectangle.point_lists.push_back(b);
		c.x = a.x + 5000;
		c.y = b.y;
		rectangle.point_lists.push_back(c);
		d.x = c.x;
		d.y = a.y;
		rectangle.point_lists.push_back(d);
	}

	//slow target
	int i = 0;
	for (; i<slow_target_num; ) {
		utility::TARGET* temp_target = new utility::TARGET();
		temp_target->Vel_limite[0] = 0;
		temp_target->Vel_limite[1] = 10;
		temp_target->Acc_limite_x[0] = -0.4;
		temp_target->Acc_limite_x[1] = 0.4;
		temp_target->Acc_limite_y[0] = -g * tan( 20.0 * pi / 180.0 );
		temp_target->Acc_limite_y[1] = g * tan(20.0 * pi / 180.0 );
		temp_target->state.position.x = PosMin.x + (PosMax.x - PosMin.x)*(rand() % 1000 / 1000.0);
		temp_target->state.position.y = PosMin.y + (PosMax.y - PosMin.y)*(rand() % 1000 / 1000.0);
		temp_target->state.velocity.x = 8;//线速度
		temp_target->state.velocity.y = 2 * pi*(rand() % 1000 / 1000.0);//角度
		temp_target->state.accelerate.x = 0;//线加速度
		temp_target->state.accelerate.y = 0;//角速度
		if (isInObstacle(temp_target->state))
			continue;
		target.push_back(temp_target);
		i++;
	}
	for (; i<slow_target_num+middel_target_num; ) {
		utility::TARGET* temp_target = new utility::TARGET();
		temp_target->Vel_limite[0] = 0;
		temp_target->Vel_limite[1] = 20;
		temp_target->Acc_limite_x[0] = -0.4;
		temp_target->Acc_limite_x[1] = 0.4;
		temp_target->Acc_limite_y[0] = -g * tan(20.0 * pi / 180.0);
		temp_target->Acc_limite_y[1] = g * tan(20.0 * pi / 180.0);
		temp_target->state.position.x = PosMin.x + (PosMax.x - PosMin.x)*(rand() % 1000 / 1000.0);
		temp_target->state.position.y = PosMin.y + (PosMax.y - PosMin.y)*(rand() % 1000 / 1000.0);;
		temp_target->state.velocity.x = 18;//线速度
		temp_target->state.velocity.y = 2 * pi*(rand() % 1000 / 1000.0);//角度
		temp_target->state.accelerate.x = 0;//线加速度
		temp_target->state.accelerate.y = 0;//角速度
		if (isInObstacle(temp_target->state))
			continue;
		target.push_back(temp_target);
		i++;
	}
	for (; i<slow_target_num + middel_target_num +fast_target_num; i) {
		utility::TARGET* temp_target = new utility::TARGET();
		temp_target->Vel_limite[0] = 0;
		temp_target->Vel_limite[1] = 30;
		temp_target->Acc_limite_x[0] = -0.4;
		temp_target->Acc_limite_x[1] = 0.4;
		temp_target->Acc_limite_y[0] = -g * tan(20.0 * pi / 180.0);
		temp_target->Acc_limite_y[1] = g * tan(20.0 * pi / 180.0);
		temp_target->state.position.x = PosMin.x + (PosMax.x - PosMin.x)*(rand() % 1000 / 1000.0);
		temp_target->state.position.y = PosMin.y + (PosMax.y - PosMin.y)*(rand() % 1000 / 1000.0);;
		temp_target->state.velocity.x = 28;//线速度
		temp_target->state.velocity.y = 2 * pi*(rand() % 1000 / 1000.0);//角度
		temp_target->state.accelerate.x = 0;//线加速度
		temp_target->state.accelerate.y = 0;//角速度
		if (isInObstacle(temp_target->state))
			continue;
		target.push_back(temp_target);
		i++;
	}


	//map
	for (int i = 0; i < size_y; i++) {//初始化地图，i=0代表第一行，j=0代表第一列
		vector<utility::grid*> temp;
		for (int j = 0; j < size_x; j++) {
			utility::grid* tempGridPtr = new utility::grid();
			for (int k = 0; k<uav_num; k++)
				tempGridPtr->search_time.push_back(-forget_time);
			temp.push_back(tempGridPtr);
		}
		global_map.push_back(temp);
	}
	//sparse_map
	for (int i = 0; i < sparse_size_y; i++) {//初始化地图，i=0代表第一行，j=0代表第一列
		vector<utility::grid*> temp;
		for (int j = 0; j < sparse_size_x; j++) {
			utility::grid* tempGridPtr = new utility::grid();
			for (int k = 0; k<uav_num; k++)
				tempGridPtr->search_time.push_back(-forget_time);
			temp.push_back(tempGridPtr);
		}
		sparse_map.push_back(temp);
	}


	//UAV
	i = 0;
	for (; i < slow_uav_num; ) {//初始化无人机
		utility::UAV* uav_temp = new utility::UAV();
		uav_temp->id = i;
		uav_temp->Vel_limite[0]=0;
		uav_temp->Vel_limite[1] = 25;
		uav_temp->Acc_limite_x[0] = -0.4;
		uav_temp->Acc_limite_x[1] = 0.4;
		uav_temp->Acc_limite_y[0] = -g*tan(20.0 * pi / 180.0);
		uav_temp->Acc_limite_y[1] = g * tan(20.0 * pi / 180.0);
		uav_temp->h = 400;
		uav_temp->search_r = 2000;
		uav_temp->track_target_num = -1;
		uav_temp->coverd_area_cnt = 1;

		uav_temp->state.position.x = PosMin.x + (PosMax.x - PosMin.x)*(rand() % 1000 / 1000.0);//位置
		uav_temp->state.position.y = PosMin.y + (PosMax.y - PosMin.y)*(rand() % 1000 / 1000.0);
		uav_temp->state.velocity.x = uav_temp->Vel_limite[0] + (uav_temp->Vel_limite[1] - uav_temp->Vel_limite[0]) * (rand() % 1000 / 1000.0);//线速度
		uav_temp->state.velocity.y = 2 * pi*(rand() % 1000 / 1000.0);//角度
		uav_temp->state.accelerate.x = 0;//线加速度
		uav_temp->state.accelerate.y = 0;//角速度
		
		if (isInObstacle(uav_temp->state))
			continue;

		for (int j = 0; j < target_num; j++) {
			utility::State temp_state;
			uav_temp->target_state.push_back(make_pair(temp_state, -forget_time));
			uav_temp->Tj[j] = 1;
		}
		updateSubMap(uav_temp);
		for (int j = 0; j < particle_num; j++) {
			utility::particle* temp = new utility::particle();
			uav_temp->swarm.push_back(temp);
		}
		uav_.push_back(uav_temp);
		union_.parent.push_back(i);
		i++;
	}
	for (; i < slow_uav_num + middel_uav_num; ) {//初始化无人机
		utility::UAV* uav_temp = new utility::UAV();
		uav_temp->id = i;
		uav_temp->Vel_limite[0] = 40;
		uav_temp->Vel_limite[1] = 50;
		uav_temp->Acc_limite_x[0] = -0.5;
		uav_temp->Acc_limite_x[1] = 0.5;
		uav_temp->Acc_limite_y[0] = -g * tan(25.0 * pi / 180.0);
		uav_temp->Acc_limite_y[1] = g * tan(20.0 * pi / 180.0);
		uav_temp->h = 600;
		uav_temp->search_r = 3000;
		uav_temp->track_target_num = -1;
		uav_temp->coverd_area_cnt = 1;

		uav_temp->state.position.x = PosMin.x + (PosMax.x - PosMin.x)*(rand()%1000 / 1000.0);//位置
		uav_temp->state.position.y = PosMin.y + (PosMax.y - PosMin.y)*(rand() % 1000 / 1000.0);;
		uav_temp->state.velocity.x = uav_temp->Vel_limite[0] + (uav_temp->Vel_limite[1] - uav_temp->Vel_limite[0]) * (rand() % 1000 / 1000.0);//线速度
		uav_temp->state.velocity.y = 2 * pi*(rand() % 1000 / 1000.0);//角度
		uav_temp->state.accelerate.x = 0;//线加速度
		uav_temp->state.accelerate.y = 0;//角速度
		if (isInObstacle(uav_temp->state))
			continue;

		for (int j = 0; j < target_num; j++) {
			utility::State temp_state;
			uav_temp->target_state.push_back(make_pair(temp_state, -forget_time));
			uav_temp->Tj[j] = 1;
		}
		updateSubMap(uav_temp);
		for (int j = 0; j < particle_num; j++) {
			utility::particle* temp = new utility::particle();
			uav_temp->swarm.push_back(temp);
		}
		uav_.push_back(uav_temp);
		union_.parent.push_back(i);
		i++;
	}
	for (; i < slow_uav_num+ middel_uav_num+ fast_uav_num; ) {//初始化无人机
		utility::UAV* uav_temp = new utility::UAV();
		uav_temp->id = i;
		uav_temp->Vel_limite[0] = 60;
		uav_temp->Vel_limite[1] = 90;
		uav_temp->Acc_limite_x[0] = -0.6;
		uav_temp->Acc_limite_x[1] = 0.6;
		uav_temp->Acc_limite_y[0] = -g * tan(30.0 * pi / 180.0);
		uav_temp->Acc_limite_y[1] = g * tan(30.0 * pi / 180.0);
		uav_temp->h = 1000;
		uav_temp->search_r = 5000;
		uav_temp->track_target_num = -1;
		uav_temp->coverd_area_cnt = 1;

		uav_temp->state.position.x = PosMin.x + (PosMax.x - PosMin.x)*(rand() % 1000 / 1000.0);//位置
		uav_temp->state.position.y = PosMin.y + (PosMax.y - PosMin.y)*(rand() % 1000 / 1000.0);
		uav_temp->state.velocity.x = uav_temp->Vel_limite[0] + (uav_temp->Vel_limite[1] - uav_temp->Vel_limite[0]) * (rand() % 1000 / 1000.0);//线速度
		uav_temp->state.velocity.y = 2 * pi*(rand() % 1000 / 1000.0);//角度
		uav_temp->state.accelerate.x = 0;//线加速度
		uav_temp->state.accelerate.y = 0;//角速度

		if (isInObstacle(uav_temp->state))
			continue;

		for (int j = 0; j < target_num; j++) {
			utility::State temp_state;
			uav_temp->target_state.push_back(make_pair(temp_state, -forget_time));
			uav_temp->Tj[j] = 1;
		}
		updateSubMap(uav_temp);
		for (int j = 0; j < particle_num; j++) {
			utility::particle* temp = new utility::particle();
			uav_temp->swarm.push_back(temp);
		}
		uav_.push_back(uav_temp);
		union_.parent.push_back(i);
		i++;
	}
};

void ParticleSwarm::updateTargetStates() {
	//目标状态更新，这里是等速运动
	output_target.open(target_path.c_str(), ios::app | ios::binary);
	for (int target_tag = 0; target_tag < target_num; target_tag++) {
		double  Acc_X = target[target_tag]->Acc_limite_x[0] + (target[target_tag]->Acc_limite_x[1] - target[target_tag]->Acc_limite_x[0]) *(rand()%1000 / 1000.0);
		double  Acc_Y = target[target_tag]->Acc_limite_y[0] + (target[target_tag]->Acc_limite_y[1] - target[target_tag]->Acc_limite_y[0]) *(rand() % 1000 / 1000.0);
		target[target_tag]->state.position.x += target[target_tag]->state.velocity.x * cos(target[target_tag]->state.velocity.y)* dt + 0.5 * Acc_X* dt * dt;//位置更新
		target[target_tag]->state.position.y += target[target_tag]->state.velocity.x * sin(target[target_tag]->state.velocity.y)* dt + 0.5 * Acc_Y* dt * dt;//位置更新

		double line_X = target[target_tag]->state.velocity.x * cos(target[target_tag]->state.velocity.y) + Acc_X * dt;//速度在XY的分量
		double line_Y = target[target_tag]->state.velocity.x * sin(target[target_tag]->state.velocity.y) + Acc_Y * dt;

		target[target_tag]->state.velocity.x = sqrt(line_X * line_X + line_Y * line_Y);
		target[target_tag]->state.velocity.y = line_Y > 0 ? acos(line_X / target[target_tag]->state.velocity.x): 2 * pi - acos(line_X / target[target_tag]->state.velocity.x);
		if (target[target_tag]->state.velocity.x > target[target_tag]->Vel_limite[1]) {
			target[target_tag]->state.velocity.x = target[target_tag]->Vel_limite[1];
		}else if(target[target_tag]->state.velocity.x < target[target_tag]->Vel_limite[0])
			target[target_tag]->state.velocity.x = target[target_tag]->Vel_limite[0];
		//边界判断
		if (target[target_tag]->state.position.x < PosMin.x) {
			target[target_tag]->state.position.x = PosMin.x;
			target[target_tag]->state.velocity.y = pi * (0.5 - (rand() % 1000) / 1000.0);//角度
		}
		else if (target[target_tag]->state.position.x > PosMax.x) {
			target[target_tag]->state.position.x = PosMax.x;
			target[target_tag]->state.velocity.y = pi * (0.5 + (rand() % 1000) / 1000.0);//角度
		}

		if (target[target_tag]->state.position.y < PosMin.y) {
			target[target_tag]->state.position.y = PosMin.y;
			target[target_tag]->state.velocity.y = pi * ((rand() % 1000) / 1000.0);//角度
		}
		else if (target[target_tag]->state.position.y > PosMax.y) {
			target[target_tag]->state.position.y = PosMax.y;
			target[target_tag]->state.velocity.y = pi * (1 + (rand() % 1000) / 1000.0);//角度
		}
		output_target << target[target_tag]->state.position.x << " " << target[target_tag]->state.position.y << " ";
	}
	output_target << endl;
	output_target.close();
	return;
};

void ParticleSwarm::updateUAVStates() {
	output_uav.open(uav_path.c_str(), ios::app | ios::binary);
	output_traj_Point.open(traj_Point_path.c_str(), ios::app | ios::binary);
	ofstream time_out_pso, time_out_submap;
	time_out_pso.open("data/time_pso.txt", ios::app | ios::binary);
	time_out_submap.open("data/time_submap.txt", ios::app | ios::binary);
	for (int i = 0; i<uav_num; i++) {
		//cout << "updating uav" << i << endl;
		clock_t start, finish;
		double  duration;
		
		if (uav_[i]->track_target_num == -1) {
			start = clock();
			spreadParticles(uav_[i]);
			updateParticleStates(uav_[i]);
			finish = clock();
			duration = (finish - start) ;
			time_out_pso << duration << " ";
			//cout << "time for PSO is " << duration << endl;
		}
		
		updateArtificalPotentialFieldStateImpl(uav_[i], uav_[i]->traj_Point, uav_[i]->state);
		
		//cout << "time for APF is " << duration << endl;
		output_uav << uav_[i]->state.position.x << " " << uav_[i]->state.position.y << " " << uav_[i]->track_target_num << " ";
		
		output_traj_Point << uav_[i]->traj_Point.position.x << " " << uav_[i]->traj_Point.position.y << " ";

		start = clock();
		updateSubMap(uav_[i]);
		finish = clock();
		duration = (double)(finish - start) ;
		time_out_submap << duration << " ";

		for (int j = 0; j<target_num; j++) {//target inf0
			if (dist(target[j]->state.position, uav_[i]->state.position)<uav_[i]->search_r) {
				uav_[i]->target_state[j].first = target[j]->state;
				uav_[i]->target_state[j].second = cunt;
			}
		}
	}
	time_out_pso << endl;
	time_out_submap << endl;
	time_out_pso.close();
	time_out_submap.close();
	output_uav << endl;
	output_uav.close();
	output_traj_Point << endl;
	output_traj_Point.close();
	return;
}

void ParticleSwarm::informationShare() {
	for (int i = 0; i < uav_num; i++) {//reset
		union_.parent[i] = i;
		uav_[i]->coverd_area_cnt = 0;
	}
	for (int i = 0; i < uav_num; i++) {
		for (int j = i; j < uav_num; j++) {
			if (dist(uav_[i]->state.position, uav_[j]->state.position) <= communication_R) {//union
				union_.join(i, j);
				for (int k = 0; k<target_num; k++) {//exchange target info
					if (uav_[i]->target_state[k].second<uav_[j]->target_state[k].second)
						uav_[i]->target_state[k] = uav_[j]->target_state[k];
					else
						uav_[j]->target_state[k] = uav_[i]->target_state[k];
					int x = uav_[i]->target_state[k].first.position.x / resolution;
					int y = uav_[i]->target_state[k].first.position.y / resolution;
					if (global_map[y][x]->search_time[i] > uav_[j]->target_state[k].second + 1) {//如果此处没有目标
						uav_[i]->target_state[k].second = -forget_time;
						uav_[j]->target_state[k].second = -forget_time;
					}
				}
			}
		}
	}

	union_.setup_barrel();//setup barrel

	//dense map
	for (int i = 0; i<size_y; i++) {
		for (int j = 0; j <size_x; j++) {
			for (int k = 0; k<union_.barrel.size(); k++) {
				float max_count = -forget_time;
				for (int l = 0; l<union_.barrel[k].size(); l++) {
					max_count = std::max(max_count, global_map[i][j]->search_time[union_.barrel[k][l]]);//find the lowest search time
				}
				for (int l = 0; l<union_.barrel[k].size(); l++) {
					global_map[i][j]->search_time[union_.barrel[k][l]] = max_count;//set the search time
					if (cunt-max_count<forget_time)
						uav_[union_.barrel[k][l]]->coverd_area_cnt++;//count the coverd area
				}
			}
		}
	}

	//sparse map
	for (int i = 0; i<sparse_size_y; i++) {
		for (int j = 0; j <sparse_size_x; j++) {
			for (int k = 0; k<union_.barrel.size(); k++) {
				float max_count = -forget_time;
				for (int l = 0; l<union_.barrel[k].size(); l++) {
					max_count = std::max(max_count, sparse_map[i][j]->search_time[union_.barrel[k][l]]);//find the lowest search time
				}
				for (int l = 0; l<union_.barrel[k].size(); l++) {
					sparse_map[i][j]->search_time[union_.barrel[k][l]] = max_count;//set the search time
				}
			}
		}
	}

	return;
}

void ParticleSwarm::updateMission() {
	for (int i = 0; i < uav_num; i++) {
		for (int j = 0; j < target_num; j++) {//如果无人机i发现的目标j所在的栅格的搜索次数大于1000次
			if (cunt-uav_[i]->target_state[j].second >= forget_time)
				target_state[i][j] = 0;//不让无人机i去追目标j
			else
				target_state[i][j] = 10000.0/(uav_[i]->state.position - target[j]->state.position).distance() / uav_[i]->search_r + 100.0/((uav_[i]->Vel_limite[1] - target[j]->Vel_limite[1]));
			//cout << "target_state[" << i << "][" << j << "]=" << target_state[i][j] << endl;
		}
	}
	set<int> row, col;
	for (int l = 0; l < target_num; l++) {
		double maxTargetState = 0;
		int uav_cnt, target_cnt;
		for (int i = 0; i < uav_num; i++) {//寻找满足条件的最大值，并行与列
			for (int j = 0; j < target_num; j++) {
				if (row.find(i) == row.end() && col.find(j) == col.end() && target_state[i][j] > maxTargetState) {
					maxTargetState = target_state[i][j];
					uav_cnt = i;
					target_cnt = j;
				}
			}
		}
		if (maxTargetState > 0.001) {//当能够找到有效的任务分配时，执行任务分配
			cout << "uav_cnt = " << uav_cnt << " target_cnt = " << target_cnt << endl;
			if (dist(uav_[uav_cnt]->state.position, target[target_cnt]->state.position) < 500) {
				tracked[target_cnt] = true;
				uav_[uav_cnt]->state.velocity.x = target[target_cnt]->state.velocity.x;
				cout << "target " << target_cnt << " is successfully tracked by uav " << uav_cnt << endl;
			}

			for (int i = 0; i < uav_num; i++) {
				if (i == uav_cnt) {
					uav_[i]->Tj[target_cnt] = 1;//分配该无人机追踪该目标
				}
				else
					uav_[i]->Tj[target_cnt] = 0;//否则分配该无人机不追踪该目标
			}
			row.insert(uav_cnt); col.insert(target_cnt);//将行与列放入set中，下次循环跳过该行与列
		}
	}
	for (int i = 0; i < uav_num; i++) {
		int j = 0;
		for (; j < target_num; j++) {
			//cout << "uav[" << i << "].TJ[" << j << "]= " << uav[i].Tj[j] << " "<< uav[i].covered_target_id[j].second<< endl;
			if (uav_[i]->Tj[j] == 1 && cunt-uav_[i]->target_state[j].second < forget_time) {//分配该无人机追踪该目标
				uav_[i]->track_target_num = j;
				uav_[i]->traj_Point = uav_[i]->target_state[j].first;
				break;
			}
		}
		if (j >= target_num) {
			uav_[i]->track_target_num = -1;
			cout << "uav_cnt = " << i << " is free" << endl;  
		}//该无人机尚未分配任何目标
	}
}

void ParticleSwarm::run() {
	init();
	while (cunt < 10000) 
	{
		//更新目标位置
		updateTargetStates();

		//更新无人机状态
		updateUAVStates();

		//机间通信
		if (uav_num>1)
			informationShare();

		//任务分配
		updateMission();

		cout << "cunt" << cunt << endl;

		//判断终止
		int i = 0;
		for (; i<target_num; i++) {
			if (!tracked[i])
				break;
		}
		if (i >= target_num)
			break;

		cunt++;
	}
	//std::cout << "target_state:" << endl << target_state << endl;
	std::cout << "All target has been tracked!The total number of step is " << cunt << std::endl;
}