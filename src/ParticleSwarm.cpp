#include"stdafx.h"
#include "ParticleSwarm.h"

ParticleSwarm::ParticleSwarm()
{
	target_state = *(new vector<vector<double>>(UAV_num, vector<double>(target_num, 0)));
	tracked = *(new vector<bool>(target_num, false));
	path = new HybridAStar::DubinsPath();
}


ParticleSwarm::~ParticleSwarm()
{
}

double ParticleSwarm::sigmod(double x) {
	return 1.0 / (1 + exp(-x));
}

double ParticleSwarm::dubinsDistance(utility::State& state1, utility::State& state2) {
	double minR = state1.velocity.x * state1.velocity.x / (g * tanTheta);//��Сת��뾶
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

	if (isnan(theTi))
		return 1000000;

	double q1[3] = { state1.position.x,state1.position.y,state1.velocity.y };//start point state
	double q2[3] = { state2.position.x,state2.position.y,theTi };//end point state;
	HybridAStar::dubins_init(q1, q2, minR, path);
	return HybridAStar::dubins_path_length(path);
}

bool ParticleSwarm::isInBound(utility::State& state) {
	if (state.position.x <= PosMin.x || state.position.x >= PosMax.x ||
		state.position.y <= PosMin.y || state.position.y >= PosMax.y)
		return false;
	return true;
}

bool ParticleSwarm::isInRader(utility::State& state) {
	for (int i = 0; i < radar_.size(); i++) {
		if (utility::dist(state.position, radar_[i]->center_point) < radar_[i]->R)
			return true;
	}
	return false;
}

void ParticleSwarm::updateSubMap(utility::UAV * uav) {

	for (int i = 0; i < size_y; i++) {
		for (int j = 0; j < size_x; j++) {
			utility::point2D grid_pose((j + 0.5)*resolution, (i + 0.5)*resolution);
			global_map[i][j]->search_count[uav->id]++;
			if (dist(grid_pose, uav->state.position) < search_R)
				global_map[i][j]->search_count[uav->id] = 0;
			if (global_map[i][j]->search_count[uav->id] >= forget_time)
				global_map[i][j]->search_count[uav->id] = forget_time;
		}
	}


	return;
}

void ParticleSwarm::updateUAVStatesInDubinsState(utility::UAV* uav) {
	double minR = uav->state.velocity.x * uav->state.velocity.x / (g * tanTheta);//��Сת��뾶
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
		vector<double> temp;
		temp.push_back(temp_point[0]);
		temp.push_back(temp_point[1]);
		temp.push_back(velocity);
		temp.push_back(temp_point[2]);
		uav->path_.push(temp);

		if (t>path->param[0] && t<path->param[1] && velocity + AccMax * dt<VelMax.x) {
			t += velocity * dt + 0.5*AccMax*dt*dt;
			velocity = min(VelMax.x, velocity + AccMax * dt);
		}
		else {
			t += velocity * dt;
		}
	}

	return;
}

void ParticleSwarm::spreadParticles(utility::UAV * uav) {
	srand((unsigned)time(NULL));
	int j = 0;
	while (j<particle_num) {
		utility::particle* temp_particle = uav->swarm[j];
		double length = search_R + 1000 + particle_R * (rand() % 1000) / 1000.0;
		double theta = 2 * pi*((rand() % 1000) / 1000.0);
		temp_particle->state.position.x = length * cos(theta) + uav->state.position.x;
		temp_particle->state.position.y = length * sin(theta) + uav->state.position.y;
		temp_particle->state.velocity.x = VelMin.x + (VelMax.x - VelMin.x) * ((rand() % 1000) / 1000.0);//���ٶ�
		temp_particle->state.velocity.y = 2 * pi*((rand() % 1000) / 1000.0);//�Ƕ�
		temp_particle->state.accelerate.x = 0;//�߼��ٶ�
		temp_particle->state.accelerate.y = 0;//���ٶ�
											  //�߽��ж�
		if (!isInBound(temp_particle->state))
			continue;
		if (isInRader(temp_particle->state))
			continue;

		temp_particle->Pbest_state = temp_particle->state;//�ֲ�����
		temp_particle->fitness = 0;
		j++;
	}
	uav->Gbest_state = uav->swarm.front()->Pbest_state;
	uav->Gbest_fitness = 0;
	return;
}

void ParticleSwarm::updateParticleStates(utility::UAV * uav) {//��������
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
			temp_grid_fit = sparse_map[i][j]->search_count[uav->id] - sigmod(dubinsDistance(*temp, uav->state));
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
	for (int i = 0; i<100; i++) {
		uav->last_Gbest_state = uav->Gbest_state;
		for (int j = 0; j < particle_num; j++) {
			utility::particle & temp_part = *(uav->swarm[j]);
			//�ٶȸ���
			double r1 = (double)rand() / RAND_MAX;
			double r2 = (double)rand() / RAND_MAX;
			double line_X = temp_part.state.velocity.x * cos(temp_part.state.velocity.y);//�ٶ���XY��ķ���
			double line_Y = temp_part.state.velocity.x * sin(temp_part.state.velocity.y);
			line_X = weight * line_X + c1 * r1 *(temp_part.Pbest_state.position.x - temp_part.state.position.x) + c2 * r2 *(uav->Gbest_state.position.x - temp_part.state.position.x);
			line_Y = weight * line_Y + c1 * r1 *(temp_part.Pbest_state.position.y - temp_part.state.position.y) + c2 * r2 *(uav->Gbest_state.position.y - temp_part.state.position.y);
			temp_part.state.velocity.x = sqrt(line_X * line_X + line_Y * line_Y);//���ӵ��ٶȸ��¹�ʽ
			if (line_Y >= 0)
				temp_part.state.velocity.y = acos(line_X / temp_part.state.velocity.x);
			else
				temp_part.state.velocity.y = 2 * pi - acos(line_X / temp_part.state.velocity.x);

			//�ٶ�����
			if (temp_part.state.velocity.x > VelMax.x)
				temp_part.state.velocity.x = VelMax.x;
			else if (temp_part.state.velocity.x < VelMin.x)
				temp_part.state.velocity.x = VelMin.x;
			//���ӵ�λ�ø��¹�ʽ
			temp_part.state.position.x += temp_part.state.velocity.x * dt*cos(temp_part.state.velocity.y);
			temp_part.state.position.y += temp_part.state.velocity.x * dt*sin(temp_part.state.velocity.y);
			//�߽��ж�
			if (temp_part.state.position.x < PosMin.x) {
				temp_part.state.position.x = PosMin.x;
				temp_part.state.velocity.y = pi * (0.5 - (rand() % 1000) / 1000.0);//�Ƕ�
			}
			else if (temp_part.state.position.x > PosMax.x) {
				temp_part.state.position.x = PosMax.x;
				temp_part.state.velocity.y = pi * (0.5 + (rand() % 1000) / 1000.0);//�Ƕ�
			}
			if (temp_part.state.position.y < PosMin.y) {
				temp_part.state.position.y = PosMin.y;
				temp_part.state.velocity.y = pi * ((rand() % 1000) / 1000.0);//�Ƕ�
			}
			else if (temp_part.state.position.y > PosMax.y) {
				temp_part.state.position.y = PosMax.y;
				temp_part.state.velocity.y = pi * (1 + (rand() % 1000) / 1000.0);//�Ƕ�
			}

			//���ʸ���
			int x = temp_part.state.position.x / resolution;
			int y = temp_part.state.position.y / resolution;
			int t = global_map[y][x]->search_count[uav->id];
			//cout<<"size_x*size_y-uav->coverd_area_cnt="<<size_x*size_y-uav->coverd_area_cnt<<endl;
			//cout<<"1 - exp(-tao *t )="<<1 - exp(-tao *t )<<endl;
			long double  num = (size_x*size_y - uav->coverd_area_cnt) / 1000000.0;
			for (int k = 0; k < target_num; k++) {
				temp_part.p[k] = sigmod(t) / num;
				//cout << "num=" << num << "; temp_part.p[k] = " << temp_part.p[k] <<"  "<< 1 - exp(-tao * t) <<" ; t="<<t<< endl;
			}

			//��Ӧֵ����
			//fitness1
			double temp_fitness_1 = 0, temp_fitness_2 = 0, temp_fitness_3 = 0;
			for (int k = 0; k < target_num; k++) {

				double Dik = dubinsDistance(uav->state, temp_part.state) / search_R;//���˻�k������i�ľ���
				double Pij = temp_part.p[k];
				double dik = 100 * (1 - sigmod(Dik));
				int Tj = uav->Tj[k];//Ŀ��k�Ƿ�δ����֮��ƥ������˻�����
									//temp_fitness_1 += Tj * (Pij  + Dik +Cjk);
									//cout << "; Pij="<< Pij<<"; dik="<< dik << endl;
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
			double temp_fitness = w1 * temp_fitness_1 + w2 * temp_fitness_2 + w3 * temp_fitness_3;//��Ӧֵ
			if (isInRader(temp_part.state))
				temp_fitness = 0;
			if (temp_fitness >= temp_part.fitness) {//�ֲ�����λ���ж�
				temp_part.fitness = temp_fitness;
				temp_part.Pbest_state = temp_part.state;
			}
			//cout<<j<<endl;

			if (temp_fitness >= uav->Gbest_fitness) {//ȫ������λ���ж�
													 //cout << "����temp_fitness" << temp_fitness << " current particle" << j << endl;
				uav->Gbest_fitness = temp_fitness;
				uav->Gbest_state = temp_part.state;
			}
			if (uav->id == 0)
				particle_state << temp_part.state.position.x << " " << temp_part.state.position.y << " ";

		}
		if (uav->id == 0) {
			particle_state << endl;
			uav_state << uav->state.position.x << " " << uav->state.position.y << endl;
			best_state << uav->Gbest_state.position.x << " " << uav->Gbest_state.position.y << endl;
		}

		//if (uav->last_Pbest_state.position.x() == uav->Pbest_state.position.x() && uav->last_Pbest_state.position.y() == uav->Pbest_state.position.y())
		//    break;
	}
	//cout<<"uav->Gbest_fitness"<<uav->Gbest_fitness<<endl;
	cout << "Gbest_fitness = " << uav->Gbest_fitness << endl;
	uav->traj_Point = uav->Gbest_state;
	uav->updatePrevPoses();
	return;
}

void ParticleSwarm::init() {
	//obstacle
	for (int i = 0; i < radar_num; i++) {
		utility::radar* temp_rader = new utility::radar();
		temp_rader->center_point.x = 50000;
		temp_rader->center_point.y = 50000;
		temp_rader->R = 10000;
		radar_.push_back(temp_rader);
	}

	//target
	for (int i = 0; i<target_num; i++) {
		utility::TARGET* temp_target = new utility::TARGET();
		temp_target->state.position.x = 30000 + i * 20000;//PosMin.x + (PosMax.x - PosMin.x) * (double)rand() / RAND_MAX;10+i*100
		temp_target->state.position.y = 50000 + i * 20000;//PosMin.y + (PosMax.y - PosMin.y) * (double)rand() / RAND_MAX;30+i*200
		temp_target->state.velocity.x = 40;//���ٶ�
		temp_target->state.velocity.y = 2 * pi*rand() / RAND_MAX;//�Ƕ�
		temp_target->state.accelerate.x = 0;//�߼��ٶ�
		temp_target->state.accelerate.y = 0;//���ٶ�
		target.push_back(temp_target);
	}
	//map
	for (int i = 0; i < size_y; i++) {//��ʼ����ͼ��i=0�����һ�У�j=0�����һ��
		vector<utility::grid*> temp;
		for (int j = 0; j < size_x; j++) {
			utility::grid* tempGridPtr = new utility::grid();
			for (int k = 0; k<UAV_num; k++)
				tempGridPtr->search_count.push_back(forget_time);
			temp.push_back(tempGridPtr);
		}
		global_map.push_back(temp);
	}
	//sparse_map
	for (int i = 0; i < sparse_size_y; i++) {//��ʼ����ͼ��i=0�����һ�У�j=0�����һ��
		vector<utility::grid*> temp;
		for (int j = 0; j < sparse_size_x; j++) {
			utility::grid* tempGridPtr = new utility::grid();
			for (int k = 0; k<UAV_num; k++)
				tempGridPtr->search_count.push_back(forget_time);
			temp.push_back(tempGridPtr);
		}
		sparse_map.push_back(temp);
	}
	//UAV
	for (int i = 0; i < UAV_num; i++) {//��ʼ�����˻�
		utility::UAV* uav_temp = new utility::UAV();
		srand((int(time(NULL)) + i));
		uav_temp->id = i;

		//uav_temp->state.position.x = (PosMax.x-PosMin.x) *(rand() / double(RAND_MAX)) ;//λ��
		//uav_temp->state.position.y = (PosMax.y - PosMin.y) *(rand() / double(RAND_MAX));
		uav_temp->state.position.x = 0;//λ��
		uav_temp->state.position.y = (PosMax.x - PosMin.x) / (UAV_num + 2) * (i + 1);
		uav_temp->state.velocity.x = VelMin.x + (VelMax.x - VelMin.x) * (rand() / double(RAND_MAX));//���ٶ�
		uav_temp->state.velocity.y = 0;//�Ƕ�
		uav_temp->state.accelerate.x = 0;//�߼��ٶ�
		uav_temp->state.accelerate.y = 0;//���ٶ�
		uav_temp->search_r = search_R;
		uav_temp->particle_r = particle_R;
		uav_temp->coverd_area_cnt = 5000;
		for (int j = 0; j < target_num; j++) {
			utility::point2D target_pose(-1, -1);
			uav_temp->target_position.push_back(make_pair(target_pose, forget_time));
			uav_temp->Tj[j] = 1;
		}
		updateSubMap(uav_temp);
		for (int j = 0; j < particle_num; j++) {
			utility::particle* temp = new utility::particle();
			uav_temp->swarm.push_back(temp);
		}
		cout << "1" << endl;
		spreadParticles(uav_temp);
		cout << "2" << endl;
		updateParticleStates(uav_temp);
		cout << "3" << endl;
		updateUAVStatesInDubinsState(uav_temp);
		cout << "4" << endl;
		while (uav_temp->path_.size()>1) {//only keep one state
			uav_temp->path_.pop();
		}

		uav_temp->track_target_num = -1;
		uav.push_back(uav_temp);
		union_.parent.push_back(i);
	}
};

void ParticleSwarm::updateTargetStates() {
	//Ŀ��״̬���£������ǵ����˶�
	output_target.open(target_path.c_str(), ios::app | ios::binary);
	for (int target_tag = 0; target_tag < target_num; target_tag++) {
		double line_X = target[target_tag]->state.velocity.x * cos(target[target_tag]->state.velocity.y);//�ٶ���XY�ķ���
		double line_Y = target[target_tag]->state.velocity.x * sin(target[target_tag]->state.velocity.y);
		target[target_tag]->state.position.x += line_X * dt;//λ�ø���
		target[target_tag]->state.position.y += line_Y * dt;

		//�߽��ж�
		if (target[target_tag]->state.position.x < PosMin.x) {
			target[target_tag]->state.position.x = PosMin.x;
			target[target_tag]->state.velocity.y = pi * (0.5 - (rand() % 1000) / 1000.0);//�Ƕ�
		}
		else if (target[target_tag]->state.position.x > PosMax.x) {
			target[target_tag]->state.position.x = PosMax.x;
			target[target_tag]->state.velocity.y = pi * (0.5 + (rand() % 1000) / 1000.0);//�Ƕ�
		}

		if (target[target_tag]->state.position.y < PosMin.y) {
			target[target_tag]->state.position.y = PosMin.y;
			target[target_tag]->state.velocity.y = pi * ((rand() % 1000) / 1000.0);//�Ƕ�
		}
		else if (target[target_tag]->state.position.y > PosMax.y) {
			target[target_tag]->state.position.y = PosMax.y;
			target[target_tag]->state.velocity.y = pi * (1 + (rand() % 1000) / 1000.0);//�Ƕ�
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
	for (int i = 0; i<UAV_num; i++) {
		if (uav[i]->path_.empty()) {
			uav[i]->state.position.x += uav[i]->state.velocity.x*cos(uav[i]->state.velocity.y)*dt;
			uav[i]->state.position.y += uav[i]->state.velocity.x*sin(uav[i]->state.velocity.y)*dt;
			output_uav << uav[i]->state.position.x << " " << uav[i]->state.position.y << " " << uav[i]->track_target_num << " ";
			output_traj_Point << uav[i]->traj_Point.position.x << " " << uav[i]->traj_Point.position.y << " ";
			cout << "error" << endl;
			continue;
		}
		uav[i]->state.position.x = uav[i]->path_.front()[0];
		uav[i]->state.position.y = uav[i]->path_.front()[1];
		uav[i]->state.velocity.x = uav[i]->path_.front()[2];
		uav[i]->state.velocity.y = uav[i]->path_.front()[3];
		uav[i]->path_.pop();
		output_uav << uav[i]->state.position.x << " " << uav[i]->state.position.y << " " << uav[i]->track_target_num << " ";
		output_traj_Point << uav[i]->traj_Point.position.x << " " << uav[i]->traj_Point.position.y << " ";
		updateSubMap(uav[i]);
		for (int j = 0; j<target_num; j++) {//target inf0
			if (dist(target[j]->state.position, uav[i]->state.position)<search_R) {
				uav[i]->target_position[j].first = target[j]->state.position;
				uav[i]->target_position[j].second = 0;
			}
			else
				uav[i]->target_position[j].second++;//search_cunt++
		}
	}
	output_uav << endl;
	output_uav.close();
	output_traj_Point << endl;
	output_traj_Point.close();
	return;
}

void ParticleSwarm::informationShare() {
	for (int i = 0; i < UAV_num; i++) {//reset
		union_.parent[i] = i;
		uav[i]->coverd_area_cnt = 0;
	}
	for (int i = 0; i < UAV_num; i++) {
		for (int j = i; j < UAV_num; j++) {
			if (dist(uav[i]->state.position, uav[j]->state.position) <= communication_R) {//union
				union_.join(i, j);
			}
			for (int k = 0; k<target_num; k++) {//exchange target info
				if (uav[i]->target_position[k].second>uav[j]->target_position[k].second)
					uav[i]->target_position[k] = uav[j]->target_position[k];
				else
					uav[j]->target_position[k] = uav[i]->target_position[k];
			}
		}
	}

	union_.setup_barrel();//setup barrel
	int sum[9] = { 0 };
	for (int i = 0; i<size_y; i++) {
		for (int j = 0; j <size_x; j++) {
			for (int k = 0; k<union_.barrel.size(); k++) {
				int min_count = forget_time;
				for (int l = 0; l<union_.barrel[k].size(); l++) {
					min_count = std::min(min_count, global_map[i][j]->search_count[union_.barrel[k][l]]);//find the lowest search time
				}
				for (int l = 0; l<union_.barrel[k].size(); l++) {
					global_map[i][j]->search_count[union_.barrel[k][l]] = min_count;//set the search time
					if (min_count<forget_time)
						uav[union_.barrel[k][l]]->coverd_area_cnt++;//count the coverd area
																	//sparse_map update
					if (i == 0 || j == 0)
						continue;
					if (i%rato == 0 && j%rato == 0) {
						sparse_map[j / rato - 1][i / rato - 1]->search_count[union_.barrel[k][l]] = sum[union_.barrel[k][l]] / 40000;
						sum[union_.barrel[k][l]] = 0;
					}
					else {
						sum[union_.barrel[k][l]] += min_count;
					}

				}
			}

		}
	}


	return;
}

void ParticleSwarm::updateMission() {
	for (int i = 0; i < UAV_num; i++) {
		for (int j = 0; j < target_num; j++) {//������˻�i���ֵ�Ŀ��j���ڵ�դ���������������1000��
											  //cout << "uav[i]->target_position[j].second = " << uav[i]->target_position[j].second  << endl;
			if (uav[i]->target_position[j].second >= forget_time)
				target_state[i][j] = 0;//�������˻�iȥ׷Ŀ��j
			else
				target_state[i][j] = 1000.0 / dist(uav[i]->state.position, target[j]->state.position);
		}
	}
	set<int> row, col;
	for (int l = 0; l < target_num; l++) {
		double maxTargetState = 0;
		int uav_cnt, target_cnt;
		for (int i = 0; i < UAV_num; i++) {//Ѱ���������������ֵ����������
			for (int j = 0; j < target_num; j++) {
				if (row.find(i) == row.end() && col.find(j) == col.end() && target_state[i][j] > maxTargetState) {
					maxTargetState = target_state[i][j];
					uav_cnt = i;
					target_cnt = j;
				}
			}
		}
		if (maxTargetState > 0.001) {//���ܹ��ҵ���Ч���������ʱ��ִ���������
			cout << "uav_cnt = " << uav_cnt << " target_cnt = " << target_cnt << endl;
			if (dist(uav[uav_cnt]->state.position, target[target_cnt]->state.position) < 500) {
				tracked[target_cnt] = true;
				uav[uav_cnt]->state.velocity.x = target[target_cnt]->state.velocity.x;
				cout << "target " << target_cnt << " is successfully tracked by uav " << uav_cnt << endl;
			}

			for (int i = 0; i < UAV_num; i++) {
				if (i == uav_cnt) {
					uav[i]->Tj[target_cnt] = 1;//��������˻�׷�ٸ�Ŀ��
				}
				else
					uav[i]->Tj[target_cnt] = 0;//�����������˻���׷�ٸ�Ŀ��
			}
			row.insert(uav_cnt); col.insert(target_cnt);//�������з���set�У��´�ѭ��������������
		}
	}
	for (int i = 0; i < UAV_num; i++) {
		int j = 0;
		for (; j < target_num; j++) {
			//cout << "uav[" << i << "].TJ[" << j << "]= " << uav[i].Tj[j] << " "<< uav[i].covered_target_id[j].second<< endl;
			if (uav[i]->Tj[j] == 1 && uav[i]->target_position[j].second < forget_time) {//��������˻�׷�ٸ�Ŀ��
				uav[i]->track_target_num = j;
				uav[i]->traj_Point = target[j]->state;
				uav[i]->state.velocity.x = 2 * target[j]->state.velocity.x;
				updateUAVStatesInDubinsState(uav[i]);
				cout << "uav " << i << " is tracking target " << j << endl;
				break;
			}
		}
		if (j >= target_num) {
			uav[i]->track_target_num = -1;
			cout << "uav_cnt = " << i << " is free" << endl;
		}//�����˻���δ�����κ�Ŀ��
	}
}

void ParticleSwarm::run() {
	init();
	int cunt = 0;
	while (cunt < 10000) 
	{
		//������˻�����ȫ�����ţ�������������
		for (int i = 0; i < UAV_num; i++) {
			if (uav[i]->path_.empty()) {
				cout << endl << endl << endl << "replanning uav" << i << endl << endl << endl;
				if (uav[i]->track_target_num == -1) {
					spreadParticles(uav[i]);
					updateParticleStates(uav[i]);
					updateUAVStatesInDubinsState(uav[i]);
				}
				else
				{
					updateUAVStatesInDubinsState(uav[i]);
				}
			}
		}

		//����Ŀ��λ��
		updateTargetStates();

		//�������˻�״̬
		updateUAVStates();

		//����ͨ��
		if (UAV_num>1)
			informationShare();

		//�������
		updateMission();

		cout << "cunt" << cunt << endl;

		//�ж���ֹ
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