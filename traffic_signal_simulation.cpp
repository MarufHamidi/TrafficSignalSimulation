#include<iostream>
#include<fstream>
#include<ctime>
#include<cmath>

using namespace std;

ifstream conf("conf.txt");
ofstream logger("log.txt");
ofstream csv("output.csv");
ofstream dynamic("dynamic.data");
ofstream static_data("static.data");

int total_iteration;
double total_running_time;
double total_lost_time;
double saturation_headway;
int total_roads;
int *road_flow_weights;
int *road_saturation_flow;
double flow_scale;
int random_seed;
double static_green_time;

void get_default_conf(){
	//cout<<"Default configuration."<<endl;
	total_iteration = 100;
	total_running_time = 3600;

	total_lost_time = 12;
	saturation_headway = 2.5;
	
	total_roads = 4;
	road_flow_weights = new int[total_roads];
	road_saturation_flow = new int[total_roads];
	road_flow_weights[0] = 890;
	road_saturation_flow[0] = 1800;
	road_flow_weights[1] = 183;
	road_saturation_flow[1] = 800;
	road_flow_weights[2] = 328;
	road_saturation_flow[2] = 1500;
	road_flow_weights[3] = 412;
	road_saturation_flow[3] = 1700;
	flow_scale = 1;
	static_green_time = 120;
	srand(time(NULL));	
	//cout<<"Time : "<<time(NULL)<<endl;
	logger<<"Time : "<<time(NULL)<<endl;	
	random_seed = rand();
	//cout<<"Random seed : "<<random_seed<<endl;
	logger<<"Random seed : "<<random_seed<<endl;
	srand(random_seed);
	for(int i=0;i<total_roads;i++){
		srand(rand());
	}
}

void get_conf(){
	conf>>total_iteration;
	if(total_iteration == -1){
		get_default_conf();
		return;
	}
	conf>>total_running_time;
	conf>>total_lost_time;
	conf>>saturation_headway;
	conf>>total_roads;
	road_flow_weights = new int[total_roads];
	road_saturation_flow = new int[total_roads];
	for(int i=0;i<total_roads;i++){
		conf>>road_flow_weights[i];
		conf>>road_saturation_flow[i];
	}
	conf>>flow_scale;
	conf>>static_green_time;
	conf>>random_seed;
	if(random_seed == -1){
		srand(time(NULL));
		//cout<<"Time : "<<time(NULL)<<endl;
		logger<<"Time : "<<time(NULL)<<endl;
		random_seed = rand();
	}
	//cout<<"Random seed : "<<random_seed<<endl;
	logger<<"Random seed : "<<random_seed<<endl;
	srand(random_seed);
	for(int j=0;j<total_roads;j++){
		srand(rand());
	}
}

int get_vehicle_count(int road_index) {
	double r = ((double)rand()/(double)RAND_MAX);;
	int count;
	count = (int) ceil(-road_flow_weights[road_index] * log(r) * flow_scale);
	if(count > road_saturation_flow[road_index])return road_saturation_flow[road_index];
	return count;
}

int simulate(){
	int i;
	double current_time = 0;
	int cycle_count = 0;
	double *y = new double[total_roads];
	double Y;
	double optimum_cycle_time;
	double total_effective_green_time;
	int *capacity = new int[total_roads];
	int total_capacity = 0;
	int *vcount = new int[total_roads];
	int *prev_count = new int[total_roads];
	for(i=0;i<total_roads;i++)prev_count[i] = 0;
	do{
		cycle_count++;
		//cout<<"Cycle : "<<cycle_count<<endl;
		logger<<"Cycle : "<<cycle_count<<endl;
		
		Y = 0.0;
		for(i=0;i<total_roads;i++){
			vcount[i] = get_vehicle_count(i);
			
			//cout<<"Vehicle count for road "<<i<<" during cycle "<<cycle_count<<" : "<<vcount[i]<<endl;
			logger<<"Vehicle count for road "<<i<<" during cycle "<<cycle_count<<" : "<<vcount[i]<<endl;
			
			//vcount[i] += prev_count[i];
			if(vcount[i] > (road_saturation_flow[i] / total_roads)){
				vcount[i] = road_saturation_flow[i] / total_roads;
				vcount[i]--;
			}
			
			y[i] = vcount[i] * 1.0 / road_saturation_flow[i];
			Y += y[i];
		}
		optimum_cycle_time = ceil(((1.5 * total_lost_time) + 5) / (1 - Y));
		total_effective_green_time = optimum_cycle_time - total_lost_time;
		
		//cout<<"Optimum cycle time ("<<cycle_count<<") : "<<optimum_cycle_time<<endl;
		//cout<<"Total effective green time ("<<cycle_count<<") : "<<total_effective_green_time<<endl;
		logger<<"Optimum cycle time ("<<cycle_count<<") : "<<optimum_cycle_time<<endl;
		logger<<"Total effective green time ("<<cycle_count<<") : "<<total_effective_green_time<<endl;
		
		for(i=0;i<total_roads;i++){
			capacity[i] = ceil((y[i] / Y ) * (road_saturation_flow[i] / saturation_headway));
			//prev_count[i] = vcount[i] - capacity[i];
			
			//cout<<"Capacity for road "<<i<<" during cycle "<<cycle_count<<" : "<<capacity[i]<<endl;
			logger<<"Capacity for road "<<i<<" during cycle "<<cycle_count<<" : "<<capacity[i]<<endl;
			
			total_capacity += capacity[i];
		}
		
		current_time += optimum_cycle_time;
	}while(current_time <= total_running_time);
	
	//cout<<"Total cycle : "<<cycle_count<<endl;
	//cout<<"Total capacity : "<<total_capacity<<endl;
	logger<<"Total cycle : "<<cycle_count<<endl;
	logger<<"Total capacity : "<<total_capacity<<endl;
	return total_capacity;
}

int simulate(bool dynamic){
	if(dynamic)return simulate();
	
	int i;
	double current_time = 0;
	int cycle_count = 0;
	double *y = new double[total_roads];
	double Y;
	double optimum_cycle_time;
	double total_effective_green_time;
	int *capacity = new int[total_roads];
	int total_capacity = 0;
	int *vcount = new int[total_roads];
	int *prev_count = new int[total_roads];
	for(i=0;i<total_roads;i++)prev_count[i] = 0;
	do{
		cycle_count++;
		//cout<<"Cycle : "<<cycle_count<<endl;
		logger<<"Cycle : "<<cycle_count<<endl;
		
		Y = 0.0;
		for(i=0;i<total_roads;i++){
			vcount[i] = get_vehicle_count(i);
			
			//cout<<"Vehicle count for road "<<i<<" during cycle "<<cycle_count<<" : "<<vcount[i]<<endl;
			logger<<"Vehicle count for road "<<i<<" during cycle "<<cycle_count<<" : "<<vcount[i]<<endl;
			
			//vcount[i] += prev_count[i];
			if(vcount[i] > (road_saturation_flow[i] / total_roads)){
				vcount[i] = road_saturation_flow[i] / total_roads;
				vcount[i]--;
			}	
				
			y[i] = vcount[i] * 1.0 / road_saturation_flow[i];
			Y += y[i];
		}
		
		optimum_cycle_time = (static_green_time + total_lost_time) * total_roads ;
		total_effective_green_time = optimum_cycle_time - total_lost_time;
		
		//cout<<"Optimum cycle time ("<<cycle_count<<") : "<<optimum_cycle_time<<endl;
		//cout<<"Total effective green time ("<<cycle_count<<") : "<<total_effective_green_time<<endl;
		logger<<"Optimum cycle time ("<<cycle_count<<") : "<<optimum_cycle_time<<endl;
		logger<<"Total effective green time ("<<cycle_count<<") : "<<total_effective_green_time<<endl;
		
		for(i=0;i<total_roads;i++){
			capacity[i] = ceil((y[i] / Y ) * (road_saturation_flow[i] / saturation_headway));
			//prev_count[i] = vcount[i] - capacity[i];
			
			//cout<<"Capacity for road "<<i<<" during cycle "<<cycle_count<<" : "<<capacity[i]<<endl;
			logger<<"Capacity for road "<<i<<" during cycle "<<cycle_count<<" : "<<capacity[i]<<endl;
			
			total_capacity += capacity[i];
		}
		
		current_time += optimum_cycle_time;
	}while(current_time <= total_running_time);
	
	//cout<<"Total cycle : "<<cycle_count<<endl;
	//cout<<"Total capacity : "<<total_capacity<<endl;
	logger<<"Total cycle : "<<cycle_count<<endl;
	logger<<"Total capacity : "<<total_capacity<<endl;
	return total_capacity;
}

int main(void)
{	
	get_conf();
	int drs;
	int stat;
	csv<<"Iteration, Dynammic, , , Iteration, Static"<<endl;
	for(int i=1;i<=total_iteration;i++){
		cout<<"----------------Iteration #"<<i<<" --------------------"<<endl;
		logger<<"----------------Iteration #"<<i<<" --------------------"<<endl;
		drs = simulate();
		stat = simulate(false);
		csv<<i<<", "<<drs<<", , , "<<i<<", "<<stat<<endl;
		dynamic<<drs<<endl;
		static_data<<stat<<endl;
	}
	system("cls");
	cout<<"Simulation has been completed successfully."<<endl<<endl;
	cout<<"\'.csv\' file contains result."<<endl<<endl;
	cout<<"Two \'.data\' files are also generated for matlab or octave."<<endl<<endl;
	cout<<"Whole simulation data is logged in the \'log.txt\' file."<<endl<<endl;
	cout<<"For further info read the instructions."<<endl<<endl;
	cout<<"Thank you."<<endl<<endl<<endl<<endl;
	cout<<"@ Maruf Hamidi & Albab Noor"<<endl;
	cin.get();
	return 0;
}