#include "dqn.h"

float learn_rate = 1;
float reliablity = 10;
float last_rate = 0.7;
int max_epoch = 10;

int data_num; 			//保存当前数据位数，超过1000最大存储后置零
int min_data_num = 3;
const int m_size = 90;


int dead_angle = 1;	// 停止存数据的最大角，大于此角度会临时调用原pid算法，不保存任何数据，但继续用数据库中数据学习

int live_time = 0;		// 生存时间，用于给reward，死后重置
float min_error = 10000;

int max_num;	// 用于取最大值，用不惯c的返回值（不能返回多个数据）
int is_learning = 0;


const int i_size=4, h_size=10, o_size=11;
float targets[o_size],errors1[h_size], errors2[o_size];

struct dqnNet{
	float inputs[i_size], hiddens[h_size], outputs[o_size];
	float w1[i_size][h_size], w2[h_size][o_size], b1[i_size][h_size], b2[h_size][o_size];
}net_update, net_main;

struct memory{
	float inputs[i_size];
	float hiddens[h_size];
	float outputs[o_size];
	int live_time;
}memorys[m_size];

// 用于计算reward
extern int 	EncoderLeft,EncoderRight;

float my_abss(float x){
	return x ? (x>0) : -x;
}
float sigmoid(float x){
	return 1/(1+exp(-x));
}

float desigmoid(float x){
	return sigmoid(x)*(1-sigmoid(x));
}

void clone_u2m(){
	for (int i=0; i<i_size; i++)
		for (int j=0; j<h_size; j++){
			net_main.w1[i][j] = net_update.w1[i][j];
			net_main.b1[i][j] = net_update.b1[i][j];
		}
	for (int i=0; i<h_size; i++)
		for (int j=0; j<o_size; j++){
			net_main.w2[i][j] = net_update.w2[i][j];
			net_main.b2[i][j] = net_update.b2[i][j];
		}
}

float get_max(struct dqnNet net){
	max_num = 0;
	float max_val = net.outputs[0];
	for (int i=1; i<o_size; i++)
		if (net.outputs[i] > max_val){
			max_num = i;
			max_val = net.outputs[i];
		}
	return max_val;
}


void get_rand_outputs(){
	srand(1);
	int rand_num;
	for(int i=0; i<o_size; i++){
		rand_num = rand()%200/100-1;
		net_update.outputs[i] = rand_num;
	}
}

void get_i(struct dqnNet net, double KalmanAngleX, double Gx){
	float K = KalmanAngleX;
	float G = Gx;
	float EL = EncoderLeft;
	float ER = EncoderRight;
	net.inputs[0] = K;
	net.inputs[1] = G;
	net.inputs[2] = EL;
	net.inputs[3] = ER;
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void get_o(struct dqnNet net){
	
	for (int i=0; i<h_size; i++)	net.hiddens[i] = sigmoid(net.w1[i][0]*net.inputs[0] + net.b1[i][0]);
	for (int i=0; i<h_size; i++)
		for (int j=1; j<i_size; j++)
			net.hiddens[i] += sigmoid( net.w1[i][j]*net.inputs[j] + net.b1[i][j]);
	
	for (int i=0; i<o_size; i++)	net.outputs[i] = sigmoid(net.w2[i][0]*net.hiddens[0] + net.b2[i][0]);
	for (int i=0; i<o_size; i++)
		for (int j=1; j<h_size; j++)
			net.outputs[i] += sigmoid( net.w2[i][j]*net.hiddens[j] + net.b2[i][j]);
}


void get_e(float outputs[], float hiddens[], float reward, float max_Q){
	for (int i=0; i<o_size; i++){
		if (i==max_num)
			errors2[i] = (outputs[i] - max_Q*last_rate - reward) * desigmoid(outputs[i]);
		else errors2[i] = outputs[i] * desigmoid(outputs[i]);
	}
	for (int i=0; i<h_size; i++){
		errors1[i] = errors2[0]*net_update.w2[i][0];
		for (int j=1; j<o_size; j++)
			errors1[i] += errors2[j]*net_update.w2[i][j];
		errors1[i] *= desigmoid(hiddens[i]);
	}
}

float get_error(float outputs[]){
	float ans = 0;
	for(int i=0; i<o_size; i++)
		ans += (outputs[i] - targets[i])*(outputs[i] - targets[i])/2;
	return ans;
}

void learn(){
	srand(21);
	for (int _=0; _<max_epoch; _++){
		
		//随机取数据
		int rand_num = rand()%data_num;
		while (memorys[rand_num+1].live_time - memorys[rand_num].live_time != 1)
			rand_num = rand()%data_num;
		
		for (int i=0; i<i_size; i++) 	net_main.inputs[i] = memorys[rand_num].inputs[i];
		get_o(net_main);
		float max_val = get_max(net_main);
		
		float reward = memorys[rand_num].live_time * 0.01 - memorys[rand_num+1].inputs[0] * reliablity/50 - memorys[rand_num+1].inputs[1] * reliablity/10000;
		if (memorys[rand_num].live_time > 100)
			reward = 1 - memorys[rand_num+1].inputs[0] * reliablity/50 - memorys[rand_num+1].inputs[1] * reliablity/10000;
		reward = sigmoid(reward);
		
		for (int i=0; i<o_size; i++){
			if (max_num==i)		targets[i] = max_val*last_rate + reward - memorys[rand_num].outputs[i];
			else				targets[i] = 0;
		}
			
		get_e(memorys[rand_num].outputs, memorys[rand_num].hiddens, reward, max_val);
		printf("%f\r\n", get_error(memorys[rand_num].outputs));
			
		for (int i=0; i<h_size; i++)
			for (int j=0; j<o_size; j++){
				net_update.w2[i][j] += learn_rate*errors2[j]*memorys[rand_num].hiddens[i];
				net_update.b2[i][j] += learn_rate*errors2[j];
			}

		for (int i=0; i<i_size; i++)
			for (int j=0; j<h_size; j++){
				net_update.w1[i][j] += learn_rate*errors1[j]*memorys[rand_num].inputs[i];
				net_update.b1[i][j] += learn_rate*errors1[j];
			}
		
		float last_error = get_error(memorys[rand_num].outputs);
		if (last_error < min_error){
			reliablity++;
			dead_angle += 0.1;
			min_error = last_error;
		}
	}
}

void accumulate(double KalmanAngleX, double Gx){
	get_i(net_update, KalmanAngleX, Gx);
	srand(32);
	int rand_num = rand()%100;
	if (rand_num > reliablity)	get_o(net_update);
	else						get_rand_outputs();

	for (int i=0; i<i_size; i++)	memorys[data_num].inputs[i] = my_abss(net_update.inputs[i]);		//注意，此处用了abs了！
	for (int i=0; i<h_size; i++)	memorys[data_num].hiddens[i] = net_update.hiddens[i];
	for (int i=0; i<o_size; i++)	memorys[data_num].outputs[i] = net_update.outputs[i];
	memorys[data_num].live_time = live_time;
	
	data_num++;
	if (data_num>m_size-1)	data_num = 0;
}

int my_main(double KalmanAngleX, double Gx){
	
	if (is_learning){
		learn();
	}
	else{
		if (data_num > min_data_num)	is_learning = 1;
	}
	if (! (data_num % 10))	clone_u2m();
	
	if (KalmanAngleX > dead_angle || KalmanAngleX < -dead_angle){	// 存活判断
		live_time = 0;
		return 1;
	}
	else{
		live_time++;
		accumulate(KalmanAngleX, Gx);
		get_max(net_main);
		return 0;
	}
}

