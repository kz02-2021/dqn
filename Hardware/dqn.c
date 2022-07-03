//#include "arm_math.h"
#include "mpu6050.h"
#include<math.h>

double learn_rate = 0.01;

const int i_size=4, h_size=10, o_size=3;
double inputs[i_size], hiddens[h_size], outputs[o_size], targets[o_size], errors1[h_size], errors2[o_size];
double w1[i_size][h_size], w2[h_size][o_size], b1[i_size][h_size], b2[h_size][o_size];

extern int 	EncoderLeft,EncoderRight;
extern MPU6050_t MPU6050;

double sigmoid(double x){
	return 1/(1+exp(-x));
}

double desigmoid(double x){
	return sigmoid(x)*(1-sigmoid(x));
}
	
	
void get_it(){
	inputs[0] = MPU6050.KalmanAngleX;
	inputs[1] = MPU6050.Gx;
	inputs[2] = EncoderLeft;
	inputs[3] = EncoderRight;
}

void get_o(){
	for (int i=0; i<h_size; i++)	hiddens[i] = sigmoid(w1[i][0]*inputs[0] + b1[i][0]);
	for (int i=0; i<h_size; i++)
		for (int j=1; j<i_size; j++)
			hiddens[i] += sigmoid( w1[i][j]*inputs[j] + b1[i][j]);
	
	for (int i=0; i<o_size; i++)	outputs[i] = sigmoid(w2[i][0]*hiddens[0] + b2[i][0]);
	for (int i=0; i<o_size; i++)
		for (int j=1; j<h_size; j++)
			outputs[i] += sigmoid( w2[i][j]*hiddens[j] + b2[i][j]);
}

void get_e(){
	for (int i=0; i<o_size; i++)
		errors2[i] = (outputs[i] - targets[i]) * desigmoid(outputs[i]);
	for (int i=0; i<h_size; i++){
		errors1[i] = errors2[0]*w2[i][0];
		for (int j=1; j<o_size; j++)
			errors1[i] += errors2[j]*w2[i][j];
	}
}

double get_error(){
	double ans = 0;
	for(int i=0; i<o_size; i++)
		ans += (outputs[i] - targets[i])*(outputs[i] - targets[i])/2;
	return ans;
}

void learn(){
	get_it();
	get_o();
	get_e();
	for (int i=0; i<h_size; i++)
		for (int j=0; j<o_size; j++){
			w2[i][j] += learn_rate*errors2[j]*hiddens[i];
			b2[i][j] += learn_rate*errors2[j];
		}

	for (int i=0; i<i_size; i++)
		for (int j=0; j<h_size; j++){
			w1[i][j] += learn_rate*errors1[j]*inputs[i];
			b1[i][j] += learn_rate*errors1[j];
		}			
}

int main(void){
	
}

