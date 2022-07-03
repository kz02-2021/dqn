#include "arm_math.h"

const int i_size=7, h_size=10, o_size=3;
double inputs[i_size], hiddens[h_size], outputs[o_size], targets[o_size], errors[o_size];
double w1[i_size][h_size], w2[h_size][o_size], b1[i_size][h_size], b2[h_size][o_size];




double sigmoid(double x){
	
}

void get_it(){
	;
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
	
	
}

void learn(){
	
}

