#include "control.h"
/************************
ֱ�������Ʋ���:KP:410,KD:0.133
************************/
float Vertical_Kp=500;
float Vertical_Kd=0.001;

//float test = 0.0f;
//float test_last = 0.0f;
//float test_d = 0.0f;
//float test_sum = 0.0f;
//float test_ki = 3.7f;
//float test_limit = 1000.0f;
//float angle_limit = 0.01f;
//int v_sum = 0;


/*************************
�ٶȻ����Ʋ���:KP:500,KI:600/200
*************************/
float Velocity_Kp=0;
float Velocity_Ki=0;
const float max_encoder = 800;

/*************************
ת�򻷿��Ʋ���:KP:-0.05,KP2:0.3
**************************/
float Turn_Kp=-0.05;/*����Ϊ��*/
float Turn_Kp2=0.3;

/*************************
����ֵ����
**************************/
int my_abs(int i)
{
	if(i<=0)i=-i;
	else i=i;
	return i;
}


/*************************
PWM���뺯��
**************************/
void Load(int moto1,int moto2)//moto1=-200����ת200������
{
	//1.�о������ţ���Ӧ����ת
	if(moto1>0)	{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_12,GPIO_PIN_SET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_13,GPIO_PIN_RESET) ;}//��ת
	else 				{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_12,GPIO_PIN_RESET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_13,GPIO_PIN_SET) ;}//��ת
	//2.�о�PWMֵ
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,my_abs(moto1 ));
	
	if(moto2>0)	{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_14,GPIO_PIN_SET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_15,GPIO_PIN_RESET) ;}
	else 				{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_14,GPIO_PIN_RESET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_15,GPIO_PIN_SET) ;}	
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,my_abs(moto2 ));
}

/*********************
ֱ����PD��������Kp*Ek+Kd*Ek_D

��ڣ������Ƕȡ���ʵ�Ƕȡ���ʵ���ٶ�
���ڣ�ֱ�������
*********************/
int Vertical(float Med,float Angle,float gyro_Y)
{
	int PWM_out;
//	test = Angle-Med;
//	test_d = test-test_last;
//	if(test > 0.24 || test < -0.24){
	PWM_out=Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);
//		PWM_out=Vertical_Kp*test-Vertical_Kd*gyro_Y + test_ki*test_sum;//��1�� ���Դ���
//	}
//	else PWM_out = 0;
//	
//	test_sum = test_sum + test;
//	if (test_sum > test_limit ){
//		test_sum = test_limit;}
//	else if(test_sum < -test_limit){
//		test_sum = -test_limit;
//	}
//		
//	test_last = test;
	return PWM_out;
}

/*********************
�ٶȻ�PI��Kp*Ek+Ki*Ek_S
*********************/
int Velocity(int target_speed,int encoder_left,int encoder_right)
{
	static int PWM_out,Encoder_Err,Encoder_S,EnC_Err_Lowout,EnC_Err_Lowout_last;//��2��
	float a=0.8;//��3��
	
	//1.�����ٶ�ƫ��
	Encoder_Err=(encoder_left+encoder_right)-target_speed/2000 ;//��ȥ���
	//2.���ٶ�ƫ����е�ͨ�˲�
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//ʹ�ò��θ���ƽ�����˳���Ƶ���ţ���ֹ�ٶ�ͻ�䡣
	EnC_Err_Lowout_last=EnC_Err_Lowout;//��ֹ�ٶȹ����Ӱ��ֱ����������������
	//3.���ٶ�ƫ����֣����ֳ�λ��
	Encoder_S+=EnC_Err_Lowout;//��4��
	//4.�����޷�
	Encoder_S=Encoder_S>max_encoder?max_encoder:(Encoder_S<(-max_encoder)?(-max_encoder):Encoder_S);
	//5.�ٶȻ������������
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;//��5��
//	v_sum = Encoder_S;
	return PWM_out;
}



/*********************
ת�򻷣�ϵ��*Z����ٶ�
*********************/
int Turn(int gyro_Z,int speed)
{
	int PWM_out;
	
	PWM_out=Turn_Kp*gyro_Z+Turn_Kp2*speed ;
	return PWM_out;
}
