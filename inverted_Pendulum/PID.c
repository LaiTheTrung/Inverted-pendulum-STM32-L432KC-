#include "PID.h"                 // Device header

void updateValue (int newMes, int t, int *mes_array, int *t_array){
	for (int i=1; i<10; i++){
		mes_array[i-1] = mes_array[i];
	}
	mes_array[9] = newMes;
	for (int i=1; i<10; i++){
		t_array[i-1] = t_array[i];
	}
	t_array[9] = t;
}
void updateValue2 (int newMes, int *mes_array){
	mes_array[0] = mes_array[1];
	mes_array[1] = newMes;
}
int PI(int P, int I, int expected,int ctr_value, int *mes_array, int *t_array )
{
	int PI_value;
	PI_value = P* (expected-mes_array[9])*180/800;
	// sample time = 
	for (int i =0; i<10; i++){
		PI_value += I*(expected - mes_array[9])*180/800*t_array[i];
	}
	ctr_value += PI_value;
	return ctr_value ;
}
double PD(double P, double D, int expected,double ctr_value, int *mes_array, int deltaT )
{
	double PD_value = P*(double)(expected - mes_array[1]);
	PD_value += D* (double)(( mes_array[0] - mes_array[1] )/deltaT)*1000.0; 
	ctr_value += PD_value;
	return ctr_value ;
}

double PID(double P, double I, double D, int expected,double ctr_value, int *mes_array, int *t_array )
{
	double PID = P*(double)(expected - mes_array[9]);
	// sample time = 
	for (int i =0; i<10; i++){
		PID += I*(double)((expected - mes_array[9])*t_array[i])/1000.0; // t (ms)
	}
	PID += D* (double)(( mes_array[8] - mes_array[9] )/(t_array[9]+0.1))*1000.0; 
	ctr_value += PID;
	return ctr_value ;
	
}