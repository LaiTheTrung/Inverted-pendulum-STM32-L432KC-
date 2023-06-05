#include "stm32l4xx.h"                  // Device header

void updateValue (int newMes, int t, int *mes_array, int *t_array);
int PI(int P, int I, int expected,int ctr_value, int *mes_array, int *t_array );
double PID(double P, double I, double D, int expected,double ctr_value, int *mes_array, int *t_array );
void updateValue2 (int newMes, int *mes_array);
double PD(double P, double D, int expected,double ctr_value, int *mes_array, int deltaT );