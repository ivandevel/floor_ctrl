#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "pid.h"
#include <math.h>
#include <inttypes.h>

#ifdef DFS_90

#define POW_MAX 1000
#define POW_MIN 0
#define PidH_Kp   20  
#define PidH_Ki   22  
#define PidH_Kd   20  
#define TSAMPLE   100

#else

#define POW_MAX 100L
#define POW_MIN 0L
#define PidH_Kp   20  
#define PidH_Ki   22  
#define PidH_Kd   20  
#define TSAMPLE   50

#endif

#pragma optimize=none
int16_t pid(uint16_t ref, int16_t inp)
{
  static int Xp = 0;  //���������� �������� inp
  static int Xpp = 0; //����-���������� �������� inp
  static long Y = 0;
  //���������� ������:
  int e = ref - inp;
  //���� ���������������� ������������:
  Y = Y - (long)PidH_Kp * (inp - Xp) * 10;
  //���� ������������ ������������:
  Y = Y + (long)PidH_Ki * e * TSAMPLE / 100;
  //���� ���������������� ������������:
  Y = Y - (long)PidH_Kd * (inp - 2 * Xp + Xpp) * 500 / TSAMPLE;
  //���������� ��������:
  Xpp = Xp; Xp = inp;
  if(!ref) Y = 0;
  //if(!PidH_En) Y = (long)out << 8;
  //����������� ��������� ��������:
	if(Y > POW_MAX * 256L) 
          Y = POW_MAX * 256L;
	if(Y < POW_MIN * 256L) 
          Y = POW_MIN * 256L;
	return(Y >> 8);
}


//Define parameter
#define epsilon 2
#define dt 800             //100ms loop time
#define MAX  1000        //For Current Saturation
#define MIN 0
#define Kp  100
#define Ki  100
#define Kd  100

#pragma optimize=none
int32_t PIDcal(int32_t setpoint, int32_t actual_position)
{
	static int32_t pre_error = 0;
	static int32_t integral = 0;
	int32_t error;
	int32_t derivative;
	int32_t output;

	//Caculate P,I,D
	error = setpoint - actual_position;

	//In case of error too small then stop intergration
	if(abs(error) > epsilon)
	{
		integral = integral + error/dt;
	}
	derivative = (error - pre_error)/dt;
	output = Kp*error + Ki*integral + Kd*derivative;

	//Saturation Filter
	if(output > MAX)
	{
		output = MAX;
	}
	else if(output < MIN)
	{
		output = MIN;
	}
        //Update error
        pre_error = error;

 return output;
}

