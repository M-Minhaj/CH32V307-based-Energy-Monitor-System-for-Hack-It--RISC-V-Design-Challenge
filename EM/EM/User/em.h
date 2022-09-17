#ifndef ZMPT101B_h
#define ZMPT101B_h
#include <stdio.h>
#include <math.h>
#include "debug.h"
#include "moving_average.h"

//Calibration
#define CAL_FACTORV 1
#define R1V 130000
#define R2V 576.0
#define CAL_FACTORI 1
#define R1I 150.0
#define ADC_SCALE 4095.0
#define VREF 3300
#define DEFAULT_FREQUENCY 50.0
#define OFFESET_W 8
#define OFFESET_V 0
#define OFFESET_I -0.17

//Control Setting
#define LOAD_RESTART_TIME 30.0
#define MAX_LOAD 15.0
#define UNDER_VOLT 170.0
#define OVER_VOLT 250.0
#define HYST 10.0
#define START_DELAY 5



float getRMS(short i);

float V_rms(void);

float I_rms(void);
float P_rms(void);

float V_ins(void);
float I_ins(void);
void plotter(void);
float P_avg(void);

#endif
