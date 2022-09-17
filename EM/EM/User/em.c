#include "em.h"

extern u16 Adc_Val[4];
extern FilterTypeDef filterStruct_V, filterStruct_I;

float getRMS(short i)
{
	uint32_t period = 1000000 / DEFAULT_FREQUENCY;

	long measurements_count = 0;

	double Vsum;
	float Vnow;

	Vsum = 0;
	measurements_count = 0;

	SysTick->SR &= ~(1 << 0);
	SysTick->CMP = 9 * period;
	SysTick->CTLR |= (1 << 4) | (1 << 5) | (1 << 0);

	while ((SysTick->SR &(1 << 0)) != (1 << 0))

	{
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		Vnow = Adc_Val[i] - ADC_SCALE / 2;

		Vnow = Vnow *VREF / ADC_SCALE;

		Vnow = pow(Vnow, 2);

		Vsum = Vsum + Vnow;
		measurements_count++;

	}

	SysTick->CTLR &= ~(1 << 0);

	float rms = sqrt(Vsum / measurements_count);

	return rms;

}

float V_rms()

{
	float V = Moving_Average_Compute(getRMS(0), &filterStruct_V);

	V = R1V *(V / R2V);

	V = (V / (1000.0 *CAL_FACTORV));

	return V + OFFESET_V ;

}

float I_rms()

{
	float I = Moving_Average_Compute(getRMS(1), &filterStruct_I);

	I = (I) / (R1I *CAL_FACTORI);
	return I + OFFESET_I;

}

float P_rms()
{
	return V_rms() *I_rms();

}

float V_ins()
{
	float Vp = Adc_Val[0] - ADC_SCALE / 2;

	Vp = Vp * (VREF / ADC_SCALE);

	Vp = R1V * (Vp / R2V);

	Vp = ((Vp) / (1000.0 * CAL_FACTORV));
	return Vp;

}

float I_ins()
{
	float Ip = Adc_Val[1] - ADC_SCALE / 2;
	Ip = Ip * (VREF / ADC_SCALE);

	Ip = (Ip) / (R1I * CAL_FACTORI);

	return Ip;

}

void plotter()

{


//	while(1)
// {
	Delay_Us(20);
	float U = V_ins();
    float I = I_ins();
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	if (U < -1000)
		U = -1000;
	if (U > 1000)
		U = 1000;

	if (I < -50)
		I = -50;
	if (I > 50)
		I = 50;

	printf("V:");

	printf("%f", U);

	printf(" ");

	printf("I:");

	printf("%f", I);
	printf(" ");

	#if 0
	printf("VI:");
	printf("%f", U *I);
	printf(" ");
	#endif

	printf("\n ");

 //}

 }

float P_avg()
{
	uint32_t period_A = (1000000 / DEFAULT_FREQUENCY);

	u64 measurements_count_A = 0;

	double Vsum_A = 0;
	SysTick->SR &= ~(1 << 0);
	SysTick->CMP = 9 * period_A;
	SysTick->CTLR |= (1 << 4) | (1 << 5) | (1 << 0);

	while ((SysTick->SR &(1 << 0)) != (1 << 0))
	{
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);


		float B = I_ins();
		float A = V_ins();








		Vsum_A = Vsum_A + (B*A);
		measurements_count_A++;


	}

	SysTick->CTLR &= ~(1 << 0);

	float PA_ = (Vsum_A / measurements_count_A) + OFFESET_W;

	return PA_;


}
