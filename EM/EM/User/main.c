/**********************************(C) COPYRIGHT *******************************

*File Name          : main.c
*Author             : Muhammad_Minhaj
*Version            : V1.0.0
*Date               : 2021/06/06
*Description        : Main program body.

*******************************************************************************/


/**********************

Credit and thanks to

https://github.com/olikraus/u8g2 for OLED library

https://github.com/mhtb32/EfficientMovingAverage/blob/master/moving_average.c for moving average library



**********************/


/*

I2C CH_1
Pins
I2C1_SCL(PB8)¡¢
I2C1_SDA(PB9)¡£

*/
#include <em.h>
#include "debug.h"
#include "u8g2.h"


short int UNDER_VOL_FLAG;
short int OVER_VOL_FLAG;

FilterTypeDef filterStruct_V, filterStruct_I;

/*I2C Mode Definition */
#define HOST_MODE 0
#define SLAVE_MODE 1

/*I2C Communication Mode Selection */
#define I2C_MODE HOST_MODE
//#define I2C_MODE   SLAVE_MODE

/*Global define */
#define Size 7
#define OLED_ADDRESS 0x3C << 1
#define TX_TIMEOUT 100

u32 TxBuf[1];
u16 Adc_Val[4];
s16 Calibrattion_Val1 = 0;
s16 Calibrattion_Val2 = 0;

void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

u8 i;
//u64 milli;
u8 a = 0;

float vol;
float cur;
float pf;
float pw;
u32 Wh;

char vol_buffer[10];
char cur_buffer[10];
char pf_buffer[10];
char pow_rms_buffer[10];
char pow_av_buffer[10];
char kWh_buffer[10];
char bs[10];
//RTC

void BKP_Tamper_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	BKP_TamperPinCmd(DISABLE);
	PWR_BackupAccessCmd(ENABLE);
	BKP_ClearFlag();
}

void TIM3_Int_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//Clock enable

	//Timer TIM3 initialization
	TIM_TimeBaseStructure.TIM_Period = arr;	//Set the value of the active auto-reload register period loaded on the next update event
	TIM_TimeBaseStructure.TIM_Prescaler = psc;	//Set the prescaler value used as the divisor of the TIMx clock frequency
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//Set the clock division: TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM count up mode
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	//Initialize the time base unit of TIMx according to the specified parameters

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);	//Enable the specified TIM3 interrupt, allow update interrupt

	TIM_Cmd(TIM3, ENABLE);	//Enable TIMx
}

void GPIO_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = { 0 };

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void IIC_Init(u32 bound, u16 address)
{
	GPIO_InitTypeDef GPIO_InitStructure = { 0 };
	I2C_InitTypeDef I2C_InitTSturcture = { 0 };

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitTSturcture.I2C_ClockSpeed = bound;
	I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
	I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
	I2C_InitTSturcture.I2C_OwnAddress1 = address;
	I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
	I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &I2C_InitTSturcture);

	I2C_Cmd(I2C1, ENABLE);

	#if (I2C_MODE == HOST_MODE)
		I2C_AcknowledgeConfig(I2C1, ENABLE);

	#endif
}

uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
	switch (msg)
	{
		case U8X8_MSG_GPIO_AND_DELAY_INIT:	// called once during init phase of u8g2/u8x8

			Delay_Init();
			break;	// can be used to setup pins
		case U8X8_MSG_DELAY_100NANO:	// delay arg_int *100 nano seconds
			for (uint16_t n = 0; arg_int *n < 20; n++)
			{
				__NOP();
			}

			break;
		case U8X8_MSG_DELAY_10MICRO:	// delay arg_int *10 micro seconds
			for (uint16_t n = 0; n < arg_int *2000; n++)
			{
				__NOP();
			}

			break;
		case U8X8_MSG_DELAY_MILLI:	// delay arg_int *1 milli second
			Delay_Ms(arg_int);

			break;

		default:
			return 0;
	}

	return 1;
}

void I2C_Master_Transmit(I2C_TypeDef *I2Cx, uint16_t slave_add, uint8_t *buff, uint8_t idx)
{
	IIC_Init(100000, slave_add);

	u8 i = 0;

	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) != RESET);
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2Cx, slave_add, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	while (idx > 0)
	{
		if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE) != RESET)
		{
			I2C_SendData(I2Cx, buff[i]);
			idx--;
			i++;
		}
	}

	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(I2Cx, ENABLE);

}

uint8_t u8x8_byte_stm32_hw_i2c(u8x8_t *u8g2, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	// u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER
	//	 add extra byte for the i2c address
	static uint8_t buffer[34];
	static uint8_t buf_idx;
	uint8_t * data;
	switch (msg)
	{
		case U8X8_MSG_BYTE_SEND:
			data = (uint8_t*) arg_ptr;
			while (arg_int > 0)
			{
				buffer[buf_idx++] = *data;
				data++;
				arg_int--;
			}

			break;
		case U8X8_MSG_BYTE_INIT:
			// add your custom code to init i2c subsystem
			break;
		case U8X8_MSG_BYTE_SET_DC:
			//	 ignored for i2c
			break;
		case U8X8_MSG_BYTE_START_TRANSFER:
			buf_idx = 0;
			break;
		case U8X8_MSG_BYTE_END_TRANSFER:
			I2C_Master_Transmit(I2C1, OLED_ADDRESS, (uint8_t*) buffer, buf_idx);
			break;
		default:
			return 0;
	}

	return 1;
}

u8g2_t u8g2;

void ADC_Function_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure = { 0 };
	GPIO_InitTypeDef GPIO_InitStructure = { 0 };

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div4);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_DeInit(ADC1);
	ADC_DeInit(ADC2);

	ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_InitStructure.ADC_OutputBuffer = ADC_OutputBuffer_Disable;
	ADC_InitStructure.ADC_Pga = ADC_Pga_1;

	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5);

	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	ADC_BufferCmd(ADC1, DISABLE);	//disable buffer
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
	Calibrattion_Val1 = Get_CalibrationValue(ADC1);

	ADC_BufferCmd(ADC1, ENABLE);	//enable buffer

	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 1, ADC_SampleTime_239Cycles5);

	ADC_SoftwareStartConvCmd(ADC2, ENABLE);
	ADC_Cmd(ADC2, ENABLE);

	ADC_BufferCmd(ADC2, DISABLE);	//disable buffer
	ADC_ResetCalibration(ADC2);
	while (ADC_GetResetCalibrationStatus(ADC2));
	ADC_StartCalibration(ADC2);
	while (ADC_GetCalibrationStatus(ADC2));
	Calibrattion_Val2 = Get_CalibrationValue(ADC2);

	ADC_BufferCmd(ADC2, ENABLE);	//enable buffer
}

void DMA_Tx_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize)
{
	DMA_InitTypeDef DMA_InitStructure = { 0 };
	NVIC_InitTypeDef NVIC_InitStructure = { 0 };

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(DMA_CHx);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
	DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = bufsize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA_CHx, &DMA_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE, ENABLE);
}

u16 Get_ConversionVal1(s16 val)
{
	if ((val + Calibrattion_Val1) < 0) return 0;
	if ((Calibrattion_Val1 + val) > 4095 || val == 4095) return 4095;
	return (val + Calibrattion_Val1);
}

u16 Get_ConversionVal2(s16 val)
{
	if ((val + Calibrattion_Val2) < 0) return 0;
	if ((Calibrattion_Val2 + val) > 4095 || val == 4095) return 4095;
	return (val + Calibrattion_Val2);
}

int main(void)
{
	Delay_Init();

	USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n", SystemCoreClock);
	#if (I2C_MODE == HOST_MODE)
		u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_stm32_hw_i2c, u8g2_gpio_and_delay_stm32);

	u8g2_InitDisplay(&u8g2);	// send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&u8g2, 0);
	u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_courB14_tf);

	ADC_Function_Init();

	DMA_Tx_Init(DMA1_Channel1, (u32) &ADC1->RDATAR, (u32) TxBuf, 1);
	DMA_Cmd(DMA1_Channel1, ENABLE);

	GPIO_INIT();
	GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//Set NVIC interrupt group 2: 2-bit preemption priority, 2-bit response priority

	TIM3_Int_Init(20000 - 1, 7200 - 1);
	BKP_Tamper_Init();

	Wh = BKP->DATAR1;

	// beef the buzzer

	GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
	for (int j = 0; j <= 3220; j++)

	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
		Delay_Us(244);

		GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
		Delay_Us(244);
	}

	// Wait for avoiding start voltage fluctuation
	for (int m = START_DELAY; m >= 0; m--)
	{
		u8g2_DrawStr(&u8g2, 23, 25, "Starting");

		sprintf(bs, "%0d", m);

		u8g2_DrawStr(&u8g2, 57, 50, bs);

		Delay_Ms(1000);

		u8g2_SendBuffer(&u8g2);
		u8g2_ClearBuffer(&u8g2);
	}

	// Set relay ON to allow electricity
	GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);

	#elif(I2C_MODE == SLAVE_MODE)
	# endif

	while (1)
	{
		// adc conversion ON
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);

		// store kWh reading in back after each 2 seconds

		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)	//Check whether the TIM3 update interrupt occurs or not
		{
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	//Clear the TIMx update interrupt flag

			Wh = Wh + P_avg() / 1800;

			BKP_WriteBackupRegister(BKP_DR1, Wh);

		}

		// Draw plotter for serial serial plotter
		plotter();
		//Read functions
		// Put max and min limits for diaplay

		float Ir = I_rms();
		if (Ir > 99.0)
			Ir = 99.0;
		if (Ir < 0.0)
			Ir = 0;

		float Vr = V_rms();
		if (Vr > 999.0)
			Vr = 999.0;
		if (Vr < 0.0)
			Vr = 0;

		float kPr = (Vr *Ir) / 1000.0;

		if (kPr > 99.0)
			kPr = 99.0;
		if (kPr < 0.0)
			kPr = 0;

		float kPa = P_avg() / 1000.0;
		if (kPa > 99.0)
			kPa = 99.0;
		if (kPa < 0.0)
			kPa =  -1*kPa;

		float pf = kPa / kPr;
		if (pf > 1.0)
			pf = 1.0;
		if (pf < 0.0)
			pf = 0;

		float kWh = Wh / 1000.0;
		if (kWh > 9999.0)
			kWh = 0;

		// convert float, int to string for display and format
		sprintf(vol_buffer, "%-4.0f", Vr);

		sprintf(cur_buffer, "%-3.2f", Ir);
		sprintf(pf_buffer, "%-3.1f", pf);
		sprintf(pow_rms_buffer, "%-3.2f", kPr);
		sprintf(pow_av_buffer, "%-3.2f", kPa);

		if (kWh < 9.9)

			sprintf(kWh_buffer, "%-4.2f", kWh);
		else if (kWh < 99.9)
			sprintf(kWh_buffer, "%-4.1f", kWh);
		else
			sprintf(kWh_buffer, "%-4.0f", kWh);

		//Display at proper position

		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_crox1c_tr);

		u8g2_DrawStr(&u8g2, 42, 15, "A");
		u8g2_DrawStr(&u8g2, 42, 30, "kVA");
		u8g2_DrawStr(&u8g2, 42, 45, "kWh");

		u8g2_DrawStr(&u8g2, 108, 15, "V");
		u8g2_DrawStr(&u8g2, 108, 30, "kW");

		u8g2_DrawStr(&u8g2, 108, 45, "pf");

		u8g2_SetFont(&u8g2, u8g2_font_t0_16b_mn);

		u8g2_DrawStr(&u8g2, 5, 15, cur_buffer);
		u8g2_DrawStr(&u8g2, 75, 15, vol_buffer);
		u8g2_DrawStr(&u8g2, 5, 30, pow_rms_buffer);
		u8g2_DrawStr(&u8g2, 75, 30, pow_av_buffer);

		u8g2_DrawStr(&u8g2, 5, 45, kWh_buffer);
		u8g2_DrawStr(&u8g2, 75, 45, pf_buffer);

		u8g2_SetFont(&u8g2, u8g2_font_crox1c_tr);

		// Check Cutoff limits for voltage

		if (Vr < UNDER_VOLT)
		{
			GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);

			u8g2_DrawStr(&u8g2, 30, 60, "Low Volt");

			UNDER_VOL_FLAG = 1;

		}
		else if (Vr > OVER_VOLT)

		{
			GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);

			u8g2_DrawStr(&u8g2, 30, 60, "High Volt");

			OVER_VOL_FLAG = 1;

		}
		else
		{
			if (UNDER_VOL_FLAG == 1)
			{
				if (Vr > UNDER_VOLT + HYST)
				{
					GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);

					u8g2_DrawStr(&u8g2, 35, 60, "Normal");

					UNDER_VOL_FLAG = 0;
				}
				else
				{
					GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);

					u8g2_DrawStr(&u8g2, 30, 60, "Low Volt");

				}
			}
			else if (OVER_VOL_FLAG == 1)
			{
				if (Vr < (OVER_VOLT - HYST))
				{
					GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);

					u8g2_DrawStr(&u8g2, 35, 60, "Normal");

					OVER_VOL_FLAG = 0;
				}
				else
				{
					GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);

					u8g2_DrawStr(&u8g2, 30, 60, "High Volt");

				}
			}
			else
			{
				GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);

				u8g2_DrawStr(&u8g2, 35, 60, "Normal");

			}
		}

		//I_rms=16;

		if (Ir > MAX_LOAD)
		{
			GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
			int p = LOAD_RESTART_TIME;
			while (1)

			{
				p--;
				u8g2_ClearBuffer(&u8g2);
				u8g2_DrawStr(&u8g2, 26, 15, "High Load");
				u8g2_DrawStr(&u8g2, 26, 30, "Reduce Load");
				u8g2_DrawStr(&u8g2, 26, 45, "Restarting");
				sprintf(bs, "%0d", p);

				u8g2_DrawStr(&u8g2, 64, 60, bs);

				for (int j = 0; j <= 1000; j++)

				{
					GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
					Delay_Us(244);

					GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
					Delay_Us(244);

				}

				Delay_Ms(1000);

				u8g2_SendBuffer(&u8g2);

				if (p <= 0)
					NVIC_SystemReset();

			}
		}

		u8g2_SendBuffer(&u8g2);

	}
}

void DMA1_Channel1_IRQHandler()
{
	if (DMA_GetITStatus(DMA1_IT_TC1) == SET)
	{
		DMA_ClearITPendingBit(DMA1_IT_GL1);

		Adc_Val[0] = TxBuf[0] &0xffff;
		Adc_Val[1] = (TxBuf[0] >> 16) &0xffff;

		#if 0
		printf("ADC1 ch2=%d\r\n", Get_ConversionVal1(Adc_Val[0]));
		printf("ADC2 ch3=%d\r\n", Get_ConversionVal2(Adc_Val[1]));
		#endif

	}
}
