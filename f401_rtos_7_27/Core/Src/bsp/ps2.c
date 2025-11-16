#include "PS2.h"
#include "spi.h"


uint8_t CMD[3] = { 0x01,0x42,0x00 };
uint8_t PS2OriginalValue[9] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
uint8_t RockerValue[4] = { 0x00,0x00,0x00,0x00 };
uint8_t ButtonValue[16] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
uint8_t i,mode;

void Delay_us(uint32_t udelay)
{
	uint32_t startval, tickn, delays, wait;

	startval = SysTick->VAL;
	tickn = HAL_GetTick();
	delays = udelay * 72;
	if (delays > startval)
	{
		while (HAL_GetTick() == tickn)
		{

		}
		wait = 72000 + startval - delays;
		while (wait < SysTick->VAL)
		{

		}
	}
	else
	{
		wait = startval - delays;
		while (wait < SysTick->VAL && HAL_GetTick() == tickn)
		{

		}
	}
}

void PS2OriginalValueGet(void) 
{
	short i = 0;

	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, &CMD[0], &PS2OriginalValue[0], 1, HAL_MAX_DELAY);
	Delay_us(10);
	HAL_SPI_TransmitReceive(&hspi1, &CMD[1], &PS2OriginalValue[1], 1, HAL_MAX_DELAY);
	Delay_us(10);
	HAL_SPI_TransmitReceive(&hspi1, &CMD[2], &PS2OriginalValue[2], 1, HAL_MAX_DELAY);
	Delay_us(10);
	for (i = 3; i < 9; i++)
	{
		HAL_SPI_TransmitReceive(&hspi1, &CMD[2], &PS2OriginalValue[i], 1, HAL_MAX_DELAY);
		Delay_us(10);
	}
	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_SET);

}


void PS2AllValueUpdate(void)
{
	PS2OriginalValueGet(); 
	RockerValueGet();
	ButtonValueGet();
	mode = PS2OriginalValue[1];
	PS2OriginalValueClear();
}

void RockerValueGet(void)
{
	int i;
	for (i = 5; i < 9; i++)
	{
		PS2OriginalValue[i] = (int)PS2OriginalValue[i];
		RockerValue[i - 5] = PS2OriginalValue[i];
	}
}

void PS2OriginalValueClear(void)
{
	for (i = 0; i < 9; i++)
	{
		PS2OriginalValue[i] = 0x00; 
	}
}

void ButtonValueGet(void)
{
	uint8_t bit = 1;
	uint8_t button = 0;
	for (bit = 8; bit > 0; bit--)
	{
		bit -= 1;
		ButtonValue[button] = (PS2OriginalValue[3] & (1 << bit)) >> bit;
		bit += 1;
		button++;
	}
	for (bit = 8; bit > 0; bit--)
	{
		bit -= 1;
		ButtonValue[button] = (PS2OriginalValue[4] & (1 << bit)) >> bit;
		bit += 1;
		button++;
	}
	for (button = 0; button < 16; button++)
	{
		if (ButtonValue[button] == 1)  ButtonValue[button] = 0;
		else  ButtonValue[button] = 1;
	}
}

int PS2RedLight(void)
{
	if (mode == 0X73)
		return 1;
	else
		return 0;
}
