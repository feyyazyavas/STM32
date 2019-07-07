#include "ds18b20.h"

ds18b20::ds18b20()
{	
}

ds18b20::~ds18b20()
{	
}

void ds18b20::Start(unsigned short int _sensorPin)
{
	sensorPin = _sensorPin;
	GpioSetClock();
	check = 2;
}

void ds18b20::Task()
{
	Ds18b20GetTemperature();
	FanControl();
}

void ds18b20::GpioSetClock()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
}

void ds18b20::GpioSetInput()
{
	GpioInitStructure.Pin = sensorPin;
	GpioInitStructure.Mode = GPIO_MODE_INPUT;
	GpioInitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DS18B20_PORT, &GpioInitStructure);
}

void ds18b20::GpioSetOutput()
{
	GpioInitStructure.Pin = sensorPin;
	GpioInitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GpioInitStructure.Pull = GPIO_NOPULL;
  GpioInitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DS18B20_PORT, &GpioInitStructure);
}

void ds18b20::Delay_us(unsigned int microseconds)
{
	unsigned int delay = microseconds * 7;
	while(delay--)
	{
	}
}

int ds18b20::Ds18b20Init()
{
	GpioSetOutput();
	HAL_GPIO_WritePin(DS18B20_PORT, sensorPin, GPIO_PIN_RESET);
	Delay_us(480);
	GpioSetInput();
	Delay_us(80);
	if(!(HAL_GPIO_ReadPin(DS18B20_PORT, sensorPin)))
	{
		Delay_us(400);
		return 0;
	}
	else
	{
		Delay_us(400);
		return 1;
	}
}

void ds18b20::Ds18b20Write(unsigned int data)
{
	GpioSetOutput();
	for(int i=0; i<8; i++)
	{
		if((data & (1<<i)) != 0)
		{
			GpioSetOutput();
			HAL_GPIO_WritePin(DS18B20_PORT, sensorPin, GPIO_PIN_RESET);
			Delay_us(2);
			GpioSetInput();
			Delay_us(60);
		}
		else
		{
			GpioSetOutput();
			HAL_GPIO_WritePin(DS18B20_PORT, sensorPin, GPIO_PIN_RESET);
			Delay_us(60);
			GpioSetInput();
		}
	}
}

int ds18b20::Ds18b20Read()
{
	unsigned int value = 0;
	GpioSetInput();
	for(int i=0; i<8; i++)
	{
		GpioSetOutput();
		HAL_GPIO_WritePin(DS18B20_PORT, sensorPin, GPIO_PIN_RESET);
		Delay_us(2);
		GpioSetInput();
		if(HAL_GPIO_ReadPin(DS18B20_PORT, sensorPin))
		{
			value |= 1<<i;
		}
		Delay_us(60);
	}
	return value;
}

float ds18b20::Ds18b20GetTemperature()
{
	check = Ds18b20Init();
	Ds18b20Write(0xCC);
	Ds18b20Write(0xCC);
	
	Ds18b20Init();
	Ds18b20Write(0xCC);
	Ds18b20Write(0xCC);
	
	uint8_t temp_l = Ds18b20Read();
	uint8_t temp_h = Ds18b20Read();
	uint8_t temp = (temp_h<<8)|temp_l;
	temperature = (float)temp/16;
	return temperature;
}

void ds18b20::FanControl()
{
	if(temperature >= SetMaxTemperature)
	{
		fanPosition = true;
	}
	else if(fanPosition == true && temperature <= SetMinTemperature)
	{
		fanPosition = false;
	}
}
