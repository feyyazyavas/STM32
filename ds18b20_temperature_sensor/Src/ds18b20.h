#ifndef __DS18B20_H
#define __DS18B20_H

#include "stm32f0xx_hal.h"
#include "stdint.h"

#define DS18B20_PORT GPIOA
#define DS18B20_SENSOR1 GPIO_PIN_0
#define DS18B20_SENSOR2 GPIO_PIN_1
#define SetMaxTemperature 25
#define SetMinTemperature 22

class ds18b20
{
	private:
		float temperature;
		unsigned char check;
		unsigned short int sensorPin;
		GPIO_InitTypeDef GpioInitStructure;
	
	public:
		bool fanPosition;
	
	private:
		void GpioSetClock();
		void GpioSetInput();
		void GpioSetOutput();
		void Delay_us(unsigned int microseconds);
		int Ds18b20Init();
		void Ds18b20Write(unsigned int data);
		int Ds18b20Read();
		float Ds18b20GetTemperature();
		void FanControl();
	public:
		ds18b20();
		~ds18b20();
	
		void Start(unsigned short int _sensorPin);
		void Task();
	};



#endif
