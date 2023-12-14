/*
 * adc.c
 *
 * Created: 03.02.2023 19:01:00
 *  Author: User
 */ 
#include "adc_hal.h"

volatile static uint8_t adc_convert_done = 1;
const float R1 = 10000.0;
const float Vcc = 3.3;
const int ADCResolution = 1024;


ISR(ADC_vect){
	adc_convert_done = 1;
}

void adc_init(void){
	ADMUX |= REFS_AVCC << REFS1 | 1 << REFS0;
	ADCSRA |= 1 << ADEN | 1 << ADIE | 0b111 << ADPS0;
}


void adc_pin_enable(uint8_t pin){
	DIDR0 |= 1 << pin;
}

void adc_pin_disable(uint8_t pin){
	DIDR0 &= ~(1 << pin);
}

void adc_pin_select(uint8_t souce){
	ADMUX &= 0xF0;
	ADMUX |= souce;
}


uint16_t adc_convert(void){
	ADCSRA |= 1 << ADEN;
	uint8_t adcl = 0;
	uint8_t adch = 0;
	
	adc_convert_done = 0;
	
	ADCSRA |= 1 << ADSC;
	while(adc_convert_done == 0);
	
	adcl = ADCL;
	adch = ADCH;
	ADCSRA &=~ (1 << ADEN);
	return (adch << 8 | adcl);
	
	
}

uint16_t get_adc(uint8_t source){
	adc_pin_select(source);
	//uint16_t adc_tmp_data = adc_convert();
	return adc_convert();
}

uint16_t get_mVolt(uint8_t source){
	adc_pin_select(source);
	//uint16_t adc_tmp_data = adc_convert();
	return ADC_VOLT(adc_convert());
}

uint16_t get_mAmps(uint8_t source){
	adc_pin_select(source);
	//uint16_t adc_tmp_data = adc_convert();
	return ADC_AMPS(adc_convert());
}

int16_t getNTC(uint8_t thermalZone){
	return ADCtoCelsius(get_adc(thermalZone));
}

int16_t ADCtoCelsius(uint16_t adcValue) {

	float R2 = R1 / ((ADCResolution - adcValue) / (float)adcValue);
	float temp = 1.0 / ((log(R2 / R1) / 3950.0) + (1.0 / 298.15)) - 273.15;
	return (int16_t)(temp * 10);
}