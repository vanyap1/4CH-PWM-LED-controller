/*
 * 4CH_LED_Controller.c
 *
 * Created: 14.12.2023 18:12:35
 * Author : Vanya
 */ 

#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "string.h"
#include "stdbool.h"
#include "uart_hal.h"
#include "gpio_driver.h"
#include "twi_hal1.h"
#include "adc_hal.h"
#include "spi1_hall.h"
#include "stdint.h"

#include <stdio.h>
static FILE mystdout = FDEV_SETUP_STREAM((void *)uart_send_byte, NULL, _FDEV_SETUP_WRITE);

gpio drvEn = {(uint8_t *)&PORTD , PORTD7};
gpio ledRun = {(uint8_t *)&PORTE , PORTE3};
gpio ledFail = {(uint8_t *)&PORTC , PORTC0};

gpio ch1Config = {(uint8_t *)&PORTB , PORTB2};
gpio ch2Config = {(uint8_t *)&PORTB , PORTB3};
gpio ch3Config = {(uint8_t *)&PORTD , PORTD6};
gpio ch4Config = {(uint8_t *)&PORTD , PORTD5};

#define	CH1					OCR1B 	
#define CH2					OCR2A
#define CH3					OCR0A		
#define CH4					OCR0B

#define VOLTMONITOR1		4	//input voltage channel
#define VOLTMONITOR2		5	//FET driver voltage
#define CURRMONITOR			3	//output current
#define NTC1				2	//onboard ntc
#define NTC2				1	//external ntc

#define CRITICALVOLTAGE		10500
#define WORKRESTOREVOLTAGE	11000
#define LIGHTTARGETLEVEL	255
#define INCRSTEPTIME		20


uint16_t inputVoltage = 0;
uint16_t fetVoltage = 0;
uint16_t outputCurrent = 0;
int16_t ntc1Celsius = 0;
int16_t ntc2Celsius = 0;
uint8_t	lightLevel = 0;
uint8_t protectionFlg = 0;
	
	
int main(void)
{
	//uart_init(250000, 1);
	twi1_init(400000);
	
	gpio_set_pin_direction(&drvEn , PORT_DIR_OUT); gpio_set_pin_level(&drvEn, true);
	gpio_set_pin_direction(&ledRun , PORT_DIR_OUT); gpio_set_pin_level(&ledRun, false);
	gpio_set_pin_direction(&ledFail , PORT_DIR_OUT); gpio_set_pin_level(&ledFail, false);
	
	gpio_set_pin_direction(&ch1Config , PORT_DIR_OUT); gpio_set_pin_level(&ch1Config, false);
	gpio_set_pin_direction(&ch2Config , PORT_DIR_OUT); gpio_set_pin_level(&ch2Config, false);
	gpio_set_pin_direction(&ch3Config , PORT_DIR_OUT); gpio_set_pin_level(&ch3Config, false);
	gpio_set_pin_direction(&ch4Config , PORT_DIR_OUT); gpio_set_pin_level(&ch4Config, false);
    
	sei();
	
	adc_init();
	
	//Set channels default values
	CH1 = 0;
	CH2 = 0;
	CH3 = 0;
	CH4 = 0;

	//Setup timer 0, CH A and B for out 3 and 4
	TCCR0A = (0b10 << COM0A0) | (0b10 << COM0B0) | (0b01 << WGM00);
	TCCR0B = (0b0 << WGM02) | (0b1 << CS10);
	
	//Setup timer 1, CH B for out 1
	TCCR1A = (0b10 << COM1B0) | (0b01 << WGM00);
	TCCR1B = (0b0 << WGM02) | (0b1 << CS10);
	
	//Setup timer 2, CH A for out 2
	TCCR2A = (0b10 << COM2A0) | (0b01 << WGM00);
	TCCR2B = (0b0 << WGM02) | (0b1 << CS10);

    while (1) 
    {
		ntc1Celsius = getNTC(NTC1);
		inputVoltage = get_mVolt(VOLTMONITOR1);
		outputCurrent = get_mAmps(CURRMONITOR);
		
		
		if (inputVoltage <= CRITICALVOLTAGE){
			protectionFlg = 1;
		}
		if (inputVoltage >= WORKRESTOREVOLTAGE){
			protectionFlg = 0;
		}
		
		if(protectionFlg != 0){
			gpio_set_pin_level(&drvEn, true);
			gpio_toggle_pin_level(&ledFail);
			
			lightLevel=0;
			_delay_ms(250);
		}else{
			gpio_set_pin_level(&drvEn, false);
			gpio_set_pin_level(&ledFail, false);
			
			if(lightLevel < LIGHTTARGETLEVEL){
				lightLevel++;
				_delay_ms(INCRSTEPTIME);
			}
			CH1 = lightLevel;
			CH2 = lightLevel;
			CH3 = lightLevel;
			CH4 = lightLevel;
			
			
		}
		
		
		
		
		
		
		
		
		
		
		
    }
}

