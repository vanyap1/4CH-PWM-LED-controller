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
#include <avr/wdt.h>

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

gpio sData = {(uint8_t *)&PORTE , PORTE0};
gpio sClock = {(uint8_t *)&PORTE , PORTE1};



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

#define TWIADDR				0x54
#define TWISPEED			400000
uint8_t twiDataBuff[64];
uint8_t	pwmChCalcReg[4];

uint8_t TAB_PWM_1[256] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 1 , 1 , 1 , 1,
	1 , 2 , 2 , 2 , 2 , 3 , 3 , 3 , 3 , 4 , 4 , 4 , 5 , 5 , 5 , 6,
	6 , 6 , 7 , 7 , 7 , 8 , 8 , 9 , 9 , 9 , 10 , 10 , 11 , 11 , 12 , 12,
	13 , 13 , 13 , 14 , 14 , 15 , 16 , 16 , 17 , 17 , 18 , 18 , 19 , 19 , 20 , 21,
	21 , 22 , 22 , 23 , 24 , 24 , 25 , 25 , 26 , 27 , 27 , 28 , 29 , 30 , 30 , 31,
	32 , 32 , 33 , 34 , 35 , 35 , 36 , 37 , 38 , 38 , 39 , 40 , 41 , 42 , 42 , 43,
	44 , 45 , 46 , 46 , 47 , 48 , 49 , 50 , 51 , 52 , 53 , 53 , 54 , 55 , 56 , 57,
	58 , 59 , 60 , 61 , 62 , 63 , 64 , 65 , 66 , 67 , 68 , 69 , 70 , 71 , 72 , 73,
	74 , 75 , 76 , 77 , 78 , 79 , 80 , 81 , 82 , 83 , 84 , 86 , 87 , 88 , 89 , 90,
	91 , 92 , 93 , 95 , 96 , 97 , 98 , 99 , 100 , 102 , 103 , 104 , 105 , 106 , 108 , 109,
	110 , 111 , 113 , 114 , 115 , 116 , 118 , 119 , 120 , 122 , 123 , 124 , 125 , 127 , 128 , 129,
	131 , 132 , 133 , 135 , 136 , 137 , 139 , 140 , 142 , 143 , 144 , 146 , 147 , 149 , 150 , 151,
	153 , 154 , 156 , 157 , 159 , 160 , 162 , 163 , 164 , 166 , 167 , 169 , 170 , 172 , 173 , 175,
	176 , 178 , 180 , 181 , 183 , 184 , 186 , 187 , 189 , 190 , 192 , 194 , 195 , 197 , 198 , 200,
	202 , 203 , 205 , 207 , 208 , 210 , 211 , 213 , 215 , 216 , 218 , 220 , 221 , 223 , 225 , 227,
228 , 230 , 232 , 233 , 235 , 237 , 239 , 240 , 242 , 244 , 246 , 247 , 249 , 251 , 253 , 255 };


uint16_t inputVoltage = 0;
uint16_t fetVoltage = 0;
uint16_t outputCurrent = 0;
int16_t ntc1Celsius = 0;
int16_t ntc2Celsius = 0;
uint8_t	lightLevel = 0;
uint8_t protectionFlg = 0;
	
	
	
int main(void)
{
	sei();
	wdt_enable(WDTO_2S);
	//uart_init(250000, 1);
	//twi1_init(400000);
	twi1_slave_init(TWIADDR, &twiDataBuff, TWISPEED);
	
	gpio_set_pin_direction(&sData , PORT_DIR_IN);
	gpio_set_pin_direction(&sClock , PORT_DIR_IN);
	gpio_set_pin_direction(&drvEn , PORT_DIR_OUT); gpio_set_pin_level(&drvEn, true);
	gpio_set_pin_direction(&ledRun , PORT_DIR_OUT); gpio_set_pin_level(&ledRun, false);
	gpio_set_pin_direction(&ledFail , PORT_DIR_OUT); gpio_set_pin_level(&ledFail, true);
	
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
	
	
	_delay_ms(100);
	gpio_set_pin_level(&ledFail, false);
    while (1) 
    {
		wdt_reset();
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
			gpio_set_pin_level(&ledRun, false);
			gpio_toggle_pin_level(&ledFail);
			
			
			lightLevel=0;
			_delay_ms(250);
		}else{
			gpio_set_pin_level(&drvEn, false);
			gpio_set_pin_level(&ledFail, false);
			gpio_set_pin_level(&ledRun, true);
			
			if(lightLevel < LIGHTTARGETLEVEL){
				lightLevel++;
				_delay_ms(INCRSTEPTIME);
			}
			
			
			for (uint8_t ch = 0; ch<=sizeof(pwmChCalcReg); ch++)
			{
				if(pwmChCalcReg[ch] < twiDataBuff[ch]){
					if(pwmChCalcReg[ch]!= 255){
						pwmChCalcReg[ch]++;
					}
				}
				else{
					if(pwmChCalcReg[ch]!= 0){
						pwmChCalcReg[ch]--;
					}
				}
			}
			CH1 = TAB_PWM_1[pwmChCalcReg[0]];
			CH2 = TAB_PWM_1[pwmChCalcReg[1]];
			CH3 = TAB_PWM_1[pwmChCalcReg[2]];
			CH4 = TAB_PWM_1[pwmChCalcReg[3]];
			_delay_ms(1);
			
		}
		
		
		
		
		
		
		
		
		
		
		
    }
}

