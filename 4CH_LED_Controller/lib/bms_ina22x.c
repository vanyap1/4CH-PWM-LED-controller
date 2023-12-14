
/*
 * bms_ina22x.c
 *
 * Created: 16.11.2023 21:05:21
 *  Author: Vanya
 */ 
#include "bms_ina22x.h"
#include "twi_hal1.h"
#include <stdint.h>

uint8_t INA_ADDR = 0;


uint8_t PowerMeterInit(uint8_t addr){
	uint8_t data_reg[2];
	uint16_t config_reg = 0;
	INA_ADDR = addr;
	config_reg |= (SHUNT_AND_BUS_CONTINUOUS << MODE1) | (CONVESION_TIME_8224us << VSHCT0)  | (CONVESION_TIME_8224us << VBUSCT0) | (AVERAGES_4 << AVG0) | (0 << RST);
	data_reg[0] = (config_reg >> 8) & 0xFF;
	data_reg[1] = config_reg & 0xFF;
	return twi_write(INA_ADDR, CONFIG_REG, data_reg, sizeof(data_reg));
	
	uint16_t callibrationReg = 0;
	data_reg[0] = (callibrationReg >> 8) & 0xFF;
	data_reg[1] = callibrationReg & 0xFF;
	return twi_write(INA_ADDR, CALIBRATION_REG, data_reg, sizeof(data_reg));
}


uint16_t PowerMeterGetId(){
	
	return PowerMeterGetReg(MANUFACTURER_ID_REG);
}

uint16_t PowerMeterGetReg(uint8_t reg_addr){
	uint8_t reg_raw[2] = {0, 0};	//Read one 16 bit word
	twi_read(INA_ADDR, reg_addr, reg_raw, sizeof(reg_raw));
	return (reg_raw[0] << 8) | reg_raw[1];
}

uint8_t PowerMeterMeasure(powerData * pmStruc){
	float milliAmptVal = (PowerMeterGetReg(SHUNT_VOLTAGE_REG)*CURRENT_MULTIPLIER)+CURRENT_OFFSET;
	float milliVoltVal = (PowerMeterGetReg(BUS_VOLTAGE_REG)*VOLTAGE_MULTIPLIER)+VOLTAGE_OFFSET;
	pmStruc->voltage = (uint16_t)milliVoltVal;
	pmStruc->current = (int16_t)milliAmptVal;
	pmStruc->power = (milliAmptVal*milliVoltVal)/1000000;
	pmStruc->energy=pmStruc->energy+pmStruc->power/3600;
	return 1;
}