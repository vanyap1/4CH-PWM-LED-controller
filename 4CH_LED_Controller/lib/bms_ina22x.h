
/*
 * bms_ina22x.h
 *
 * Created: 16.11.2023 21:05:27
 *  Author: Vanya
 */ 
#include <avr/io.h>
#include "string.h"
#include "stdbool.h"

#ifndef BMS_H_
#define BMS_H_
	
//Calibration multipliers and offset
#define VOLTAGE_MULTIPLIER		1.2288031
#define VOLTAGE_OFFSET			0
#define CURRENT_MULTIPLIER		0.8514162
#define CURRENT_OFFSET			0
#define POWER_MULTIPLIER		1
#define POWER_OFFSET			0





#define CONFIG_REG					0x00
#define SHUNT_VOLTAGE_REG			0x01
#define BUS_VOLTAGE_REG				0x02
#define POWER_REG					0x03
#define CURRENT_REG					0x04
#define CALIBRATION_REG				0x05
#define MASK_ENABLE_REG				0x06
#define ALERT_LIMIT_REG				0x07
#define MANUFACTURER_ID_REG			0xFE
#define DIE_ID_REG					0xFF


//Config bits definition
#define MODE1	0
#define MODE2	1
#define MODE3	2
#define VSHCT0	3
#define VSHCT1	4
#define VSHCT2	5
#define VBUSCT0	6
#define VBUSCT1	7
#define VBUSCT2	8
#define AVG0	9
#define AVG1	10
#define AVG2	11
#define RST		15


//Averaging Mode
//Determines the number of samples that are collected and averaged. Table 6 shows all the AVG bit settings and
//related number of averages for each bit setting.
#define AVERAGES_1			0
#define AVERAGES_4			1
#define AVERAGES_16			2
#define AVERAGES_64			3
#define AVERAGES_128		4
#define AVERAGES_256		5
#define AVERAGES_512		6
#define AVERAGES_1024		7

//Bus Voltage Conversion Time
//Sets the conversion time for the bus voltage measurement. Table 7 shows the VBUSCT bit options and related
//conversion times for each bit setting.
//Related also to Shunt voltage

#define CONVESION_TIME_140us		0
#define CONVESION_TIME_204us		1
#define CONVESION_TIME_332us		2
#define CONVESION_TIME_588us		3
#define CONVESION_TIME_1100us		4 //Default value after reset
#define CONVESION_TIME_2116us		5
#define CONVESION_TIME_4156us		6
#define CONVESION_TIME_8224us		7


//Operating Mode
//Selects continuous, triggered, or power-down mode of operation. These bits default to continuous shunt and bus
//measurement mode. The mode settings are shown in Table 9.

#define POWERDOWN					0
#define SHUNT_V_TRIG				1
#define BUS_V_TRIG					2
#define SHUNT_AND_BUS_TRIG			3
//#define POWERDOWN					4 
#define SHUNT_V_CONTINUOUS			5
#define BUS_V_CONTINUOUS			6
#define SHUNT_AND_BUS_CONTINUOUS	7 //Default value after reset

//ALert Table 15. Mask/Enable Register (06h) (Read/Write)


#define LEN				0	//Alert Latch Enable; configures the latching feature of the Alert pin and Alert Flag bits.
/*
1 = Latch enabled
0 = Transparent (default)
When the Alert Latch Enable bit is set to Transparent mode, the Alert pin and Flag bit resets to the idle states when
the fault has been cleared. When the Alert Latch Enable bit is set to Latch mode, the Alert pin and Alert Flag bit
remains active following a fault until the Mask/Enable Register has been read.
*/
#define APOL			1	//Alert Polarity bit; sets the Alert pin polarity.
/*
1 = Inverted (active-high open collector)
0 = Normal (active-low open collector) (default)
*/
#define OVF				2	//Math Overflow Flag
/*
This bit is set to '1' if an arithmetic operation resulted in an overflow error. It indicates that current and power data
may be invalid.
*/
#define CVRF			3	//Conversion Ready Flag
/*
Although the device can be read at any time, and the data from the last conversion is available, the Conversion
Ready Flag bit is provided to help coordinate one-shot or triggered conversions. The Conversion Ready Flag bit is
set after all conversions, averaging, and multiplications are complete. Conversion Ready Flag bit clears under the
following conditions:
1.) Writing to the Configuration Register (except for Power-Down selection)
2.) Reading the Mask/Enable Register
*/
#define AFF				4	//Alert Function Flag
/*
While only one Alert Function can be monitored at the Alert pin at a time, the Conversion Ready can also be
enabled to assert the Alert pin. Reading the Alert Function Flag following an alert allows the user to determine if the
Alert Function was the source of the Alert.
When the Alert Latch Enable bit is set to Latch mode, the Alert Function Flag bit clears only when the Mask/Enable
Register is read. When the Alert Latch Enable bit is set to Transparent mode, the Alert Function Flag bit is cleared
following the next conversion that does not result in an Alert condition.
*/
#define CNVR			10	//Conversion Ready
/*
Setting this bit high configures the Alert pin to be asserted when the Conversion Ready Flag, Bit 3, is asserted
indicating that the device is ready for the next conversion.
*/
#define POL				11	//Power Over-Limit
/*
Setting this bit high configures the Alert pin to be asserted if the Power calculation made following a bus voltage
measurement exceeds the value programmed in the Alert Limit Register.
*/
#define BUL				12	//Bus Voltage Under-Voltage
/*
Setting this bit high configures the Alert pin to be asserted if the bus voltage measurement following a conversion
drops below the value programmed in the Alert Limit Register.
*/
#define BOL				13	//Bus Voltage Over-Voltage
/*
Setting this bit high configures the Alert pin to be asserted if the bus voltage measurement following a conversion
exceeds the value programmed in the Alert Limit Register.
*/
#define SUL				14	//Shunt Voltage Under-Voltage
/*
Setting this bit high configures the Alert pin to be asserted if the shunt voltage measurement following a conversion
drops below the value programmed in the Alert Limit Register.
*/
#define SOL				15	//Shunt Voltage Over-Voltage
/*
Setting this bit high configures the Alert pin to be asserted if the shunt voltage measurement following a conversion
exceeds the value programmed in the Alert Limit Register.
*/


typedef struct powerMeterData {
	uint16_t pmId;				//PowerMeter ID and Die Revision ID Bits
	int16_t current;			//current shunt reg
	uint16_t voltage;			//voltage reg
	float power;				//power reg
	float energy;
} powerData;




uint8_t PowerMeterInit(uint8_t addr);
uint16_t PowerMeterGetId();	
uint16_t PowerMeterGetReg(uint8_t reg_addr);
uint8_t PowerMeterMeasure(powerData * pmStruc);






#endif