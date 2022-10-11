#ifndef MAX30102_LIB_H
#define MAX30102_LIB_H

#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>

// ADRESSES
#define MAX30102_WRITE_ADDRESS          0xAE //7-bit I2C write Address
#define MAX30102_READ_ADDRESS          	0xAF //7-bit I2C read Address
#define MAX30102_I2C_ADDR 				0x57 //max address
#define I2C_DELAY						5	//I2C read or write wait time
#define I2C_BUFFER_LENGTH 				32
#define MAX30102_SAMPLE_LEN_MAX 		32  //max sample length

#define INTERRUPT_STATUS_1				0x00 //interrupt status register to show value of each interrupt
#define INTERRUPT_STATUS_2				0x01 // interrupt status 2 registers
#define DIE_TEMPERATURE_CONFIG 			0X21 // enable temperature register for initiating a temperature read
#define DIE_TEMPERATURE_INTEGER			0X1F //contains temperature read integer
#define DIE_TEMPERATURE_FRACTION		0X20 //contains temperature fraction
#define INTERRUPT_ENABLE_1				0x02 //a-full-enable, ppg-rdy-en, alc-ovf-en


#define INTERRUPT_ENABLE_2				0X03 //set register to enable interrupt for taking a temperature measurement
#define FIFO_WRITE_POINTER				0X04 //FIFO write pointer
#define FIFO_READ_POINTER				0X06 //FIFO read pointer
#define OVERFLOW_COUNTER				0X05 //overflow counter register
#define FIFO_READ_POINTER				0X06 // FIFO read pointer
#define	FIFO_DATA_REGISTER				0X07 //FIFO data register
#define FIFO_READ_POINTER				0X06 // FIFO read pointer register
#define FIFO_REGISTER					7 	//FIFO REGISTER

#define FIFO_CONFIGURATION				0X08 //FIFO configuration register

#define MODE_CONFIGURATION 				0X09 //Mode configuration register

#define SPO2_CONFIGURATION 				0X0A //SPO2 configuration register

#define LED1_PA							0X0C //LED pulse amplitude register
#define LED2_PA							0X0D // LED pulse amplitude register

//Important constants
#define enableTempMeasurement 			0X01 		//when written to DIE_TEMPERATURE_CONFIG allows temperature reading to begin
#define dieTempRdyEn					0X02 	  	//when written to INTERRUPT_ENABLE_2 allows interrupt to happen when temp is read complete
#define allInterruptsEnable 			0b11100000 	//when written to INTERRUPT_ENABLE_1 enables all interrupts except when new sample is read
#define defaultLedPulse					0X1F  		//project works with about 51mA on Red and IR led
#define	spo2Mode						0X03 		//SpO2 Mode configuration
#define spo2ModeConfiguration 			0b00100111 // ADC range= 4096nA, SPO2_SAMPLE_RATE = 100, LED_PW=411
#define interruptEnable					0X03 		//enables interrupt
#define fifoConfigurationData 			0b01010001 //SAMPLE_AVERAGE = 4,ROLLOVER_EN = 1, FIFO_A_FULL = 1

///////////////////////////////////////////////Processing///////////////////////////////////////////////////////////////

//max30102 structure
typedef struct max30102 {
	 I2C_HandleTypeDef *_ui2c;
	 float temperature;
	 uint32_t _ir_samples[32];
	 uint32_t _red_samples[32];
	 uint32_t  oxygen;
	 uint32_t  heart_rate;
} max30102;


__weak void max30102_plot(uint32_t ir_sample, uint32_t red_sample);

void max30102_init(max30102 *obj, I2C_HandleTypeDef *hi2c);
void max30102_write(max30102 *obj, uint8_t reg, uint8_t *buf, uint16_t buflen);
void max30102_read(max30102 *obj, uint8_t reg, uint8_t *buf, uint16_t buflen);

void max30102_reset(max30102 *obj);

void max30102_clear_fifo(max30102 *obj);

void max30102_set_mode(max30102 *obj); //spo2 mode
void max30102_interrupt_config(max30102 *obj); //enable relavent interrupts
void max30102_spo2_config(max30102 *obj); //spo2 config
void max30102_fifo_config(max30102 *obj); //config fifo register
void max30102_led2_settings(max30102 *obj); //led2 settings
void max30102_led1_settings(max30102 *obj); //led1 settings

void max30102_set_die_temp_en(max30102 *obj);
void max30102_set_die_temp_rdy(max30102 *obj);

void max30102_interrupt_handler(max30102 *obj); //read interrupt and get relavant data
void max30102_read_fifo(max30102 *obj); //read fifo register

void max30102_read_fifo(max30102 *obj);
void max30102_read_temp(max30102 *obj, int8_t *temp_int, uint8_t *temp_frac);


void push_fresh_sample(max30102 *obj, int *count);

#endif
