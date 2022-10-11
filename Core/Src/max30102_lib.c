#include  "max30102_lib.h"

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * @brief Built-in plotting function. Called during an interrupt to print/plot the current sample.
 * @note Override this in your main.c if you do not use printf() for printing.
 * @param ir_sample
 * @param red_sample
 */
__weak void max30102_plot(uint32_t ir_sample, uint32_t red_sample)
{

}

/**
 * @brief MAX30102 initiation function.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param hi2c Pointer to I2C object handle
 */
void max30102_init(max30102 *obj, I2C_HandleTypeDef *hi2c)
{
    obj->_ui2c = hi2c;
//    obj->_interrupt_flag = 0;
    memset(obj->_ir_samples, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));
    memset(obj->_red_samples, 0, MAX30102_SAMPLE_LEN_MAX * sizeof(uint32_t));
}

/**
 * @brief Write buffer of buflen bytes to a register of the MAX30102.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param reg Register address to write to.
 * @param buf Pointer containing the bytes to write.
 * @param buflen Number of bytes to write.
 */

void max30102_write(max30102 *obj, uint8_t reg, uint8_t *buf, uint16_t buflen)
{
    uint8_t *payload = (uint8_t *)malloc((buflen + 1) * sizeof(uint8_t));
    *payload = reg;
    if (buf != NULL && buflen != 0)
        memcpy(payload + 1, buf, buflen);
    HAL_I2C_Master_Transmit(obj->_ui2c, MAX30102_I2C_ADDR << 1, payload, buflen + 1, I2C_DELAY);
    free(payload);
}

/**
 * @brief Read buflen bytes from a register of the MAX30102 and store to buffer.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param reg Register address to read from.
 * @param buf Pointer to the array to write to.
 * @param buflen Number of bytes to read.
 */
void max30102_read(max30102 *obj, uint8_t reg, uint8_t *buf, uint16_t buflen)
{
    uint8_t reg_addr = reg;
    HAL_I2C_Master_Transmit(obj->_ui2c, MAX30102_WRITE_ADDRESS, &reg_addr, 1, I2C_DELAY);
    HAL_I2C_Master_Receive(obj->_ui2c, MAX30102_READ_ADDRESS, buf, buflen, I2C_DELAY);
 }


/**
 * @brief Reset the sensor.
 *
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_reset(max30102 *obj)
{
    uint8_t val = 0x40;
    max30102_write(obj, MODE_CONFIGURATION, &val, 1);
}

/**
 * @brief Clear all FIFO pointers in the sensor.
 *
 * @param obj Pointer to max30102_t object instance.
 */
void max30102_clear_fifo(max30102 *obj)
{
    uint8_t val = 0x00;
    max30102_write(obj, FIFO_WRITE_POINTER, &val, 3);
    max30102_write(obj, FIFO_READ_POINTER, &val, 3);
    max30102_write(obj, OVERFLOW_COUNTER, &val, 3);
}

/**
 * @brief
 *
 * @param obj Pointer to max30102_t object instance.
 * @param smp_ave
 * @param roll_over_en Roll over enabled(1) or disabled(0).
 * @param fifo_a_full Number of empty samples when A_FULL interrupt issued (0 < fifo_a_full < 15).
 */
void max30102_set_fifo_config(max30102 *obj)
{
    uint8_t config = 0x00;
    config = fifoConfigurationData;
    max30102_write(obj, FIFO_CONFIGURATION, &config, 1);
}




//set mode for max30102
void max30102_set_mode(max30102 *obj)
{
	uint8_t value = spo2Mode;
	max30102_write(obj, MODE_CONFIGURATION, &value, 1);
}

//enable interrupts
void max30102_interrupt_config(max30102 *obj)
{
	uint8_t value = allInterruptsEnable;
	max30102_write(obj, INTERRUPT_ENABLE_1, &value, 1);
}

//spo2 config
void max30102_spo2_config(max30102 *obj)
{
	uint8_t value = spo2ModeConfiguration;
	max30102_write(obj, SPO2_CONFIGURATION, &value, 1);
}

//fifo config
void max30102_fifo_config(max30102 *obj)
{
	uint8_t value = fifoConfigurationData;
	max30102_write(obj, FIFO_CONFIGURATION, &value, 1);
}

void max30102_led1_settings(max30102 *obj)
{
	uint8_t v = defaultLedPulse;
	max30102_write(obj, LED1_PA, &v, 1);

}

void max30102_led2_settings(max30102 *obj)
{
	uint8_t a = defaultLedPulse;
	max30102_write(obj, LED2_PA, &a, 1);

}

void max30102_set_die_temp_en(max30102 *obj)
{
	uint8_t value = enableTempMeasurement;
	max30102_write(obj, DIE_TEMPERATURE_CONFIG, &value, 1);
}

void max30102_set_die_temp_rdy(max30102 *obj)
{
	uint8_t value = dieTempRdyEn;
	max30102_write(obj, INTERRUPT_ENABLE_2, &value, 1);
}

void max30102_interrupt_handler(max30102 *obj)
{
	//read register 0x00 and 0X01 interrupt status register
    uint8_t reg[2] = {0x00};
    // Interrupt flag in registers 0x00 and 0x01 are cleared on read
    max30102_read(obj, INTERRUPT_STATUS_1, reg, 2);

    uint8_t b = reg[0];
    	if((b & 0X01) == 1)
    	{
    	//PWR_RDY shows that sensor has been set off. Configure sensor again.

    	}

    	uint8_t a = reg[0];
    	if(((a>>7) & 0X01) == 1 )
    	{
    	//Interrupt becoming full now read sensor FIFO data
	     max30102_read_fifo(obj);
    	}

    	uint8_t c = reg[0];
    	if(((c>>6) & 0X01) == 1 )
    	{
    	//PPG_RDY
    	}

    	if((reg[1]>>1) & 0X01)
    	{
    	// Temperature data ready
    		  int8_t temp_int;
    		  uint8_t temp_frac;
    		  max30102_read_temp(obj, &temp_int, &temp_frac);

    		  obj->temperature = temp_int + 0.0625f * temp_frac;
    	}
}

void max30102_read_fifo(max30102 *obj)
{
    // First transaction: Get the FIFO_WR_PTR
    uint8_t wr_ptr;
    uint8_t rd_ptr;
    max30102_read(obj, FIFO_WRITE_POINTER, &wr_ptr, 1);
    max30102_read(obj, FIFO_READ_POINTER, &rd_ptr, 1);

    uint8_t num_samples = 0;

    if(wr_ptr > 0 )
    {
    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    }


    num_samples  = ((uint8_t)wr_ptr - (uint8_t)rd_ptr + 32)%32;


    for(uint8_t i = 0; i< num_samples; i++)
    {
    	uint8_t sample[6];
    	max30102_read(obj, FIFO_DATA_REGISTER, sample, 6);
        uint32_t ir_sample = ((uint32_t)(sample[0] << 16) | (uint32_t)(sample[1] << 8) | (uint32_t)(sample[2])) & 0x3ffff;
        uint32_t red_sample = ((uint32_t)(sample[3] << 16) | (uint32_t)(sample[4] << 8) | (uint32_t)(sample[5])) & 0x3ffff;

        obj->_ir_samples[i] = ir_sample;
        obj->_red_samples[i] = red_sample;
    }


}


/**
 * @brief Read die temperature.
 *
 * @param obj Pointer to max30102_t object instance.
 * @param temp_int Pointer to store the integer part of temperature. Stored in 2's complement format.
 * @param temp_frac Pointer to store the fractional part of temperature. Increments of 0.0625 deg C.
 */

void max30102_read_temp(max30102 *obj, int8_t *temp_int, uint8_t *temp_frac)
{
    max30102_read(obj, DIE_TEMPERATURE_INTEGER, (uint8_t *)temp_int, 1);
    max30102_read(obj, DIE_TEMPERATURE_FRACTION, temp_frac, 1);
}

#ifdef __cplusplus
}
#endif
