/* vim: set ai et ts=4 sw=4: */
/*
 * R820T2 downconverter driver by Eric Brombaugh, 2017
 * Ported to HAL and refactored by Aleksander Alekseev, 2018
 */

#ifndef __r820t2__
#define __r820t2__

#include "stm32f4xx.h"

#define R820T2_I2C_PORT hi2c1
extern I2C_HandleTypeDef R820T2_I2C_PORT;

#define R820T2_NUM_REGS     0x20

void R820T2_read(uint8_t addr, uint8_t *data, uint8_t num);
void R820T2_write(uint8_t addr, uint8_t *data, uint8_t num);
uint8_t R820T2_read_reg(uint8_t n);
void R820T2_write_reg(uint8_t n, uint8_t data);
void R820T2_init(void);
int32_t R820T2_calibrate(void);
void R820T2_set_frequency(uint32_t freq);
void R820T2_set_bandwidth(uint8_t bw);
void R820T2_set_lna_gain(uint8_t gain_index);
void R820T2_set_mixer_gain(uint8_t gain_index);
void R820T2_set_vga_gain(uint8_t gain_index);
void R820T2_set_lna_agc(uint8_t value);
void R820T2_set_mixer_agc(uint8_t value);

#endif
