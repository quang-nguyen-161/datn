#ifndef _MAX_H
#define _MAX_H

#include "stdbool.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"

#include "nrf_drv_timer.h"
#include <stdint.h>
#include <stdbool.h>
#include "filter.h"

#define MAX30102_I2C_ADDR 0x57
#define MAX30102_I2C_TIMEOUT 1000
#define MAX30102_ADDR_LEN 1

#define MAX30102_BYTES_PER_SAMPLE 6
#define MAX30102_SAMPLE_LEN_MAX 32

#define MAX30102_SAMPLES_PER_SECOND	100
#define MAX30102_MEASUREMENT_SECONDS 5
#define MAX30102_BUFFER_LENGTH	((MAX30102_MEASUREMENT_SECONDS+1)*MAX30102_SAMPLES_PER_SECOND)
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_100

#define MAX30102_INTERRUPT_STATUS_1 0x00
#define MAX30102_INTERRUPT_STATUS_2 0x01
#define MAX30102_INTERRUPT_ENABLE_1 0x02
#define MAX30102_INTERRUPT_ENABLE_2 0x03
#define MAX30102_INTERRUPT_A_FULL 7
#define MAX30102_INTERRUPT_PPG_RDY 6
#define MAX30102_INTERRUPT_ALC_OVF 5
#define MAX30102_INTERRUPT_DIE_TEMP_RDY 1

#define MAX30102_FIFO_WR_PTR 0x04
#define MAX30102_OVF_COUNTER 0x05
#define MAX30102_FIFO_RD_PTR 0x06

#define MAX30102_FIFO_DATA 0x07

#define MAX30102_FIFO_CONFIG 0x08
#define MAX30102_FIFO_CONFIG_SMP_AVE 5
#define MAX30102_FIFO_CONFIG_ROLL_OVER_EN 4
#define MAX30102_FIFO_CONFIG_FIFO_A_FULL 0

#define MAX30102_MODE_CONFIG 0x09
#define MAX30102_MODE_SHDN 7
#define MAX30102_MODE_RESET 6
#define MAX30102_MODE_MODE 0

#define MAX30102_SPO2_CONFIG 0x0a
#define MAX30102_SPO2_ADC_RGE 5
#define MAX30102_SPO2_SR 2
#define MAX30102_SPO2_LEW_PW 0

#define MAX30102_LED_IR_PA1 0x0c
#define MAX30102_LED_RED_PA2 0x0d

#define MAX30102_MULTI_LED_CTRL_1 0x11
#define MAX30102_MULTI_LED_CTRL_SLOT2 4
#define MAX30102_MULTI_LED_CTRL_SLOT1 0
#define MAX30102_MULTI_LED_CTRL_2 0x12
#define MAX30102_MULTI_LED_CTRL_SLOT4 4
#define MAX30102_MULTI_LED_CTRL_SLOT3 0

#define MAX30102_DIE_TINT 0x1f
#define MAX30102_DIE_TFRAC 0x20
#define MAX30102_DIE_TFRAC_INCREMENT 0.0625f
#define MAX30102_DIE_TEMP_CONFIG 0x21
#define MAX30102_DIE_TEMP_EN 1

#define STORAGE_SIZE 32             /* Buffer size in samples */

typedef enum max30102_mode_t
{
    max30102_heart_rate = 0x02,
    max30102_spo2 = 0x03,
    max30102_multi_led = 0x07
} max30102_mode_t;

typedef enum max30102_smp_ave_t
{
    max30102_smp_ave_1,
    max30102_smp_ave_2,
    max30102_smp_ave_4,
    max30102_smp_ave_8,
    max30102_smp_ave_16,
    max30102_smp_ave_32,
} max30102_smp_ave_t;

typedef enum max30102_sr_t
{
    max30102_sr_50,
    max30102_sr_100,
    max30102_sr_200,
    max30102_sr_400,
    max30102_sr_800,
    max30102_sr_1000,
    max30102_sr_1600,
    max30102_sr_3200
} max30102_sr_t;

typedef enum max30102_led_pw_t
{
    max30102_pw_15_bit,
    max30102_pw_16_bit,
    max30102_pw_17_bit,
    max30102_pw_18_bit
} max30102_led_pw_t;

typedef enum max30102_adc_t
{
    max30102_adc_2048,
    max30102_adc_4096,
    max30102_adc_8192,
    max30102_adc_16384
} max30102_adc_t;

typedef enum max30102_multi_led_ctrl_t
{
    max30102_led_off,
    max30102_led_red,
    max30102_led_ir
} max30102_multi_led_ctrl_t;

typedef struct max30102_t
{
    uint32_t _ir_samples[STORAGE_SIZE];
    uint32_t _red_samples[STORAGE_SIZE];
    uint8_t _interrupt_flag;
	uint8_t head;
	uint8_t tail;
} max30102_t;

typedef struct samplestruct
{
    uint32_t red;
    uint32_t iRed;
} SAMPLE;





void max30102_write(uint8_t register_address, uint8_t value);
void max30102_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes);
void max30102_init();
void max30102_reset();
void max30102_turnoff();
void max30102_turnon();
void max30102_clear_fifo();
void max30102_set_led_pulse_width(max30102_led_pw_t pw);
void max30102_set_fifo_config(max30102_smp_ave_t smp_ave, uint8_t roll_over_en, uint8_t fifo_a_full);
void max30102_set_adc_resolution(max30102_adc_t adc);
void max30102_set_sampling_rate(max30102_sr_t sr);
void max30102_set_led_current_1(float ma);
void max30102_set_led_current_2(float ma);
void max30102_set_mode(max30102_mode_t mode);
void max30102_set_a_full(uint8_t enable);
void max30102_set_die_temp_en(uint8_t enable);
void max30102_set_die_temp_rdy(uint8_t enable);
void max30102_plot(uint32_t ir_sample, uint32_t red_sample, uint32_t time);
void max30102_get_hr_spo2(uint32_t *HR, uint32_t *SpO2);
void Max_read_fifo_sample(uint32_t *ir, uint32_t *red);
void max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led);
void max30102_sample_push(uint32_t *ir_buff, uint32_t *red_buff, uint16_t size, uint32_t sample_rate);
void max30102_setup();
void max30102_wakeup();

uint8_t  Max_num_available_data(void);
uint32_t Max_get_red_signal(void);
uint32_t Max_get_ir_signal(void);
void Max_next_sample(void);
uint16_t Max_check_available_data(void);

//void filter_buff(uint32_t *buff, uint32_t *filtered_buff, uint32_t size);
//void ring_buffer_push(uint32_t *buff, uint32_t value, uint32_t size, uint32_t *head);
//bool find_peak(uint32_t *buff, uint32_t size, uint32_t head);
//void peak_time_push(uint32_t *buff,
  //                  uint32_t *delta_t_buff,
  //                  uint32_t size,
  //                  uint32_t head,
  //                  uint32_t *dt_head,
	//									uint32_t delta_size);
//uint32_t hr_count(uint32_t *delta_t_buff,
   //               uint32_t delta_size,
   //               uint32_t peak,
   //              uint32_t dt_head);
//uint32_t spo2_count(uint32_t *ir_buff,
   //                   uint32_t *red_buff,
    //                  uint32_t size,
    //                  uint32_t head);
void timer2_init(void);
 uint32_t timer2_now(void);

uint32_t DC_mean(uint32_t * buff, uint32_t size, uint32_t last_DC, uint32_t last_value, uint32_t head);

typedef struct 
{
	float buff[100];
	uint8_t rb_head;
} rb_t;

#define DC_WINDOW 125  // ~0.5s at 100Hz
typedef struct {
    float buf[DC_WINDOW];
		float dc_comp;
    int idx;
    float sum;
} dc_filter_t;

bool find_peak(rb_t *rb);
void rb_push(rb_t *rb, float value);
void peak_time_push(rb_t *signal_rb,
                    rb_t *dt_rb);
uint32_t hr_count(rb_t *dt_rb,
                  uint32_t peak_count);
uint32_t spo2_count(dc_filter_t *dc_ir,
                    dc_filter_t *dc_red,
                    rb_t *ir_rb,
                    rb_t *red_rb);

void peak_time_push1(uint32_t *buff,
                    uint32_t *delta_t_buff,
                    uint32_t size,
                    uint32_t head,
                    uint32_t *dt_head,
										uint32_t delta_size);
uint32_t hr_count1(uint32_t *delta_t_buff,
                  uint32_t delta_size,
                  uint32_t peak,
                  uint32_t dt_head);
uint32_t spo2_count1(uint32_t *ir_buff,
                      uint32_t *red_buff,
                      uint32_t size,
                      uint32_t head);
void ring_buffer_push(uint32_t *buff, uint32_t value, uint32_t size, uint32_t *head);

#endif