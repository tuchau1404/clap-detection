/*
 * pre_modify.h
 *
 *  Created on: Dec 26, 2023
 *      Author: hoang
 */

#ifndef INC_PRE_MODIFY_H_
#define INC_PRE_MODIFY_H_

#include <stdint.h>

#define SAMPLE 128
#define HALF_SAMPLE (SAMPLE/2)
#define PI -3.14159265358979323846
#define MAX_AMP 32767
#define MIN_AMP -32767
#define SAMPLE_RATE 16000
/* tim2, clk=100MHz, prescaler=100-1, */
#define INTERVAL_SAMPLE (10000000/SAMPLE_RATE)
#define FREQ_INDEX 3
#define AMP_THRESHOLD 18000



extern uint32_t adc_val;

extern uint8_t flag_loud_first;
extern uint8_t flag_loud_second;

extern float data_re_first[SAMPLE];
extern float data_im_first[SAMPLE];
extern float data_re_second[SAMPLE];
extern float data_im_second[SAMPLE];





#endif /* INC_PRE_MODIFY_H_ */
