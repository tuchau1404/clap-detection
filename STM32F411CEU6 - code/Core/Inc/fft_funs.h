/*
 * fft_funs.h
 *
 *  Created on: Dec 26, 2023
 *      Author: hoang
 */

#ifndef INC_FFT_FUNS_H_
#define INC_FFT_FUNS_H_
#include <stdint.h>

extern const float sin_arr_64[64];
extern const float cos_arr_64[64];





int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
void fft(float data_re[], float data_im[],const uint8_t N);
void compute(float data_re[], float data_im[], const uint8_t N);
void rearrange(float data_re[], float data_im[], const uint8_t N);
uint8_t argmax_fft(float data_re[], float data_im[], const uint8_t N);



#endif /* INC_FFT_FUNS_H_ */
