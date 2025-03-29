///*
// * fft_funs.c
// *
// *  Created on: Dec 26, 2023
// *      Author: hoang
// */
#include "fft_funs.h"
#include "pre_modify.h"
#include "math.h"


const float sin_arr_64[]={-0.000000, -0.049068, -0.098017, -0.146730, -0.195090, -0.242980, -0.290285, -0.336890, -0.382683, -0.427555,
                    -0.471397, -0.514103, -0.555570, -0.595699, -0.634393, -0.671559, -0.707107, -0.740951, -0.773010, -0.803208,
                    -0.831470, -0.857729, -0.881921, -0.903989, -0.923880, -0.941544, -0.956940, -0.970031, -0.980785, -0.989177,
                    -0.995185, -0.998795, -1.000000, -0.998795, -0.995185, -0.989177, -0.980785, -0.970031, -0.956940, -0.941544,
                    -0.923880, -0.903989, -0.881921, -0.857729, -0.831470, -0.803208, -0.773010, -0.740951, -0.707107, -0.671559,
                    -0.634393, -0.595699, -0.555570, -0.514103, -0.471397, -0.427555, -0.382683, -0.336890, -0.290285, -0.242980,
                    -0.195090, -0.146730, -0.098017, -0.049068};
const float cos_arr_64[]={1.000000, 0.998795, 0.995185, 0.989177, 0.980785, 0.970031, 0.956940, 0.941544, 0.923880, 0.903989,
                    0.881921, 0.857729, 0.831470, 0.803208, 0.773010, 0.740951, 0.707107, 0.671559, 0.634393, 0.595699,
                    0.555570, 0.514103, 0.471397, 0.427555, 0.382683, 0.336890, 0.290285, 0.242980, 0.195090, 0.146730,
                    0.098017, 0.049068, 0.000000, -0.049068, -0.098017, -0.146730, -0.195090, -0.242980, -0.290285, -0.336890,
                    -0.382683, -0.427555, -0.471397, -0.514103, -0.555570, -0.595699, -0.634393, -0.671559, -0.707107, -0.740951,
                    -0.773010, -0.803208, -0.831470, -0.857729, -0.881921, -0.903989, -0.923880, -0.941544, -0.956940, -0.970031,
                    -0.980785, -0.989177, -0.995185, -0.998795};


void fft(float data_re[], float data_im[],const uint8_t N)
{
	rearrange(data_re, data_im, N);
	compute(data_re, data_im, N);
}

void compute(float data_re[], float data_im[], const uint8_t N)
{
//    const float pi = -3.14159265358979323846;

    for(uint16_t step=1; step<N; step <<=1)
    {
        const uint16_t jump = step << 1;
//        const float step_d = (float) step;
        float twiddle_re = 1.0;
        float twiddle_im = 0.0;

        for(uint16_t group=0; group<step; group++)
        {
        for(uint16_t pair=group; pair<N; pair+=jump)
        {

            const uint16_t match = pair + step;
            const float product_re = (twiddle_re*data_re[match]-twiddle_im*data_im[match]);
            const float product_im = (twiddle_im*data_re[match]+twiddle_re*data_im[match]);
            data_re[match] = data_re[pair]-product_re;
            data_im[match] = data_im[pair]-product_im;
            data_re[pair] += product_re;
            data_im[pair] += product_im;
        }

          // we need the factors below for the next iteration
          // if we don't iterate then don't compute
            if(group+1 == step)
            {
                continue;
            }
//
//                float angle = pi*((float) group+1)/step_d;
//                twiddle_re = cos(angle);
//                twiddle_im = sin(angle);
//                printf("%d   %f  %f  ", (group+1),step_d,((float) group+1)/step_d);

                uint8_t index  = (uint8_t)(( group+1)*N/step/2);
//
                twiddle_re = cos_arr_64[index];
                twiddle_im = sin_arr_64[index];
//                printf("%f   %f    %d\n",twiddle_re,twiddle_im,index);
        }
    }
}

void rearrange(float data_re[], float data_im[], const uint8_t N)
{
  uint16_t target = 0;
  for(uint16_t position=0; position<N; position++)
  {
    if(target>position) {
      const float temp_re = data_re[target];
      const float temp_im = data_im[target];
      data_re[target] = data_re[position];
      data_im[target] = data_im[position];
      data_re[position] = temp_re;
      data_im[position] = temp_im;
    }
    uint16_t mask = N;
    while(target & (mask >>=1))
      target &= ~mask;
    target |= mask;
  }
}

uint8_t argmax_fft(float data_re[], float data_im[], const uint8_t N)
{
	uint32_t max_fft=0;
	uint8_t argmax_fft=0;
	for (uint8_t index_fft = 0 ; index_fft<N/2; index_fft++ )
	{
		uint32_t abs_fft =sqrt(pow(data_re[index_fft],2)+pow(data_im[index_fft],2));
		if (abs_fft>max_fft)
		{

			max_fft =abs_fft;
			argmax_fft= index_fft;
		}

	}


	return argmax_fft;

}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}







