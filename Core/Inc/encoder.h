#ifndef __ENCODER_H
#define __ENCODER_H

/* function prototypes */
void vEncoderTask(void *pvParameters);
void HandleEncoderInterrupt(TIM_HandleTypeDef *htim);
uint32_t getCounter();
void resetCounter(TIM_HandleTypeDef *htim);
int32_t readDegree();

#endif // __ENCODER_H