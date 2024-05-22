#include "stm32f7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "encoder.h"

#define FULL_COUNTER 0xFFFF
#define HALF_COUNTER 0x7FFF
// resolution of degree after decimal point
#define DEGREE_RESOLUTION 100     // 2 decimal points
#define PULSE_PER_REVOLUTION 2000 // 2000 pulses per revolution

volatile static int32_t counter32Bit = 0; // store 32-bit counter value
volatile static uint16_t prevCounter = 0; // record previous counter value of 16-bit timer
volatile static int16_t revolution = 0;   // one revolution is 65536, one period of 16-bit timer

void vEncoderTask(void *pvParameters)
{
    // degree value
    int32_t degree = 0;

    // Infinite loop
    for (;;)
    {
        // Read encoder value
        // int encoderValue = readEncoder();

        // Read degree value
        degree = readDegree();
        vLoggingPrintf("Degree: %d.%d\n", degree / DEGREE_RESOLUTION, abs(degree) % DEGREE_RESOLUTION);

        // print distance value
        vLoggingPrintf("Distance: %d\n", degree * 2 / 360);

        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void HandleEncoderInterrupt(TIM_HandleTypeDef *htim)
{
    uint16_t currentCounter = __HAL_TIM_GET_COUNTER(htim);

    // Detect Overflow / Underflow event to Increase / Decrease revolution counter
    int32_t diff = currentCounter - prevCounter;

    if (diff < 0)
    {
        // overflow occurred
        if (-diff > HALF_COUNTER)
            revolution++;
    }
    else
    {
        // underflow occurred
        if (diff > HALF_COUNTER)
            revolution--;
    }

    counter32Bit = (int32_t)(revolution << 16) + currentCounter;
    prevCounter = currentCounter;
}

uint32_t getCounter()
{
    return counter32Bit;
}

void resetCounter(TIM_HandleTypeDef *htim)
{
    counter32Bit = 0;
    prevCounter = 0;
    revolution = 0;

    __HAL_TIM_SET_COUNTER(htim, 0);
}

int32_t readDegree()
{
    return (int32_t)((counter32Bit * 360 * DEGREE_RESOLUTION) / PULSE_PER_REVOLUTION);
}