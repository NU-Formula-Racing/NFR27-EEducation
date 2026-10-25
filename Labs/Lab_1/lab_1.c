
#include "main.h"
#include "blinky.h"
#include "stm32u5xx_hal_tim.h"

extern TIM_HandleTypeDef htim1;

void lab_1() {

    // Start the internal timer
    HAL_TIM_Base_Start_IT(&htim1);
    __HAL_TIM_SET_AUTORELOAD(&htim1, 999);

    while (1) {
        // blink_blocking();
        // blink_nonblocking();
    }
}
