
#include "blinky.h"
#include "main.h"
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_gpio.h"
#include "stm32u5xx_hal_tim.h"
#include <stdint.h>

void blink_blocking() {
    HAL_GPIO_WritePin(Status_LED_GPIO_Port,
        Status_LED_Pin,
        GPIO_PIN_SET);
    
    HAL_Delay(BLINK_DURATION_DEFAULT);

    HAL_GPIO_WritePin(Status_LED_GPIO_Port,
        Status_LED_Pin,
        GPIO_PIN_RESET);
    
    HAL_Delay(BLINK_DURATION_DEFAULT);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    HAL_GPIO_TogglePin(Status_LED_GPIO_Port, Status_LED_Pin);
}
