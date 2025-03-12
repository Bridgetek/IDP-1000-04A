#include "main.h"

#define LED_PIN  PIN_LED_Pin
#define LED_PORT GPIOC

void led_init(){
    #define MAX_LOOP 3
    char count_char = 0, count_loop = 0;
    char data[]="hello      \r\n ";
    
    while (count_loop < MAX_LOOP) {
    	count_loop++;
        count_char = (count_char++) % 9;
        data[7] = count_char + '0';
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(100);
    }
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}
