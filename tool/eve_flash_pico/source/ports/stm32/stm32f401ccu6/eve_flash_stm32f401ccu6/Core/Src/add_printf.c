#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "common.h"
#include "add_printf.h"

#define UART_HANDLE huart2

extern UART_HandleTypeDef UART_HANDLE;
void vprint(char is_new_line, const char *fmt, va_list argp)
{
#define BUFF_LEN 200
    char string[BUFF_LEN];
    int is_print=vsnprintf(string, BUFF_LEN, fmt, argp) > 0;
    int len=min(strlen(string), BUFF_LEN);
    if(is_new_line){
    	int len2=min(len+3, BUFF_LEN);
		string[len2-1]=0;
		string[len2-2]='\n';
		string[len2-3]='\r';
    }
    if(is_print) // build string
    {
        HAL_UART_Transmit(&UART_HANDLE, (uint8_t*)string, len, 0xffffff); // send message via UART
    }else{
        HAL_UART_Transmit(&UART_HANDLE, (uint8_t*)string, len, 0xffffff); // send message via UART
    }
}

void printf_init(){
	my_printf("UF2 flash tool for STM32, build %s", __TIME__);
#if 0
	char count = 0, loop = 0;
	while (loop < 30) {
		count = (count++) % 9;
		char data[]="hello  world\r\n ";
		data[6] = count + '0';
		data[7] = 0;
		//printf("%s --", data);
		HAL_UART_Transmit(&huart2, (char*)data, strlen(data), 10);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(100);
		break;
	}
#endif
}

int my_printfn(const char *fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(0, fmt, argp);
    va_end(argp);
}

int my_printf(const char *fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(1, fmt, argp);
    va_end(argp);
}
