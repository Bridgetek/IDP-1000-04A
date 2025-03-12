#ifndef ADD_PRINTF__H_
#define ADD_PRINTF__H_
//#include "main.h"
//
void printf_init();
int my_printf(const char *fmt, ...);
int my_printfn(const char *fmt, ...);

#define printf my_printf
#define printfn my_printfn

#endif
