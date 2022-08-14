#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "print.h"

#define PRINT_BUFFER_SIZE 256
static char printBuffer[PRINT_BUFFER_SIZE];


static void (*__print_service)(const char *str);

void print_init(void (*print_service)(const char *str)){
	__print_service = print_service;
}

void _print(const char *str) {
    __print_service(str);
}

void _printf(const char *format, va_list args) {

	if(strlen(format) >= PRINT_BUFFER_SIZE) {
		const char* errorMessage = "ERROR: format argument has length bigger than expected len!\n";
		_print(errorMessage);
		_print(format);
		_print("\r\n");
		return;
	}

	int len = vsnprintf(printBuffer, PRINT_BUFFER_SIZE, format, args);

	if(len <= PRINT_BUFFER_SIZE) {
		_print(printBuffer);
	}else{
		const char* errorMessage = "ERROR: Encoding error! in format string ->";
		_print(errorMessage);
		_print(format);
		_print("\r\n");
	}
}

void println(const char *format, ...) {
	va_list args;
	va_start(args, format);
	_printf(format, args);
	va_end(args);
	_print("\r\n");
}

void print(const char *format, ...) {
	va_list args;
	va_start(args, format);
	_printf(format, args);
	va_end(args);
}
