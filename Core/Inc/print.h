/*
 * print.h
 *
 *  Created on: Aug 14, 2022
 *      Author: petarlalov
 */

#ifndef INC_PRINT_H_
#define INC_PRINT_H_

void print_init(void (*print_service)(const char *str));

void println(const char *format, ...);
void print(const char *format, ...);


#endif /* INC_PRINT_H_ */
