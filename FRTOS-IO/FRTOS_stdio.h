/*
 * FRTOS_stdio.h
 *
 *  Created on: 23 de abr. de 2018
 *      Author: pablo
 */

#ifndef FRTOS_IO_FRTOS_STDIO_H_
#define FRTOS_IO_FRTOS_STDIO_H_


#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <avr/pgmspace.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

xSemaphoreHandle sem_PRINTF;

void FRTOS_stdio_init(void);

/**
 * Output a character to a custom device like UART, used by the printf() function
 * This function is declared here only. You have to write your custom implementation somewhere
 * \param character Character to output
 */
//void _putchar(char character);


/**
 * Tiny printf implementation
 * You have to implement _putchar if you use printf()
 * \param format A string that specifies the format of the output
 * \return The number of characters that are written into the array, not counting the terminating null character
 */
//int printf(const char* format, ...);


/**
 * Tiny sprintf implementation
 * Due to security reasons (buffer overflow) YOU SHOULD CONSIDER USING SNPRINTF INSTEAD!
 * \param buffer A pointer to the buffer where to store the formatted string. MUST be big enough to store the output!
 * \param format A string that specifies the format of the output
 * \return The number of characters that are WRITTEN into the buffer, not counting the terminating null character
 */
int sprintf(char* buffer, const char* format, ...);


/**
 * Tiny snprintf/vsnprintf implementation
 * \param buffer A pointer to the buffer where to store the formatted string
 * \param count The maximum number of characters to store in the buffer, including a terminating null character
 * \param format A string that specifies the format of the output
 * \return The number of characters that are WRITTEN into the buffer, not counting the terminating null character
 *         If the formatted string is truncated the buffer size (count) is returned
 */
int  FRTOS_snprintf(char* buffer, size_t count, const char* format, ...);

int vsnprintf(char* buffer, size_t count, const char* format, va_list va);

int  FRTOS_snprintf_P(char* buffer, size_t count, const char* format, ...);

#endif /* FRTOS_IO_FRTOS_STDIO_H_ */
