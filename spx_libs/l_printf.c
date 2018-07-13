/*
 * l_printf.c
 *
 *  Created on: 13 jul. 2018
 *      Author: pablo
 */


#include "l_printf.h"


#define PRINTF_BUFFER_SIZE        256U

static uint8_t stdout_buff[PRINTF_BUFFER_SIZE];
xSemaphoreHandle sem_stdout_buff;

//-----------------------------------------------------------------------------------
int xprintf_P( PGM_P fmt, ...)
{
	// Imprime formateando en el uart fd.usando el buffer stdout_buff.
	// Como se controla con semaforo, nos permite ahorrar los buffers de c/tarea.
	// Si bien vsnprintf no es thread safe, al usarla aqui con el semaforo del buffer
	// queda thread safe !!!

va_list args;
int i;

	// Espero el semaforo del buffer en forma persistente.
	while ( xSemaphoreTake( sem_stdout_buff, ( TickType_t ) 1 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

	// Ahora tengo en stdout_buff formateado para imprimir
	memset(stdout_buff,'\0',PRINTF_BUFFER_SIZE);
	va_start(args, fmt);
	vsnprintf_P( (char *)stdout_buff,sizeof(stdout_buff),fmt,args);
	i = frtos_write(fdUSB, (char *)stdout_buff, PRINTF_BUFFER_SIZE );

	xSemaphoreGive( sem_stdout_buff );
	return(i);

}
//-----------------------------------------------------------------------------------
int xCom_printf_P( file_descriptor_t fd, PGM_P fmt, ...)
{
	// Idem que xprintf_P pero imprime sobre el descriptor tipo uart indicado con fd.

va_list args;
int i;

	// Espero el semaforo del buffer en forma persistente.
	while ( xSemaphoreTake( sem_stdout_buff, ( TickType_t ) 1 ) != pdTRUE )
		vTaskDelay( ( TickType_t)( 1 ) );

	// Ahora tengo en stdout_buff formateado para imprimir
	memset(stdout_buff,'\0',PRINTF_BUFFER_SIZE);
	va_start(args, fmt);
	vsnprintf_P( (char *)stdout_buff,sizeof(stdout_buff),fmt,args);
	i = frtos_write(fd, (char *)stdout_buff, PRINTF_BUFFER_SIZE );

	xSemaphoreGive( sem_stdout_buff );
	return(i);

}
//-----------------------------------------------------------------------------------
void xprintf_init(void)
{
	sem_stdout_buff = xSemaphoreCreateMutex();
}
//------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
// Formatea e imprime en el fdUSB
//-----------------------------------------------------------------------------------
/*
void xprintf( const char * format, ...)
{
	// Formatea e imprime en fdUSB.
	//

va_list arg;

	va_start(arg, format);

	vsnprintf((char *)(xSerialPort.serialWorkBuffer), xSerialPort.serialWorkBufferSize, (const char *)format, arg);
	xSerialPrint((uint8_t *)(xSerialPort.serialWorkBuffer));

	va_end(arg);
}
//-----------------------------------------------------------------------------------
void xprintf_P(PGM_P format, ...)
{
	va_list arg;

	va_start(arg, format);

	while(xSerialPort.serialWorkBufferInUse == ENGAGED ) taskYIELD();
	xSerialPort.serialWorkBufferInUse = ENGAGED;

	vsnprintf_P((char *)(xSerialPort.serialWorkBuffer), xSerialPort.serialWorkBufferSize, format, arg);
	xSerialPrint((uint8_t *)(xSerialPort.serialWorkBuffer));

	xSerialPort.serialWorkBufferInUse = VACANT;

	va_end(arg);
}
//-----------------------------------------------------------------------------------
// Imprime sin formatear en el fdUSB
//-----------------------------------------------------------------------------------
void xprint( const uint8_t * str)
{
	int16_t i = 0;
	size_t stringlength;

	stringlength = strlen((char *)str);

	while(i < stringlength)
		xSerialPutChar( &xSerialPort, str[i++] );
}
//-----------------------------------------------------------------------------------
void xprint_P(PGM_P str)
{
	uint16_t i = 0;
	size_t stringlength;

	stringlength = strlen_P(str);

	while(i < stringlength)
		xSerialPutChar( &xSerialPort, pgm_read_byte(&str[i++]) );
}
//-----------------------------------------------------------------------------------
// Formatea e imprime en un fd tipo uart
//-----------------------------------------------------------------------------------
void xCom_printf( const xComPortHandlePtr pxPort, const char * format, ...)
{
	va_list arg;

	va_start(arg, format);

	while(pxPort->serialWorkBufferInUse == ENGAGED ) taskYIELD();
	pxPort->serialWorkBufferInUse = ENGAGED;

	vsnprintf((char *)(pxPort->serialWorkBuffer), pxPort->serialWorkBufferSize, (const char *)format, arg);
	xSerialxPrint(pxPort, (uint8_t *)(pxPort->serialWorkBuffer));

	pxPort->serialWorkBufferInUse = VACANT;

	va_end(arg);
}
//-----------------------------------------------------------------------------------
void xCom_printf_P( const xComPortHandlePtr pxPort, PGM_P format, ...)
{
	va_list arg;

	va_start(arg, format);

	while(pxPort->serialWorkBufferInUse == ENGAGED ) taskYIELD();
	pxPort->serialWorkBufferInUse = ENGAGED;

	vsnprintf_P((char *)(pxPort->serialWorkBuffer), pxPort->serialWorkBufferSize, format, arg);
	xSerialxPrint(pxPort, (uint8_t *)(pxPort->serialWorkBuffer));

	pxPort->serialWorkBufferInUse = VACANT;

	va_end(arg);
}
//-----------------------------------------------------------------------------------
// Imprime sin formatear en un fd tipo uart
//-----------------------------------------------------------------------------------
void xCom_print( const xComPortHandlePtr pxPort, const uint8_t * str)
{
	int16_t i = 0;
	size_t stringlength;

	stringlength = strlen((char *)str);

	while(i < stringlength)
		xSerialPutChar( pxPort, str[i++]);
}
//-----------------------------------------------------------------------------------
void xCom_print_P( const xComPortHandlePtr pxPort, PGM_P str)
{
	uint16_t i = 0;
	size_t stringlength;

	stringlength = strlen_P(str);

	while(i < stringlength)
		xSerialPutChar( pxPort, pgm_read_byte(&str[i++]) );
}
//-----------------------------------------------------------------------------------
*/
