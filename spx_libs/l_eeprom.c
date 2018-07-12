/*
 * ee_sp5K.c
 *
 *  Created on: 26/10/2015
 *      Author: pablo
 */

#include "../spx_libs/l_eeprom.h"

#define EE_VCC_SETTLE_TIME	500

//------------------------------------------------------------------------------------
bool EE_test_write(char *s0, char *s1)
{
	/* Se usa para testear desde el modo comando las funciones de escribir la eeprom.
	 * Desde el modo comando ingresamos 2 parametros que son 2 strings: la direccion y el texto
	 * Para usar EE_write debemos calcular el largo del texto antes de invocarla
	 */

uint8_t length = 0;
char *p;
size_t xReturn = 0U;

	// Calculamos el largo del texto a escribir en la eeprom.
	p = s1;
	while (*p != 0) {
		p++;
		length++;
	}

	frtos_ioctl( fdI2C,ioctl_OBTAIN_BUS_SEMPH, NULL);
	xReturn = EE_write( (uint32_t)(atol(s0)), s1, length );
	frtos_ioctl( fdI2C,ioctl_RELEASE_BUS_SEMPH, NULL);
	return(true);
}
//-----------------------------------------------------------------------------------
bool EE_test_read(char *s0, char *s1, char *s2)
{
	/* Se usa para testear desde el modo comando las funciones de leer la eeprom.
	 * Desde el modo comando ingresamos 2 parametros que son 2 strings: la direccion y el largo
	 */

size_t xReturn = 0U;

	frtos_ioctl( fdI2C,ioctl_OBTAIN_BUS_SEMPH, NULL);
	xReturn = EE_read( (uint32_t)(atol(s0)), s1, (uint8_t)(atoi(s2)) );
	frtos_ioctl( fdI2C,ioctl_RELEASE_BUS_SEMPH, NULL);
	return(true);
}
//-----------------------------------------------------------------------------------
