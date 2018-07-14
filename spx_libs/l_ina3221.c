/*
 * l_ina3221.c
 *
 *  Created on: 13 de oct. de 2017
 *      Author: pablo
 */

#include "l_ina3221.h"

//------------------------------------------------------------------------------------
uint8_t pv_INA_id2busaddr( uint8_t id )
{
	switch(id) {
	case 0:
		return(INA0_DEVADDR);	// Canales 0,1,2
		break;
	case 1:
		return(INA0_DEVADDR); // Canales 3,4,5
		break;
	default:
		return(99);
		break;

	}

	return(99);

}
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
int INA_read( uint8_t devAddress, uint8_t regAddress, char *data, uint8_t length )
{
	// C/registro es de 2 bytes de informacion.

size_t xReturn = 0U;
uint8_t val = 0;
uint8_t xBytes = 0;

//		FRTOS_snprintf_P( stdout_buff, sizeof(stdout_buff),PSTR( "INArd: devAddr=0x%02x,regAddr=0x%02x\r\n\0"), devAddress, regAddress );
//		CMD_write( stdout_buff, sizeof(stdout_buff) );

		// Lo primero es obtener el semaforo
		frtos_ioctl(fdI2C,ioctl_OBTAIN_BUS_SEMPH, NULL);

		// Luego indicamos el periferico i2c en el cual queremos leer
		val = devAddress;
		frtos_ioctl(fdI2C,ioctl_I2C_SET_DEVADDRESS, &val);
//		FRTOS_snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: devAddr=0x%02x\r\n\0"), val );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

		// Luego indicamos el registro desde donde leer: largo ( 1 bytes )
		val = 1;
		frtos_ioctl(fdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
//		FRTOS_snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: length=0x%02x\r\n\0"), val );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

		// y direccion
		val = regAddress;
		frtos_ioctl(fdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);
//		FRTOS_snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: regAddr=0x%02x\r\n\0"), val );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

		// Por ultimo leemos.( 2 bytes )
		xBytes = 2;
		xReturn = frtos_read(fdI2C, data, xBytes);
//		FRTOS_snprintf_P( debug_printfBuff, sizeof(debug_printfBuff),PSTR( "INArd: size=0x%02x\r\n\0"), xBytes );
//		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

		// Y libero el semaforo.
		frtos_ioctl(fdI2C,ioctl_RELEASE_BUS_SEMPH, NULL);

		if (xReturn != xBytes ) {
			return ( false );
		}

		return(true);

}
//------------------------------------------------------------------------------------
int INA_write( uint8_t devAddress, uint8_t regAddress, char *data, uint8_t length )
{
	// Escribe en el INA3221 en la posicion regAddress, la cantidad
	// 'length' de bytes apuntados por 'data'
	// En los INA3221 siempre se escriben solo 2 bytes de datos !!!
	//
size_t xReturn = 0U;
uint8_t val = 0;
uint8_t xBytes = 0;

	// Lo primero es obtener el semaforo
	frtos_ioctl(fdI2C,ioctl_OBTAIN_BUS_SEMPH, NULL);

	// Luego indicamos el periferico i2c ( INA3221 ) en el cual queremos leer
	val = devAddress;
	frtos_ioctl(fdI2C,ioctl_I2C_SET_DEVADDRESS, &val);

	// Luego indicamos la direccion ( el registro ) a partir de donde escribir: largo ( 1 bytes )
	val = 1;
	frtos_ioctl(fdI2C,ioctl_I2C_SET_BYTEADDRESSLENGTH, &val);
	// y direccion ( registro )
	val = regAddress;
	frtos_ioctl(fdI2C,ioctl_I2C_SET_BYTEADDRESS,&val);

	// Por ultimo escribimos xBytes.
	//xBytes = length;
	xBytes = 2;	// En los INA siempre son 2 bytes
	xReturn = frtos_write(fdI2C, data, xBytes);

	// Y libero el semaforo.
	frtos_ioctl(fdI2C,ioctl_RELEASE_BUS_SEMPH, NULL);

	if (xReturn != xBytes ) {
		return ( false );
	}

	return(true);

}
