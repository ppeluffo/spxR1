/*
 * sp5KFRTOS_rtc.c
 *
 *  Created on: 01/11/2013
 *      Author: root
 *
 * Funciones del RTC DS1340-33 modificadas para usarse con FRTOS.
 *
 *
 */
//------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#include <l_rtc79410.h>

#include "spx.h"

static char pv_bcd2dec(char num);
static char pv_dec2bcd(char num);

//------------------------------------------------------------------------------------
// Funciones de uso general
//------------------------------------------------------------------------------------
void RTC79410_start(void)
{

RtcTimeType_t rtc;
uint8_t data;

	// cuando arranca el RTC de una situacion de pwr_off no battery, esta apagado.
	// Para que comienze a correr debemos poner el bit7 de RTCSEC en 1.
	// Por otro lado, puede estar reseteado con lo que la fecha aparece en 01 01 2000.

	// Leo la hora
	RTC79410_read_dtime( &rtc);

	// Si esta reseteado la reconfiguro
	if ( ( rtc.year < 2018 ) || ( rtc.year > 2040) ){
		RTC_str2rtc("1801010000", &rtc);
		RTC79410_write_dtime(&rtc);

	} else {
	// Escribo ST = 1 para asegurarme de haber activado el RTC
		// Habilito el OSCILADOR
		data = 0x80;
		RTC79410_write(0x00, &data , 1);
	}
	return;

}
//------------------------------------------------------------------------------------
bool RTC79410_read_dtime(RtcTimeType_t *rtc)
{
	// Retorna la hora formateada en la estructura RtcTimeType_t
	// No retornamos el valor de EOSC ni los bytes adicionales.

uint8_t data[8];
uint8_t rdBytes = 0;

	// Lo primero es obtener el semaforo
	frtos_ioctl( fdI2C,ioctl_OBTAIN_BUS_SEMPH, NULL);

	rdBytes = RTC79410_read(0x00, &data, 7);

	// Y libero el semaforo.
	frtos_ioctl( fdI2C,ioctl_RELEASE_BUS_SEMPH, NULL);

	if (rdBytes != 7 ) {
		return ( false );
	}

	// Decodifico los resultados del buffer para ponerlos en la estructura RTC

	rtc->sec = pv_bcd2dec(data[0] & 0x7F);
	rtc->min = pv_bcd2dec(data[1]);
	rtc->hour = pv_bcd2dec(data[2] & 0x3F);
	rtc->day = pv_bcd2dec(data[4] & 0x3F);
	rtc->month = pv_bcd2dec(data[5] & 0x1F);
	rtc->year = pv_bcd2dec(data[6]) + 2000;

	return(true);
}
//------------------------------------------------------------------------------------
bool RTC79410_write_dtime(RtcTimeType_t *rtc)
{
	// Setea el RTC con la hora pasada en la estructura RtcTimeType
	// El procedimiento es:
	// Pongo ST en 0.
	// Espero que el bit OSCRUN este en 0.
	// Grabo la nueva hora
	// Arranco el reloj poniendo ST en 1.

uint8_t data[8];

	frtos_ioctl( fdI2C,ioctl_OBTAIN_BUS_SEMPH, NULL);

	// Pongo ST en 0.
	// Como está en el registro que corresponde a los segundos, pongo todo el
	// byte en 0.
	data[0] = 0x00;
	RTC79410_write(0x00, &data, 1 );
	//
	// Espero que el OSCRUN quede en 0.
	while ( ( data[0] & 0x20 ) != 0 ) {
		RTC79410_read( 0x03, &data, 1 );
		vTaskDelay( ( TickType_t) 1 );
	}

	// Ahora puedo grabar la nueva hora.
	// Grabo los datos sin habilitar aun el oscilador.
	data[0] = 0x00;								// RTCSEC: ST = 0, Oscillator disabled.
	data[1] = pv_dec2bcd(rtc->min & 0x7F);		// RTCMIN
	data[2] = pv_dec2bcd(rtc->hour & 0x1F);		// RTCHOUR, 12/24 = 0: 24 hours.
	data[3] = 0x09;								// RTCWKDAY:, VBAT enable
	data[4] = pv_dec2bcd(rtc->day);				// RTCDATE
	data[5] = pv_dec2bcd(rtc->month & 0x1F);	// RTCMONTH
	data[6] = pv_dec2bcd(rtc->year - 2000);		// RTCYEAR

	RTC79410_write(0x00, &data, 7);

	// Habilito el OSCILADOR
	data[0] = 0x80;
	RTC79410_write(0x00, &data, 1 );

	frtos_ioctl( fdI2C,ioctl_RELEASE_BUS_SEMPH, NULL);

	return(true);

}
//------------------------------------------------------------------------------------
bool RTC79410_test_write(char *s0, char *s1)
{
	/* Se usa para testear desde el modo comando las funciones de escribir la sram del RTC.
	 * Desde el modo comando ingresamos 2 parametros que son 2 strings: la direccion y el texto
	 * Para usar RTC79410_write_sram debemos calcular el largo del texto antes de invocarla
	 *
	 * Aqui es donde usamos el SEMAFORO !!!
	 */

uint8_t length = 1;
//char *p;
size_t xReturn = 0U;

	// Calculamos el largo del texto a escribir en la eeprom.
	// !!! IMPORTANTE
	// Por ahora, s1 es solo 1 byte.

//	p = s1;
//	while (*p != 0) {
//		p++;
//		length++;
//	}

uint8_t value;

	// El valor viene dado en un nro decimal que lo debo convertir
	value = atoi(s1);

	frtos_ioctl( fdI2C,ioctl_OBTAIN_BUS_SEMPH, NULL);
	xReturn = RTC79410_write( (uint8_t)(atoi(s0)), &value, length );
	frtos_ioctl( fdI2C,ioctl_RELEASE_BUS_SEMPH, NULL);
	return(true);
}
//-----------------------------------------------------------------------------------
bool RTC79410_test_read(char *s0, char *s1, char *s2)
{
	/* Se usa para testear desde el modo comando las funciones de leer la sram del RTC.
	 * Desde el modo comando ingresamos 2 parametros que son 2 strings: la direccion y el largo
	 */

size_t xReturn = 0U;

	frtos_ioctl( fdI2C,ioctl_OBTAIN_BUS_SEMPH, NULL);
	xReturn = RTC79410_read( (uint8_t)(atoi(s0)), s1, (uint8_t)(atoi(s2)) );
	frtos_ioctl( fdI2C,ioctl_RELEASE_BUS_SEMPH, NULL);
	return(true);
}
//-----------------------------------------------------------------------------------
void RTC_rtc2str(char *str, RtcTimeType_t *rtc)
{
	// Convierte los datos del RTC a un string con formato DD/MM/YYYY hh:mm:ss

	FRTOS_snprintf( str, 32 ,"%02d/%02d/%04d %02d:%02d:%02d\r\n",rtc->day,rtc->month, rtc->year, rtc->hour,rtc->min, rtc->sec );

}
//------------------------------------------------------------------------------------
bool RTC_str2rtc(char *str, RtcTimeType_t *rtc)
{
	// Convierto los datos de un string con formato YYMMDDhhmm a RTC

char dateTimeStr[11];
char tmp[3];
bool retS;


	/* YYMMDDhhmm */
	if ( str == NULL )
		return(false);

		memcpy(dateTimeStr, str, 10);
		// year
		tmp[0] = dateTimeStr[0]; tmp[1] = dateTimeStr[1];	tmp[2] = '\0';
		rtc->year = 2000 + atoi(tmp);
		// month
		tmp[0] = dateTimeStr[2]; tmp[1] = dateTimeStr[3];	tmp[2] = '\0';
		rtc->month = atoi(tmp);
		// day of month
		tmp[0] = dateTimeStr[4]; tmp[1] = dateTimeStr[5];	tmp[2] = '\0';
		rtc->day = atoi(tmp);
		// hour
		tmp[0] = dateTimeStr[6]; tmp[1] = dateTimeStr[7];	tmp[2] = '\0';
		rtc->hour = atoi(tmp);
		// minute
		tmp[0] = dateTimeStr[8]; tmp[1] = dateTimeStr[9];	tmp[2] = '\0';
		rtc->min = atoi(tmp);

		return(retS);

}
//------------------------------------------------------------------------------------
// FUNCIONES DE ALARMA
// Solo se implementan para la ALARMA_0
//-----------------------------------------------------------------------------------
void RTC79410_alarm0_reset(void)
{
	// Borra la flag ALM0IF del registro ALM0WKDAY de modo de limpiar la condicion
	// que genera la alarma y el pin MFP cambie de estado

size_t xReturn = 0U;
uint8_t alm_register;

	frtos_ioctl( fdI2C,ioctl_OBTAIN_BUS_SEMPH, NULL);

	xReturn = RTC79410_read( RTC79410_ALM0WKDAY, &alm_register, 1 );
	alm_register &= 0xF7;	// Pongo el bit 3 (IF) en 0.
	xReturn = RTC79410_write( RTC79410_ALM0WKDAY, &alm_register, 1 );

	frtos_ioctl( fdI2C,ioctl_RELEASE_BUS_SEMPH, NULL);

}
//------------------------------------------------------------------------------------
void RTC79410_alarm0_set( char *secs)
{
	// Configura la alarma 0 para activarse reiteradamente en secs.

size_t xReturn = 0U;
uint8_t alm_register;

	frtos_ioctl( fdI2C,ioctl_OBTAIN_BUS_SEMPH, NULL);

	// Configuro para que ponga un 1 el la salida cuando haga matching de segundos.
	alm_register = 0x81;
	xReturn = RTC79410_write( RTC79410_ALM0WKDAY, &alm_register, 1 );
	// Escribo en que segundos quiero el matching
	alm_register = atoi(secs);
	xReturn = RTC79410_write( RTC79410_ALM0SEC, &alm_register, 1 );
	// Habilito la alarma
	xReturn = RTC79410_read( RTC79410_CONTROL, &alm_register, 1 );
	alm_register = 0x10;	// Pongo el bit 3 (IF) en 0.
	xReturn = RTC79410_write( RTC79410_CONTROL, &alm_register, 1);

	frtos_ioctl( fdI2C,ioctl_RELEASE_BUS_SEMPH, NULL);


}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static char pv_dec2bcd(char num)
{
	// Convert Decimal to Binary Coded Decimal (BCD)
	return ((num/10 * 16) + (num % 10));
}
//------------------------------------------------------------------------------------
static char pv_bcd2dec(char num)
{
	// Convert Binary Coded Decimal (BCD) to Decimal
	return ((num/16 * 10) + (num % 16));
}
//------------------------------------------------------------------------------------
