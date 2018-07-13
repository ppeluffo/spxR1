/*
 * spx_analog.c
 *
 *  Created on: 14 de mar. de 2018
 *      Author: pablo
 */

#include "spx.h"

//------------------------------------------------------------------------------------
void pub_analog_config_INAS( uint16_t conf_reg_value )
{

	// Realiza la configuracion de los 2 INA grabando el conf_reg_value en el
	// registro de configuracion CONF (0x00)
	// conf_reg_value = 0x7927: Configuro los 2 INA para promediar en 128 medidas
	// conf_reg_value = 0x7920: Configuro los 2 INA para entrar en modo sleep down

char res[3];

	res[0] = ( conf_reg_value & 0xFF00 ) >> 8;
	res[1] = ( conf_reg_value & 0x00FF );
	INA3221_write( INA3221_id2busaddr(0), INA3231_CONF, res, 2 );
	INA3221_write( INA3221_id2busaddr(1), INA3231_CONF, res, 2 );

}
//------------------------------------------------------------------------------------
void pub_analog_load_defaults(void)
{

	// Realiza la configuracion por defecto de los canales analogicos.

uint8_t channel;

	systemVars.timerPoll = 60;

	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		systemVars.coef_calibracion[channel] = 3646;
		systemVars.imin[channel] = 0;
		systemVars.imax[channel] = 20;
		systemVars.mmin[channel] = 0;
		systemVars.mmax[channel] = 6.0;
		systemVars.a_ch_modo[channel] = 'L';	// Modo local
		snprintf_P( systemVars.an_ch_name[channel], PARAMNAME_LENGTH, PSTR("A%d\0"),channel );

	}
}
//------------------------------------------------------------------------------------
void pub_analog_config_channel( uint8_t channel,char *s_aname,char *s_imin,char *s_imax,char *s_mmin,char *s_mmax )
{

	// Configura los canales analogicos. Es usada tanto desde el modo comando como desde el modo online por gprs.

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	if ( ( channel >=  0) && ( channel < NRO_ANALOG_CHANNELS) ) {
		snprintf_P( systemVars.an_ch_name[channel], PARAMNAME_LENGTH, PSTR("%s\0"), s_aname );
		if ( s_imin != NULL ) { systemVars.imin[channel] = atoi(s_imin); }
		if ( s_imax != NULL ) { systemVars.imax[channel] = atoi(s_imax); }
		if ( s_mmin != NULL ) { systemVars.mmin[channel] = atoi(s_mmin); }
		if ( s_mmax != NULL ) { systemVars.mmax[channel] = atof(s_mmax); }
	}

	xSemaphoreGive( sem_SYSVars );
	return;

}
//------------------------------------------------------------------------------------
void pub_analog_config_timerpoll ( char *s_timerpoll )
{

	// Configura el tiempo de poleo.
	// Se utiliza desde el modo comando como desde el modo online
	// El tiempo de poleo debe estar entre 15s y 3600s

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.timerPoll = atoi(s_timerpoll);

	if ( systemVars.timerPoll < 15 )
		systemVars.timerPoll = 15;

	if ( systemVars.timerPoll > 3600 )
		systemVars.timerPoll = 300;

	xSemaphoreGive( sem_SYSVars );
	return;
}
//------------------------------------------------------------------------------------
void pub_analog_config_spanfactor ( uint8_t channel, char *s_spanfactor )
{

	// Configura el factor de correccion del span de canales delos INA.
	// Esto es debido a que las resistencias presentan una tolerancia entonces con
	// esto ajustamos que con 20mA den la excursión correcta.
	// Solo de configura desde modo comando.

uint16_t span;

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	span = atoi(s_spanfactor);
	systemVars.coef_calibracion[channel] = span;

	xSemaphoreGive( sem_SYSVars );
	return;
}
//------------------------------------------------------------------------------------
void pub_analog_prender_12vsensor ( void )
{

	// Prende la fuente que genera los 12V ( 10.5 ) para los sensores.
	IO_set_SENS_12V_CTL();

}
//------------------------------------------------------------------------------------
void pub_analog_apagar_12vsensor ( void )
{

	IO_clr_SENS_12V_CTL();
}
//------------------------------------------------------------------------------------
void pub_analog_read_channel ( uint8_t channel, uint16_t *raw_val, float *mag_val )
{

	// Lee un canal analogico y devuelve en raw_val el valor leido del conversor A/D y en
	// mag_val el valor convertido a la magnitud configurada.
	// Se utiliza desde el modo comando como desde el modulo de poleo de las entradas.

uint8_t ina_reg = 0;
uint8_t ina_id = 0;
uint16_t an_raw_val;
char res[3];
float an_mag_val;
float I,M,P;
uint16_t D;

	switch ( channel ) {
	case 0:
		ina_id = 0; ina_reg = INA3221_CH1_SHV;break;
	case 1:
		ina_id = 0; ina_reg = INA3221_CH2_SHV;break;
	case 2:
		ina_id = 0; ina_reg = INA3221_CH3_SHV;break;
	case 3:
		ina_id = 1; ina_reg = INA3221_CH2_SHV;break;
	case 4:
		ina_id = 1; ina_reg = INA3221_CH3_SHV;break;
	}

	// Leo el valor del INA.
	INA3221_read( INA3221_id2busaddr(ina_id), ina_reg, res ,2 );
	an_raw_val = 0;
	an_raw_val = ( res[0]<< 8 ) + res[1];
	an_raw_val = an_raw_val >> 3;

	*raw_val = an_raw_val;

	// Convierto el raw_value a la magnitud
	// Calculo la corriente medida en el canal
	I = (float)( an_raw_val) * 20 / ( systemVars.coef_calibracion[channel] + 1);

	// Calculo la magnitud
	P = 0;
	D = systemVars.imax[channel] - systemVars.imin[channel];

	an_mag_val = 0.0;
	if ( D != 0 ) {
		// Pendiente
		P = (float) ( systemVars.mmax[channel]  -  systemVars.mmin[channel] ) / D;
		// Magnitud
		M = (float) (systemVars.mmin[channel] + ( I - systemVars.imin[channel] ) * P);
		an_mag_val = M;

	} else {
		// Error: denominador = 0.
		an_mag_val = -999.0;
	}

	*mag_val = an_mag_val;

}
//------------------------------------------------------------------------------------
void pub_analog_read_battery ( float *mag_val )
{

	// Lee un canal analogico que corresponde a la bateria

uint8_t ina_reg, ina_id;
uint16_t an_raw_val;
float an_mag_val;
char res[3];

	// La bateria esta en el canal 1 (bus) del INA 1.
	ina_reg = INA3221_CH1_BUSV;
	ina_id = 1;
	INA3221_read( INA3221_id2busaddr(ina_id), ina_reg, res ,2 );
	an_raw_val = 0;
	an_raw_val = ( res[0]<< 8 ) + res[1];
	an_raw_val = an_raw_val >> 3;

	// Convierto el raw_value a la magnitud ( 8mV por count del A/D)
	an_mag_val = 0.008 * an_raw_val;
	*mag_val = an_mag_val;

}
//------------------------------------------------------------------------------------
void pub_analog_read_frame(st_analog_frame *analog_frame )
{

	// Lee todos los canales analogicos y los deja en la estructura st_analog_frame.

uint8_t channel;
uint16_t raw_val;
float mag_val;

	// Leo los canales analogicos de datos.
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++ ) {
		pub_analog_read_channel (channel, &raw_val, &mag_val );
		analog_frame->mag_val[channel] = mag_val;
	}

}
//------------------------------------------------------------------------------------
