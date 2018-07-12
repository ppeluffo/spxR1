/*
 * spxR1_tkData.c
 *
 *  Created on: 6 de dic. de 2017
 *      Author: pablo
 */

#include "FRTOS-CMD.h"
#include "spx.h"

// Este factor es porque la resistencia shunt es de 7.3 por lo que con 20mA llegamos hasta 3646 y no a 4096
#define FACTOR_CORRECCION_RSHUNT	3646

//------------------------------------------------------------------------------------
// PROTOTIPOS

static bool pv_tkData_guardar_BD(st_data_frame *dframe);
static void pv_tkData_signal_to_tkgprs(void);
static void pv_tkData_xbee_print_frame(st_data_frame *dframe);
static pv_tkData_update_remote_channels(st_data_frame *dframe);

// VARIABLES LOCALES
static char data_printfBuff[CHAR256];
static st_data_frame pv_data_frame;

// La tarea pasa por el mismo lugar c/timerPoll secs.
#define WDG_DAT_TIMEOUT	 ( systemVars.timerPoll + 60 )

//------------------------------------------------------------------------------------
void tkData(void * pvParameters)
{

( void ) pvParameters;

uint32_t waiting_ticks;
TickType_t xLastWakeTime;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	FRTOS_snprintf_P( data_printfBuff,sizeof(data_printfBuff),PSTR("starting tkData..\r\n\0"));
	CMD_write(data_printfBuff, sizeof(data_printfBuff) );

	// Configuro los INA para promediar en 128 valores.
	pub_analog_config_INAS(CONF_INAS_AVG128);

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    // Inicializo el sistema de medida de ancho de pulsos
    pub_rangeMeter_init();

    // Al arrancar poleo a los 10s
    waiting_ticks = (uint32_t)(10) * 1000 / portTICK_RATE_MS;

	// loop
	for( ;; )
	{
		pub_watchdog_kick(WDG_DAT, WDG_DAT_TIMEOUT);

		vTaskDelayUntil( &xLastWakeTime, waiting_ticks ); // Da el tiempo para entrar en tickless.

		// Leo analog,digital,rtc,salvo en BD e imprimo.
		pub_tkdata_read_frame(&pv_data_frame);

		// Muestro en pantalla.
		pub_tkData_print_frame(&pv_data_frame);

		if ( systemVars.xbee == XBEE_SLAVE ) {
			// En modo XBEE slave solo trasmito el frame por el xbee pero no lo salvo
			pv_tkData_xbee_print_frame(&pv_data_frame);

		} else {

			// Salvo en BD ( si no es el primer frame )
			if ( pv_tkData_guardar_BD(&pv_data_frame) ) {
				// Aviso a tkGPRS ( si estoy en modo continuo )
				pv_tkData_signal_to_tkgprs();
			}
		}

		// Espero un ciclo
		while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
			taskYIELD();
		waiting_ticks = (uint32_t)(systemVars.timerPoll) * 1000 / portTICK_RATE_MS;
		xSemaphoreGive( sem_SYSVars );

	}

}
//------------------------------------------------------------------------------------
static bool pv_tkData_guardar_BD(st_data_frame *dframe)
{

	// Solo los salvo en la BD si estoy en modo normal.
	// En otros casos ( service, monitor_frame, etc, no.

FAT_t l_fat;
int8_t bytes_written;
static bool primer_frame = true;

	// Para no incorporar el error de los contadores en el primer frame no lo guardo.
	if ( primer_frame ) {
		primer_frame = false;
		return(false);
	}

	// Guardo en BD
	bytes_written = FF_writeRcd( dframe, sizeof(st_data_frame) );

	if ( bytes_written == -1 ) {
		// Error de escritura o memoria llena ??
		FRTOS_snprintf_P( data_printfBuff,sizeof(data_printfBuff),PSTR("DATA: WR ERROR (%d)\r\n\0"),FF_errno() );
		CMD_write(data_printfBuff, sizeof(data_printfBuff) );
		// Stats de memoria
		FAT_read(&l_fat);
		FRTOS_snprintf_P( data_printfBuff, sizeof(data_printfBuff), PSTR("DATA: MEM [wr=%d,rd=%d,del=%d]\0"), l_fat.wrPTR,l_fat.rdPTR, l_fat.delPTR );
		CMD_write(data_printfBuff, sizeof(data_printfBuff) );
		return(false);
	}

	return(true);

}
//------------------------------------------------------------------------------------
static void pv_tkData_signal_to_tkgprs(void)
{
	// Aviso a tkGprs que hay un frame listo. En modo continuo lo va a trasmitir enseguida.
	if ( ! MODO_DISCRETO ) {
		while ( xTaskNotify(xHandle_tkGprsRx, TK_FRAME_READY , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
	}
}
//------------------------------------------------------------------------------------
static void pv_tkData_xbee_print_frame(st_data_frame *dframe)
{
	// Imprime el frame actual por el xbee.

uint8_t pos;
uint8_t channel;

	pos = 0;

	// Valores analogicos
	// Solo muestro los que tengo configurados.
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		if ( ! strcmp ( systemVars.an_ch_name[channel], "X" ) )
			continue;

		pos += FRTOS_snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR("%s=%.03f,"),systemVars.an_ch_name[channel],dframe->analog_frame.mag_val[channel] );
	}

	// Valores digitales. Lo que mostramos depende de lo que tenemos configurado
	// Niveles logicos.
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		// Si el canal no esta configurado no lo muestro.
		if ( ! strcmp ( systemVars.d_ch_name[channel], "X" ) ) {
			continue;
		}
		// Level ?
		if ( systemVars.d_ch_type[channel] == 'L') {
			pos += FRTOS_snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR("%s=%d,"),systemVars.d_ch_name[channel],dframe->digital_frame.level[channel] );
		} else {
		// Counter ?
			pos += FRTOS_snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR("%s=%.02f,"),systemVars.d_ch_name[channel],dframe->digital_frame.magnitud[channel] );
		}
	}

	// Range Meter
	if ( systemVars.rangeMeter_enabled ) {
		pos += FRTOS_snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR("PW=%d"), dframe->range );
	} else {
		pos--;	// Para eliminar la ultima coma.
	}

	// TAIL
	pos += FRTOS_snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR("\r\n\0") );

	// Envio por el XBEE uart
	//FreeRTOS_write( &pdUART_XBEE, data_printfBuff, sizeof(data_printfBuff) );

}
//------------------------------------------------------------------------------------
static pv_tkData_update_remote_channels(st_data_frame *dframe)
{
	// Para los canales que estan configurados como remotos, el valor lo leo
	// del frame que trajo el XBEE.

uint8_t channel;
st_remote_values *rv;
	
	// Leo los valores
	rv = pub_xbee_get_remote_values_ptr();
	
	// Canales analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++ ) {
		if ( systemVars.a_ch_modo[channel] == 'R') {
			dframe->analog_frame.mag_val[channel] = rv->analog_val[channel];
			rv->analog_val[channel] = 0; // Leo una vez y pongo en 0 para detectar problemas
		}
	}
	
	// Canales digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
		if ( systemVars.d_ch_modo[channel] == 'R') {
			if ( systemVars.d_ch_type[channel] == 'L') {
				dframe->digital_frame.level[channel] = (uint8_t)(rv->digital_val[channel]);
			} else {
				dframe->digital_frame.magnitud[channel] = rv->digital_val[channel];
			}
			rv->digital_val[channel] = 0; // Leo una vez y pongo en 0 para detectar problemas
		}
	}
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void pub_tkdata_read_frame(st_data_frame *dframe)
{
	// Funcion usada para leer los datos de todos los modulos, guardarlos en memoria
	// e imprimirlos.
	// La usa por un lado tkData en forma periodica y desde el cmd line cuando se
	// da el comando read frame.

	// Leo los canales analogicos.
	// Prendo los sensores, espero un settle time de 1s, los leo y apago los sensores.
	pub_analog_prender_12vsensor();
	pub_analog_config_INAS(CONF_INAS_AVG128);	// Saco a los INA del modo pwr_down
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	pub_analog_read_frame( &dframe->analog_frame);
	pub_analog_config_INAS(CONF_INAS_SLEEP);	// Pongo a los INA a dormir.
	pub_analog_apagar_12vsensor();

	// Leo la bateria
	pub_analog_read_battery ( &dframe->battery );

	// Leo los canales digitales y borro los contadores.
	pub_tkDigital_read_frame( &dframe->digital_frame, true );

	pv_tkData_update_remote_channels( dframe);
	
	// Agrego el timestamp
	RTC79410_read_dtime( &dframe->rtc);

	// Leo el ancho de pulso ( rangeMeter ). Demora 5s.
	pub_rangeMeter_ping( &dframe->range);

}
//------------------------------------------------------------------------------------
void pub_tkData_print_frame(st_data_frame *dframe)
{
	// Imprime el frame actual en consola

uint8_t pos;
uint8_t channel;

	// HEADER
	pos = FRTOS_snprintf_P( data_printfBuff, sizeof(data_printfBuff), PSTR("frame: " ) );
	// timeStamp.
	pos += FRTOS_snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),dframe->rtc.year,dframe->rtc.month,dframe->rtc.day );
	pos += FRTOS_snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR("%02d%02d%02d"),dframe->rtc.hour,dframe->rtc.min, dframe->rtc.sec );

	// Valores analogicos
	// Solo muestro los que tengo configurados.
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		if ( ! strcmp ( systemVars.an_ch_name[channel], "X" ) )
			continue;

		pos += FRTOS_snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR(",%s=%.03f"),systemVars.an_ch_name[channel],dframe->analog_frame.mag_val[channel] );
	}

	// Valores digitales. Lo que mostramos depende de lo que tenemos configurado
	// Niveles logicos.
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		// Si el canal no esta configurado no lo muestro.
		if ( ! strcmp ( systemVars.d_ch_name[channel], "X" ) ) {
			continue;
		}
		// Level ?
		if ( systemVars.d_ch_type[channel] == 'L') {
			pos += FRTOS_snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR(",%s=%d"),systemVars.d_ch_name[channel],dframe->digital_frame.level[channel] );
		} else {
		// Counter ?
			pos += FRTOS_snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR(",%s=%.02f"),systemVars.d_ch_name[channel],dframe->digital_frame.magnitud[channel] );
		}
	}

	// Range Meter
	if ( systemVars.rangeMeter_enabled ) {
		pos += FRTOS_snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR(",PW=%d"), dframe->range );
	}

	// bateria
	pos += FRTOS_snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR(",BAT=%.02f"), dframe->battery );

	// TAIL
	pos += FRTOS_snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR("\r\n\0") );

	// Imprimo
	CMD_write(data_printfBuff, sizeof(data_printfBuff) );

}
//------------------------------------------------------------------------------------
st_data_frame *pub_tkData_get_data_frame_ptr(void)
{
	return(&pv_data_frame);
}
//------------------------------------------------------------------------------------
