/*
 * spxR1_tkData.c
 *
 *  Created on: 6 de dic. de 2017
 *      Author: pablo
 */

#include "spx.h"

// Este factor es porque la resistencia shunt es de 7.3 por lo que con 20mA llegamos hasta 3646 y no a 4096
#define FACTOR_CORRECCION_RSHUNT	3646

//------------------------------------------------------------------------------------
// PROTOTIPOS

static bool pv_data_guardar_BD( void );
static void pv_data_signal_to_tkgprs(void);

static void pv_data_xbee_print_frame(void);
static void pv_data_xbee_print_frame_SPX(void);
static void pv_data_xbee_print_frame_SP5K(void);

static void pv_data_update_remote_channels(void);
static void pv_data_update_remote_channels_SPX(void);
static void pv_data_update_remote_channels_SP5K(void);

static void pv_data_print_frame_modo_SPX(void);
static void pv_data_print_frame_modo_SP5K(void);

// VARIABLES LOCALES
static st_data_frame pv_data_frame;
uint8_t wdg_counter_data;

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

	xprintf_P( PSTR("starting tkData..\r\n\0"));

	// Configuro los INA para promediar en 128 valores.
	pub_analog_config_INAS(CONF_INAS_AVG128);

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    // Inicializo el sistema de medida de ancho de pulsos
    pub_rangeMeter_init();

    // Al arrancar poleo a los 10s
    waiting_ticks = (uint32_t)(10) * 1000 / portTICK_RATE_MS;

    wdg_counter_data = 3;

	// loop
	for( ;; )
	{
		// El sanity chech pasa por 3 puntos.
		if ( wdg_counter_data == 3 ) {
			pub_ctl_watchdog_kick(WDG_DAT, WDG_DAT_TIMEOUT);
			wdg_counter_data = 0;
		} else {
			xprintf_P( PSTR("DATA: WDG ERROR sanity check (%d)\r\n\0"), wdg_counter_data );
		}

		vTaskDelayUntil( &xLastWakeTime, waiting_ticks ); // Da el tiempo para entrar en tickless.

		// Leo analog,digital,rtc,salvo en BD e imprimo.
		pub_data_read_frame( true );	// wdg_counter_data = 1

		// Muestro en pantalla.
		pub_data_print_frame( true );	// wdg_counter_data = 2

		if ( systemVars.xbee == XBEE_SLAVE ) {
			// En modo XBEE slave solo trasmito el frame por el xbee pero no lo salvo
			pv_data_xbee_print_frame();	// wdg_counter_data = 3
		} else {
			// Salvo en BD ( si no es el primer frame )
			if ( pv_data_guardar_BD() ) {	// wdg_counter_data = 3
				// Aviso a tkGPRS ( si estoy en modo continuo )
				pv_data_signal_to_tkgprs();
			}
		}

		// Espero un ciclo
		while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
			taskYIELD();

		waiting_ticks = (uint32_t)(systemVars.timerPoll) * 1000 / portTICK_RATE_MS;
		pub_ctl_reload_timerPoll();

		xSemaphoreGive( sem_SYSVars );

	}

}
//------------------------------------------------------------------------------------
static bool pv_data_guardar_BD(void)
{

	// Solo los salvo en la BD si estoy en modo normal.
	// En otros casos ( service, monitor_frame, etc, no.

FAT_t l_fat;
int8_t bytes_written;
static bool primer_frame = true;

	wdg_counter_data++;
	if ( wdg_counter_data != 3 )
		xprintf_P( PSTR("DATA: WDG guardar_en_BD 3->(%d)\r\n\0"),wdg_counter_data);

	// Para no incorporar el error de los contadores en el primer frame no lo guardo.
	if ( primer_frame ) {
		primer_frame = false;
		return(false);
	}

	// Guardo en BD
	bytes_written = FF_writeRcd( &pv_data_frame, sizeof(st_data_frame) );

	if ( bytes_written == -1 ) {
		// Error de escritura o memoria llena ??
		xprintf_P(PSTR("DATA: WR ERROR (%d)\r\n\0"),FF_errno() );
		// Stats de memoria
		FAT_read(&l_fat);
		xprintf_P( PSTR("DATA: MEM [wr=%d,rd=%d,del=%d]\0"), l_fat.wrPTR,l_fat.rdPTR, l_fat.delPTR );
		return(false);
	}

	return(true);

}
//------------------------------------------------------------------------------------
static void pv_data_signal_to_tkgprs(void)
{
	// Aviso a tkGprs que hay un frame listo. En modo continuo lo va a trasmitir enseguida.
	if ( ! MODO_DISCRETO ) {
		while ( xTaskNotify(xHandle_tkGprsRx, TK_FRAME_READY , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}
	}
}
//------------------------------------------------------------------------------------
static void pv_data_xbee_print_frame(void)
{
	// Imprime el frame actual por el xbee xcom.

	switch(systemVars.modo) {
	case MODO_SP5K:
		pv_data_xbee_print_frame_SP5K();
		break;
	case MODO_SPX:
		pv_data_xbee_print_frame_SPX();
		break;
	default:
		pv_data_xbee_print_frame_SP5K();
		break;
	}

	wdg_counter_data++;
	if ( wdg_counter_data != 3 )
		xprintf_P( PSTR("DATA: WDG xbee_print_frame 3->(%d)\r\n\0"),wdg_counter_data);
}
//------------------------------------------------------------------------------------
static void pv_data_xbee_print_frame_SPX(void)
{
uint8_t channel;
uint8_t count = 0;

	// Valores analogicos
	// Solo muestro los que tengo configurados.

	if ( systemVars.debug ==  DEBUG_XBEE ) {
		xprintf_P( PSTR("XBEE_SENT> \0") );
	}

	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		if ( ! strcmp ( systemVars.an_ch_name[channel], "X" ) )
			continue;

		if ( count != 0 ) {
			// El primer canal no lleva el separador, el resto si.
			xCom_printf_P( fdXBEE, PSTR(","));
			if ( systemVars.debug ==  DEBUG_XBEE ) {
				xprintf_P( PSTR(","));
			}
		}

		xCom_printf_P( fdXBEE, PSTR("%s=%.02f"),systemVars.an_ch_name[channel],pv_data_frame.analog_frame.mag_val[channel] );
		if ( systemVars.debug ==  DEBUG_XBEE ) {
			xprintf_P( PSTR("%s=%.02f"),systemVars.an_ch_name[channel],pv_data_frame.analog_frame.mag_val[channel] );
		}

		count++;
	}

	// Valores digitales. Lo que mostramos depende de lo que tenemos configurado
	// Niveles logicos.
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		// Si el canal no esta configurado no lo muestro.
		if ( ! strcmp ( systemVars.d_ch_name[channel], "X" ) ) {
			continue;
		}

		if ( count != 0 ) {
			// El primer canal no lleva el separador, el resto si.
			xCom_printf_P( fdXBEE, PSTR(","));
			if ( systemVars.debug ==  DEBUG_XBEE ) {
				xprintf_P( PSTR(","));
			}
		}

		// Level ?
		if ( systemVars.d_ch_type[channel] == 'L') {
			xCom_printf_P( fdXBEE, PSTR("%s=%d"),systemVars.d_ch_name[channel],pv_data_frame.digital_frame.level[channel] );
			if ( systemVars.debug ==  DEBUG_XBEE ) {
				xprintf_P( PSTR("%s=%d"),systemVars.d_ch_name[channel],pv_data_frame.digital_frame.level[channel] );
			}
		} else {
		// Counter ?
			xCom_printf_P( fdXBEE, PSTR("%s=%.02f"),systemVars.d_ch_name[channel],pv_data_frame.digital_frame.magnitud[channel] );
			if ( systemVars.debug ==  DEBUG_XBEE ) {
				xprintf_P( PSTR("%s=%.02f"),systemVars.d_ch_name[channel],pv_data_frame.digital_frame.magnitud[channel] );
			}
		}

		count++;
	}

	// Range Meter
	if ( systemVars.rangeMeter_enabled == modoRANGEMETER_ON ) {

		if ( count != 0 ) {
			// El primer canal no lleva el separador, el resto si.
			xCom_printf_P( fdXBEE, PSTR(","));
			if ( systemVars.debug ==  DEBUG_XBEE ) {
				xprintf_P( PSTR(","));
			}
		}

		xCom_printf_P( fdXBEE, PSTR("PW=%d"), pv_data_frame.range );
		if ( systemVars.debug ==  DEBUG_XBEE ) {
			xprintf_P( PSTR("PW=%d"), pv_data_frame.range );
		}
	}

	// TAIL
	xCom_printf_P( fdXBEE, PSTR("\r\n\0") );
	if ( systemVars.debug ==  DEBUG_XBEE ) {
		xprintf_P( PSTR("\r\n\0"));
	}

}
//------------------------------------------------------------------------------------
static void pv_data_xbee_print_frame_SP5K(void)
{

	uint8_t i;

	if ( systemVars.debug ==  DEBUG_XBEE ) {
		xprintf_P( PSTR("XBEE_SENT> \0") );
	}
	// Canales analogicos
	for ( i = 0; i < 3; i++) {
		xCom_printf_P( fdXBEE, PSTR("%s=%.02f,"),systemVars.an_ch_name[i],pv_data_frame.analog_frame.mag_val[i] );
		// DEBUG & LOG
		if ( systemVars.debug ==  DEBUG_XBEE ) {
			xprintf_P( PSTR("%s=%.02f,"),systemVars.an_ch_name[i],pv_data_frame.analog_frame.mag_val[i] );
		}
	}

	// Canales digitales
	xCom_printf_P( fdXBEE, PSTR("%s=%.02f,"),systemVars.d_ch_name[1],pv_data_frame.digital_frame.magnitud[1] );
	xCom_printf_P( fdXBEE, PSTR("%s=%.02f"),systemVars.d_ch_name[2],pv_data_frame.digital_frame.magnitud[2] );
	// DEBUG & LOG
	if ( systemVars.debug ==  DEBUG_XBEE ) {
		xprintf_P( PSTR("%s=%.02f,"),systemVars.d_ch_name[1],pv_data_frame.digital_frame.magnitud[1] );
		xprintf_P( PSTR("%s=%.02f"),systemVars.d_ch_name[2],pv_data_frame.digital_frame.magnitud[2] );
	}

	// Range Meter
	if ( systemVars.rangeMeter_enabled == modoRANGEMETER_ON ) {
		xCom_printf_P( fdXBEE, PSTR(",PW=%d"), pv_data_frame.range );
		if ( systemVars.debug ==  DEBUG_XBEE ) {
			xprintf_P( PSTR(",PW=%d"), pv_data_frame.range );
		}

	}

	// TAIL
	xCom_printf_P( fdXBEE, PSTR("\r\n\0") );
	if ( systemVars.debug ==  DEBUG_XBEE ) {
		xprintf_P( PSTR("\r\n\0") );
	}

}
//------------------------------------------------------------------------------------
static void pv_data_update_remote_channels(void)
{
	// Para los canales que estan configurados como remotos, el valor lo leo
	// del frame que trajo el XBEE.

	switch(systemVars.modo) {
	case MODO_SP5K:
		pv_data_update_remote_channels_SP5K();
		break;
	case MODO_SPX:
		pv_data_update_remote_channels_SPX();
		break;
	default:
		pv_data_update_remote_channels_SP5K();
		break;
	}

}
//------------------------------------------------------------------------------------
static void pv_data_update_remote_channels_SPX(void)
{
uint8_t channel;
st_remote_values *rv;

	// Leo los valores
	rv = pub_xbee_get_remote_values_ptr();

	// Canales analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++ ) {
		if ( systemVars.a_ch_modo[channel] == 'R') {
			pv_data_frame.analog_frame.mag_val[channel] = rv->analog_val[channel];
			rv->analog_val[channel] = 0; // Leo una vez y pongo en 0 para detectar problemas
		}
	}

	// Canales digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
		if ( systemVars.d_ch_modo[channel] == 'R') {
			if ( systemVars.d_ch_type[channel] == 'L') {
				pv_data_frame.digital_frame.level[channel] = (uint8_t)(rv->digital_val[channel]);
			} else {
				pv_data_frame.digital_frame.magnitud[channel] = rv->digital_val[channel];
			}
			rv->digital_val[channel] = 0; // Leo una vez y pongo en 0 para detectar problemas
		}
	}
}
//------------------------------------------------------------------------------------
static void pv_data_update_remote_channels_SP5K(void)
{

uint8_t channel;
st_remote_values *rv;

	// Leo los valores
	rv = pub_xbee_get_remote_values_ptr();


	// Canales analogicos
	for ( channel = 0; channel < 3; channel++ ) {
		if ( systemVars.a_ch_modo[channel] == 'R') {
			pv_data_frame.analog_frame.mag_val[channel] = rv->analog_val[channel];
			rv->analog_val[channel] = 0; // Leo una vez y pongo en 0 para detectar problemas
		}
	}

	// Canales digitales:
	if ( systemVars.d_ch_modo[1] == 'R') {
		pv_data_frame.digital_frame.magnitud[1] = rv->digital_val[1];
		rv->digital_val[1] = 0; // Leo una vez y pongo en 0 para detectar problemas
	}

	if ( systemVars.d_ch_modo[2] == 'R') {
		pv_data_frame.digital_frame.magnitud[2] = rv->digital_val[2];
		rv->digital_val[2] = 0; // Leo una vez y pongo en 0 para detectar problemas
	}

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void pub_data_read_frame(bool wdg_control)
{

int8_t xBytes;

	// Funcion usada para leer los datos de todos los modulos, guardarlos en memoria
	// e imprimirlos.
	// La usa por un lado tkData en forma periodica y desde el cmd line cuando se
	// da el comando read frame.

	// Leo los canales analogicos.
	// Prendo los sensores, espero un settle time de 1s, los leo y apago los sensores.
	ACH_prender_12V();
	pub_analog_config_INAS(CONF_INAS_AVG128);	// Saco a los INA del modo pwr_down

	// Normalmente espero 1s de settle time que esta bien para los sensores
	// pero cuando hay un caudalimetro de corriente, necesita casi 5s
	// vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	vTaskDelay( ( TickType_t)( ( 1000 * systemVars.pwr_settle_time ) / portTICK_RATE_MS ) );
	pub_analog_read_frame( &pv_data_frame.analog_frame);
	pub_analog_config_INAS(CONF_INAS_SLEEP);	// Pongo a los INA a dormir.
	ACH_apagar_12V();

	// Leo la bateria
	pub_analog_read_battery ( &pv_data_frame.battery );

	// Leo los canales digitales y borro los contadores.
	pub_digital_read_frame( &pv_data_frame.digital_frame, true );

	// Actualizo los canales remotos que manda el xbee.
	pv_data_update_remote_channels();

	// Agrego el timestamp
	xBytes = RTC_read_dtime( &pv_data_frame.rtc);
	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:RTC:pub_data_read_frame\r\n\0"));

	// Leo el ancho de pulso ( rangeMeter ). Demora 5s.
	if ( systemVars.rangeMeter_enabled ==  modoRANGEMETER_ON ) {
		pub_rangeMeter_ping( &pv_data_frame.range);
	}

	if ( wdg_control ) {
		wdg_counter_data++;
		if ( wdg_counter_data != 1 )
			xprintf_P( PSTR("DATA: WDG read_frame 1->(%d)\r\n\0"),wdg_counter_data);
	}

}
//------------------------------------------------------------------------------------
void pub_data_print_frame(bool wdg_control)
{
	// Imprime el frame actual en consola

	// HEADER
	xprintf_P(PSTR("frame: " ) );
	// timeStamp.
	xprintf_P(PSTR("%04d%02d%02d,"),pv_data_frame.rtc.year,pv_data_frame.rtc.month,pv_data_frame.rtc.day );
	xprintf_P(PSTR("%02d%02d%02d"),pv_data_frame.rtc.hour,pv_data_frame.rtc.min, pv_data_frame.rtc.sec );

	switch ( systemVars.modo ) {
	case MODO_SP5K:
		pv_data_print_frame_modo_SP5K();
		break;
	case MODO_SPX:
		pv_data_print_frame_modo_SPX();
		break;
	default:
		pv_data_print_frame_modo_SP5K();
		break;
	}

	// TAIL
	xprintf_P(PSTR("\r\n\0") );

	if ( wdg_control ) {
		wdg_counter_data++;
		if ( wdg_counter_data != 2 )
			xprintf_P( PSTR("DATA: WDG print_frame 2->(%d)\r\n\0"),wdg_counter_data);
	}
}
//------------------------------------------------------------------------------------
static void pv_data_print_frame_modo_SPX(void)
{

uint8_t channel;

	// Valores analogicos
	// Solo muestro los que tengo configurados.
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		if ( ! strcmp ( systemVars.an_ch_name[channel], "X" ) )
			continue;

		xprintf_P(PSTR(",%s=%.02f"),systemVars.an_ch_name[channel],pv_data_frame.analog_frame.mag_val[channel] );
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
			xprintf_P(PSTR(",%s=%d"),systemVars.d_ch_name[channel],pv_data_frame.digital_frame.level[channel] );
		} else {
		// Counter ?
			xprintf_P(PSTR(",%s=%.02f"),systemVars.d_ch_name[channel],pv_data_frame.digital_frame.magnitud[channel] );
		}
	}

	if ( systemVars.rangeMeter_enabled == modoRANGEMETER_ON ) {
		xprintf_P(PSTR(",DIST=%d\0"), pv_data_frame.range );
	}

	// bateria
	xprintf_P(PSTR(",BAT=%.02f"), pv_data_frame.battery );

}
//------------------------------------------------------------------------------------
static void pv_data_print_frame_modo_SP5K(void)
{

	// Este modo es compatible con el protocolo sp5K.
	// Considero solo 3 canales analogicos y 2 canales digitales como contadores

uint8_t channel;

	// Valores analogicos: Solo muestro los 3 primeros.
	for ( channel = 0; channel < 3; channel++) {
		xprintf_P(PSTR(",%s=%.02f"),systemVars.an_ch_name[channel],pv_data_frame.analog_frame.mag_val[channel] );
	}

	// Valores digitales. Solo muestro los canales contadores ( 1,2)
	xprintf_P(PSTR(",%s=%.02f"),systemVars.d_ch_name[1],pv_data_frame.digital_frame.magnitud[1] );
	xprintf_P(PSTR(",%s=%.02f"),systemVars.d_ch_name[2],pv_data_frame.digital_frame.magnitud[2] );

	if ( systemVars.rangeMeter_enabled == modoRANGEMETER_ON ) {
		xprintf_P(PSTR(",DIST=%d\0"), pv_data_frame.range );
	}

	// bateria
	xprintf_P(PSTR(",BAT=%.02f"), pv_data_frame.battery );

}
//------------------------------------------------------------------------------------
