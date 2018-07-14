/*
 * spx_digital.c
 *
 *  Created on: 16 de mar. de 2018
 *      Author: pablo
 *
 *          Entradas	 | Nivel o contador   |    Contador
 * ----------------------|--------------------|----------------
 * JP31,JP30   JP20,JP21 |	D3 (PA0),D0 (PB7) |	D2(PA2),D1(PB2)
 *   1	        1	            X	               X
 *   1	        3	        JP13,JP14	       JP16,JP15
 *   3	        1	        JP16,JP15	       JP13,JP14
 *   3	        3	            X	               X
 *
 * El micro en las líneas 2 ( PA2) es donde puede generar interrupciones
 * que lo despiertan del modo sleep, por lo tanto son solo esas líneas que
 * utilizamos para generar las interrupciones y funcionar como contadores.
 * En el firmware, el funcionamiento que implementamos es:
 * Leemos el nivel lógico de las 4 entradas (D0,D1,D2,D3) en cada ciclo
 * al leer un frame.
 * En D1(PB2) y D2(PA2) activamos las interrupciones por flanco que cuentan
 * pulsos y representan a los contadores CNT0 y CNT1.
 *
 * En el datalogger SPX tenemos 2 grupos de entradas digitales.
 * Las contabilizamos de acuerdo al orden de los conectores externos para que
 * las misma sea coherente desde el punto de vista del tecnico.
 * D0: JP15
 * D1: JP14
 * D2: JP13
 * D3: JP16.
 *
 * Los 2 grupos son: DO,D1 y D2,D3.
 *
 * Grupo 1: D0(JP15) y D1(JP14).
 * Pueden configurarse para que sean ambos medidas de nivel o para que D0 mida nivel y D1 contador.
 * - Configuracion A: D0 level, D1 pulsos
 *   D0: JP15->jp30(3,2) : PB7 (level)
 *   D1: JP14->jp21(1,2) : PB2 (pulsos interrupcion)
 *
 * - Configuracion B: D0 level, D1 level
 *   D0: JP15->jp21(3,2) : PB2 (level)
 *   D1: JP14->jp30(1,2) : PB7 (level)
 *
 * Grupo 2: D2(JP13) y D3(JP16).
 * Pueden configurarse para que sean ambos medidas de nivel o para que D2 mida nivel y D3 contador.
 * - Configuracion C: D2 level, D3 level
 *   D2: JP13->jp31(1,2) : PA0 (level)
 *   D3: JP16->jp20(3,2) : PA2 (level)
 *
 * - Configuracion D: D2 pulsos, D2 level
 *   D2: JP13->jp20(1,2) : PA2 (pulsos interrupcion)
 *   D3: JP16->jp31(3,2) : PA0 (level)
 *
 */

#include "spx.h"

BaseType_t xHigherPriorityTaskWokenDigital = pdFALSE;

static st_digital_frame digital_frame;

static bool wakeup_for_C0, wakeup_for_C1;

static void pv_tkDigital_init(void);

// La tarea puede estar hasta 10s en standby
#define WDG_DIN_TIMEOUT	30

//------------------------------------------------------------------------------------
void tkDigital(void * pvParameters)
{

( void ) pvParameters;
uint32_t ulNotificationValue;
const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 10000 );


	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	pv_tkDigital_init();

	xprintf_P( PSTR("starting tkDigital..\r\n\0"));

	// loop
	for( ;; )
	{

		pub_ctl_watchdog_kick(WDG_DIN, WDG_DIN_TIMEOUT);

		// Cuando la interrupcion detecta un flanco, solo envia una notificacion
		// Espero que me avisen. Si no me avisaron en 10s salgo y repito el ciclo.
		// Esto es lo que me permite entrar en tickless.
		ulNotificationValue = ulTaskNotifyTake( pdFALSE, xMaxBlockTime );

		if( ulNotificationValue != 0 ) {
			// Fui notificado: llego algun flanco que determino.

			if ( wakeup_for_C0 ) {
				// El contador C0 solo puede estar en D1
				digital_frame.counter[1]++;
				wakeup_for_C0 = false;
			}

			if ( wakeup_for_C1 ) {
				// El contador C1 solo puede estar en D2
				digital_frame.counter[2]++;
				wakeup_for_C1 = false;
			}

			if ( systemVars.debug == DEBUG_DIGITAL) {
				xprintf_P( PSTR("DIGITAL: C0=%d,C1=%d\r\n\0"),digital_frame.counter[1],digital_frame.counter[2]);
			}
			// Espero 100ms de debounced
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

			IO_clr_CLRD();		// Borro el latch llevandolo a 0.
			IO_set_CLRD();		// Lo dejo en reposo en 1

		} else   {
			// Expiro el timeout de la tarea. Por ahora no hago nada.
		}
	}
}
//------------------------------------------------------------------------------------
static void pv_tkDigital_init(void)
{
	// Configuracion inicial de la tarea

uint8_t counter;

	// Configuracion de las interrupciones que genera el contador
	// PA2, PB2.
	// Los pines ya estan configurados como entradas.
	//
	PORTA.PIN2CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_RISING_gc;	// Sensa rising edge
	PORTA.INT0MASK = PIN2_bm;
	PORTA.INTCTRL = PORT_INT0LVL0_bm;

	PORTB.PIN2CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_RISING_gc;	// Sensa rising edge
	PORTB.INT0MASK = PIN2_bm;
	PORTB.INTCTRL = PORT_INT0LVL0_bm;

	wakeup_for_C0 = false;
	wakeup_for_C1 = false;

	for ( counter = 0; counter < NRO_DIGITAL_CHANNELS; counter++) {
		digital_frame.counter[ counter ] = 0;
	}

	IO_clr_CLRD();	// Borro el latch llevandolo a 0.
	IO_set_CLRD();	// Lo dejo en reposo en 1

}
//------------------------------------------------------------------------------------
ISR(PORTA_INT0_vect)
{
	// Esta ISR se activa cuando el contador D2 (PA2) genera un flaco se subida.
	// Solo avisa a la tarea principal ( que esta dormida ) que se levante y cuente
	// el pulso y haga el debounced.
	// Dado que los ISR de los 2 contadores son los que despiertan a la tarea, debo
	// indicarle de donde proviene el llamado
	wakeup_for_C1 = true;
	vTaskNotifyGiveFromISR( xHandle_tkDigital , &xHigherPriorityTaskWokenDigital );
	PORTA.INTFLAGS = PORT_INT0IF_bm;
}
//------------------------------------------------------------------------------------
ISR(PORTB_INT0_vect)
{
	// Esta ISR se activa cuando el contador D1 (PB2) genera un flaco se subida.
	// Solo avisa a la tarea principal ( que esta dormida ) que se levante y cuente
	// el pulso y haga el debounced.
	// Dado que los ISR de los 2 contadores son los que despiertan a la tarea, debo
	// indicarle de donde proviene el llamado
	wakeup_for_C0 = true;
	vTaskNotifyGiveFromISR( xHandle_tkDigital , &xHigherPriorityTaskWokenDigital );
	PORTB.INTFLAGS = PORT_INT0IF_bm;
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void pub_digital_read_frame( st_digital_frame * dframe, bool reset_counters )
{

	// Esta funcion la invoca tkData al completar un frame para agregar los datos
	// digitales.
	// Leo los niveles de las entradas digitales y copio a dframe.
	// Respecto de los contadores, no leemos pulsos sino magnitudes por eso antes lo
	// convertimos a la magnitud correspondiente.

uint8_t i;

	// Leo el valor de las entradas digitales. ( Niveles )
	// Depende de como esten configurados
	// Grupo 1: D0(JP15) y D1(JP14).
	if ( systemVars.d_ch_type[1] == 'L') {
		// Grupo 1 esta para medir niveles en ambas entradas D0,D1
		digital_frame.level[0] = IO_read_PB2();		// D0: JP15->jp21(3,2) : PB2 (level)
		digital_frame.level[1] = IO_read_PB7();		// D1: JP14->jp30(1,2) : PB7 (level)
	} else {
		// Grupo 1 esta para medir niveles en D0 y contar pulsos en D1
		digital_frame.level[0] = IO_read_PB7();		// D0: JP15->jp30(3,2) : PB7 (level)
		digital_frame.level[1] = IO_read_PB2();		// D1: JP14->jp21(1,2) : PB2 (pulsos interrupcion)
	}

	// Grupo 2: D2(JP13) y D3(JP16).
	if ( systemVars.d_ch_type[2] == 'L') {
		// Grupo 2 esta para medir niveles en ambas entradas D2,D3
		digital_frame.level[2] = IO_read_PA0();		// D2: JP13->jp31(1,2) : PA0 (level)
		digital_frame.level[3] = IO_read_PA2();		// D3: JP16->jp20(3,2) : PA2 (level)
	} else {
		// Grupo 2 esta para contar pulsos en D2 y medir niveles en D3
		digital_frame.level[2] = IO_read_PA2();		// D2: JP13->jp20(1,2) : PA2 (pulsos interrupcion)
		digital_frame.level[3] = IO_read_PA0();		// D3: JP16->jp31(3,2) : PA0 (level)
	}

	// Convierto los contadores a las magnitudes (todos, por ahora no importa cuales son contadores )
	for (i = 0; i < NRO_DIGITAL_CHANNELS; i++) {
		digital_frame.magnitud[i] = digital_frame.counter[i] * systemVars.d_ch_magpp[i];
	}

	// Copio el resultado
	memcpy(dframe, &digital_frame, sizeof(st_digital_frame) );

	// Borro los contadores para iniciar un nuevo ciclo.
	if ( reset_counters ) {
		for (i = 0; i < NRO_DIGITAL_CHANNELS; i++) {
			digital_frame.counter[i] = 0.0;
		}
	}

}
//------------------------------------------------------------------------------------
void pub_digital_load_defaults(void)
{

	// Realiza la configuracion por defecto de los canales digitales.

	// Nombres
	snprintf_P( systemVars.d_ch_name[0], PARAMNAME_LENGTH, PSTR("L0\0") );
	snprintf_P( systemVars.d_ch_name[1], PARAMNAME_LENGTH, PSTR("C0\0") );
	snprintf_P( systemVars.d_ch_name[2], PARAMNAME_LENGTH, PSTR("C1\0") );
	snprintf_P( systemVars.d_ch_name[3], PARAMNAME_LENGTH, PSTR("L1\0") );

	// Tipos
	systemVars.d_ch_type[0] = 'L';	// JP15, D0 level
	systemVars.d_ch_type[1] = 'C';	// JP14, D1 counter
	systemVars.d_ch_type[2] = 'C';	// JP13, D2 counter
	systemVars.d_ch_type[3] = 'L';	// JP16, D3 level

	// Magnitud por pulso
	systemVars.d_ch_magpp[0] = 0.1;
	systemVars.d_ch_magpp[1] = 0.1;
	systemVars.d_ch_magpp[2] = 0.1;
	systemVars.d_ch_magpp[3] = 0.1;

	// Modo Local
	systemVars.d_ch_modo[0] = 'L';
	systemVars.d_ch_modo[1] = 'L';
	systemVars.d_ch_modo[2] = 'L';
	systemVars.d_ch_modo[3] = 'L';


}
//------------------------------------------------------------------------------------
bool pub_digital_config_channel( uint8_t channel,char *s_type, char *s_dname, char *s_magPP )
{

	// {0..3} type dname magPP

char tipo;
bool retS = false;

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	// Controlo que los parametros sean correctos.
	if ( channel > 3 ) goto EXIT;
	tipo = (char) (*s_type);

	if ( ( tipo != 'L') && (tipo != 'C') ) goto EXIT;

	// Canal 0 y 3 solo pueden ser LEVEL.
	if ( ( channel == 0 ) && (tipo != 'L') ) goto EXIT;
	if ( ( channel == 3 ) && (tipo != 'L') ) goto EXIT;

	systemVars.d_ch_type[channel] = tipo;
	snprintf_P( systemVars.d_ch_name[channel], PARAMNAME_LENGTH, PSTR("%s\0"), s_dname );
	if ( s_magPP != NULL ) { systemVars.d_ch_magpp[channel] = atof(s_magPP); }

	retS = true;

EXIT:
	xSemaphoreGive( sem_SYSVars );
	return(retS);


}
//------------------------------------------------------------------------------------
