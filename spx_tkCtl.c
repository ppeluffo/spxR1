/*
 * spx_tkCtl.c
 *
 *  Created on: 4 de oct. de 2017
 *      Author: pablo
 */

#include "spx.h"

//------------------------------------------------------------------------------------
static void pv_tkCtl_init_system(void);

static void pv_tkCtl_wink_led(void);
static void pv_tkCtl_check_terminal(void);
static void pv_tkCtl_check_wdg(void);

static bool f_terminal_is_on;
static uint16_t watchdog_timers[NRO_WDGS];

// Timpo que espera la tkControl entre round-a-robin
#define TKCTL_DELAY_S	5

// La tarea pasa por el mismo lugar c/1s.
#define WDG_CTL_TIMEOUT	30

const char string_0[] PROGMEM = "CMD";
const char string_1[] PROGMEM = "CTL";
const char string_2[] PROGMEM = "DIN";
const char string_3[] PROGMEM = "DAT";
const char string_4[] PROGMEM = "OUT";
const char string_5[] PROGMEM = "GRX";
const char string_6[] PROGMEM = "GTX";
const char string_7[] PROGMEM = "XBE";

const char * const wdg_names[] PROGMEM = { string_0, string_1, string_2, string_3, string_4, string_5, string_6, string_7 };

//------------------------------------------------------------------------------------
void tkCtl(void * pvParameters)
{

( void ) pvParameters;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	xprintf_P( PSTR("\r\nstarting tkControl..\r\n\0"));

	pv_tkCtl_init_system();

	for( ;; )
	{

		pub_watchdog_kick(WDG_CTL, WDG_CTL_TIMEOUT);

		// Para entrar en tickless.
		// Cada 5s hago un chequeo de todo. En particular esto determina el tiempo
		// entre que activo el switch de la terminal y que esta efectivamente responde.
		vTaskDelay( ( TickType_t)( TKCTL_DELAY_S * 1000 / portTICK_RATE_MS ) );

		pv_tkCtl_wink_led();
		pv_tkCtl_check_terminal();
		pv_tkCtl_check_wdg();

	}
}
//------------------------------------------------------------------------------------
static void pv_tkCtl_init_system(void)
{

FAT_t l_fat;
uint16_t recSize;
uint8_t wdg;

	// Al comienzo leo este handle para asi usarlo para leer el estado de los stacks.
	// En la medida que no estoy usando la taskIdle podria deshabilitarla. !!!
	xHandle_idle = xTaskGetIdleTaskHandle();

	// Inicializo todos los watchdogs a 15s ( 3 * 5s de loop )
	for ( wdg = 0; wdg < NRO_WDGS; wdg++ ) {
		watchdog_timers[wdg] = (uint16_t)( 15 / TKCTL_DELAY_S );
	}

	// La terminal arranca apagada. Chequeo si la prendo
	f_terminal_is_on = false;
	pv_tkCtl_check_terminal();
	// Si prendi el bluetooth, lo configuro antes que nada para 115200.
	if ( f_terminal_is_on ) {
		xCom_printf_P( fdBT, PSTR("AT+BAUD8\r\n\0"));
	}

	// Leo los parametros del la EE y si tengo error, cargo por defecto
	if ( ! u_load_params_from_EE() ) {
		pub_load_defaults();
		xprintf_P( PSTR("\r\nLoading defaults !!\r\n\0"));
	}

	// Inicializo la memoria EE ( fileSysyem)
	if ( FF_open() ) {
		xprintf_P( PSTR("FSInit OK\r\n\0"));
	} else {
		FF_format(false );	// Reformateo soft.( solo la FAT )
		xprintf_P( PSTR("FSInit FAIL !!.Reformatted...\r\n\0"));
	}

	FAT_read(&l_fat);
	xprintf_P( PSTR("MEMsize=%d,wrPtr=%d,rdPtr=%d,delPtr=%d,r4wr=%d,r4rd=%d,r4del=%d \r\n\0"),FF_MAX_RCDS, l_fat.wrPTR,l_fat.rdPTR, l_fat.delPTR,l_fat.rcds4wr,l_fat.rcds4rd,l_fat.rcds4del );

	// Imprimo el tamanio de registro de memoria
	recSize = sizeof(st_data_frame);
	xprintf_P( PSTR("RCD size %d bytes.\r\n\0"),recSize);

	// Arranco el RTC. Si hay un problema lo inicializo.
	RTC79410_start();

	// Habilito a arrancar al resto de las tareas
	startTask = true;

}
//------------------------------------------------------------------------------------
static void pv_tkCtl_wink_led(void)
{
	// SI la terminal esta desconectada salgo.
	if ( IO_read_TERMCTL_PIN() == 1 )
		return;

	// Prendo los leds
	IO_set_LED_KA();
	if ( pub_modem_prendido() ) {
		IO_set_LED_COMMS();
	}

	vTaskDelay( ( TickType_t)( 5 / portTICK_RATE_MS ) );
	//taskYIELD();

	// Apago
	IO_clr_LED_KA();
	IO_clr_LED_COMMS();

}
//------------------------------------------------------------------------------------
static void pv_tkCtl_check_terminal(void)
{
	//  Controla si la terminal esta prendida o apagada.
	//  Cuando se prende, prende el bluetooth. Cuando se apaga,
	// tambien apaga el modulo.

uint8_t terminal_pin;

	terminal_pin = IO_read_TERMCTL_PIN();

	// Terminal apagada y detecto el switch on
	if ( ( terminal_pin == 0) && ( f_terminal_is_on == false ) ) {
		// Prendo bluetooth
		IO_set_BT_PWR_CTL();

		f_terminal_is_on = true;
		return;
	}

	// Terminal prendida y detecto el switch-off
	if ( ( terminal_pin == 1) && ( f_terminal_is_on == true ) ) {
		// Apago bluetooth
		IO_clr_BT_PWR_CTL();

		f_terminal_is_on = false;
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_tkCtl_check_wdg(void)
{
	// Cada tarea periodicamente reinicia su wdg timer.
	// Esta tarea los decrementa cada 5 segundos.
	// Si alguno llego a 0 es que la tarea se colgo y entonces se reinicia el sistema.

uint8_t wdg;
char buffer[10];

	// Cada ciclo reseteo el wdg para que no expire.
	WDT_Reset();
	
	// Si algun WDG no se borro, me reseteo
	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	for ( wdg = 0; wdg < NRO_WDGS; wdg++ ) {
		if ( --watchdog_timers[wdg] <= 0 ) {
			memset(buffer,'\0', 10);
			strcpy_P(buffer, (PGM_P)pgm_read_word(&(wdg_names[wdg])));
			xprintf_P( PSTR("CTL: WDG TO(%s) !!\r\n\0"),buffer);
			vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

			// Me reseteo por watchdog
			while(1)
				;

		}
	}

	xSemaphoreGive( sem_SYSVars );

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
bool pub_terminal_is_on(void)
{
	return(f_terminal_is_on);
}
//------------------------------------------------------------------------------------
void pub_watchdog_kick(uint8_t taskWdg, uint16_t timeout_in_secs )
{
	// Reinicia el watchdog de la tarea taskwdg con el valor timeout.
	// timeout es uint16_t por lo tanto su maximo valor en segundos es de 65536 ( 18hs )

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	watchdog_timers[taskWdg] = (uint16_t) ( timeout_in_secs / TKCTL_DELAY_S );

	xSemaphoreGive( sem_SYSVars );
}
//------------------------------------------------------------------------------------
void pub_print_wdg_timers(void)
{

uint8_t wdg;
char buffer[10];

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	for ( wdg = 0; wdg < NRO_WDGS; wdg++ ) {
		memset(buffer,'\0', 10);
		strcpy_P(buffer, (PGM_P)pgm_read_word(&(wdg_names[wdg])));
		xprintf_P( PSTR("%d(%s)->%d \r\n\0"),wdg,buffer,watchdog_timers[wdg]);
	}

	xSemaphoreGive( sem_SYSVars );

	xprintf_P( PSTR("\r\n\0"));

}
//------------------------------------------------------------------------------------
void pub_print_stack_watermarks(void)
{

UBaseType_t uxHighWaterMark;

	// tkIdle
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_idle );
	xprintf_P( PSTR("IDLE:%03d,%03d,[%03d]\r\n\0"),configMINIMAL_STACK_SIZE,uxHighWaterMark,(configMINIMAL_STACK_SIZE - uxHighWaterMark)) ;

	// tkCmd
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkCmd );
	xprintf_P( PSTR("CMD: %03d,%03d,[%03d]\r\n\0"),tkCmd_STACK_SIZE,uxHighWaterMark,(tkCmd_STACK_SIZE - uxHighWaterMark)) ;

	// tkControl
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkCtl );
	xprintf_P( PSTR("CTL: %03d,%03d,[%03d]\r\n\0"),tkCtl_STACK_SIZE,uxHighWaterMark, (tkCtl_STACK_SIZE - uxHighWaterMark));

	// tkDigital
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkDigital );
	xprintf_P( PSTR("DIN: %03d,%03d,[%03d]\r\n\0"),tkDigital_STACK_SIZE,uxHighWaterMark, ( tkDigital_STACK_SIZE - uxHighWaterMark));

	// tkAnalog
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkData );
	xprintf_P( PSTR("DAT: %03d,%03d,[%03d]\r\n\0"),tkData_STACK_SIZE,uxHighWaterMark, ( tkData_STACK_SIZE - uxHighWaterMark));

	// tkOutputs
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkOutputs );
	xprintf_P( PSTR("OUT: %03d,%03d,[%03d]\r\n\0"),tkOutputs_STACK_SIZE, uxHighWaterMark, ( tkOutputs_STACK_SIZE - uxHighWaterMark));

	//kGprsTX
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkGprsTx );
	xprintf_P( PSTR("GTX: %03d,%03d,[%03d]\r\n\0"),tkGprs_tx_STACK_SIZE, uxHighWaterMark, ( tkGprs_tx_STACK_SIZE - uxHighWaterMark));

	// tkGprsRX
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkGprsRx );
	xprintf_P( PSTR("GRX: %03d,%03d,[%03d]\r\n\0"),tkGprs_rx_STACK_SIZE,uxHighWaterMark, ( tkGprs_rx_STACK_SIZE - uxHighWaterMark));

	// tkXBEE
	uxHighWaterMark = uxTaskGetStackHighWaterMark( xHandle_tkXbee );
	xprintf_P( PSTR("XBE: %03d,%03d,[%03d]\r\n\0"),tkXbee_STACK_SIZE,uxHighWaterMark, ( tkXbee_STACK_SIZE - uxHighWaterMark));
	
}
//------------------------------------------------------------------------------------

