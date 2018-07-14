/*
 * sp5KV5_tkGprs_ask_ip.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "spx_tkGprs.h"

static bool pv_gprs_netopen(void);
static void pv_gprs_read_ip_assigned(void);

// La tarea no puede demorar mas de 180s.
#define WDG_GPRS_TO_IP	180

//------------------------------------------------------------------------------------
bool st_gprs_get_ip(void)
{
	// El modem esta prendido y configurado.
	// Intento hasta 3 veces pedir la IP.
	// WATCHDOG: En el peor caso demoro 2 mins.
	// La asignacion de la direccion IP es al activar el contexto con el comando AT+CGACT

bool exit_flag = bool_RESTART;

// Entry:
	GPRS_stateVars.state = G_GET_IP;
	pub_ctl_watchdog_kick(WDG_GPRSTX, WDG_GPRS_TO_IP);

	//if ( pg_gprs_activate() ) {
	if ( pv_gprs_netopen() ) {
		exit_flag = bool_CONTINUAR;
	} else {
		// Aqui es que luego de tantos reintentos no consegui la IP.
		exit_flag = bool_RESTART;
		xprintf_P( PSTR("GPRS: ERROR: ip no asignada !!.\r\n\0") );
	}

	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static bool pv_gprs_netopen(void)
{
	// Doy el comando para atachearme a la red
	// Puede demorar unos segundos por lo que espero para chequear el resultado
	// y reintento varias veces.

uint8_t reintentos = MAX_TRYES_NET_ATTCH;
uint8_t checks;

	xprintf_P( PSTR("GPRS: netopen (get IP).\r\n\0") );

	while ( reintentos-- > 0 ) {

		// AT+NETOPEN
		// Espero 2s para dar el comando
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
		pub_gprs_flush_RX_buffer();
		if ( systemVars.debug == DEBUG_GPRS ) {
			xprintf_P( PSTR("GPRS: send NETOPEN cmd (%d)\r\n\0"),reintentos );
		}
		xCom_printf_P( fdGPRS,PSTR("AT+NETOPEN\r\0"));
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

		// Intento 3 veces ver si respondio correctamente.
		for ( checks = 0; checks < 5; checks++) {

			if ( systemVars.debug == DEBUG_GPRS ) {
				xprintf_P( PSTR("GPRS: netopen check.(%d)\r\n\0"),checks );
			}

			if ( systemVars.debug == DEBUG_GPRS ) {
				pub_gprs_print_RX_Buffer();
			}

			if ( pub_gprs_check_response("+NETOPEN: 0")) {
				xprintf_P( PSTR("GPRS: NETOPEN OK !.\r\n\0") );
				pv_gprs_read_ip_assigned();
				return(true);
			}

			if ( pub_gprs_check_response("ERROR")) {
				break;	// Salgo del for
			}

			vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );
		}

		// No pude atachearme. Debo mandar de nuevo el comando
	}

	// Luego de varios reintentos no pude conectarme a la red.
	xprintf_P( PSTR("GPRS: NETOPEN FAIL !!.\r\n\0"));
	return(false);

}
//------------------------------------------------------------------------------------
static void pv_gprs_read_ip_assigned(void)
{
	// Tengo la IP asignada: la leo para actualizar systemVars.ipaddress

char *ts = NULL;
int i=0;
char c;

	// AT+CGPADDR para leer la IP
	pub_gprs_flush_RX_buffer();
	//xCom_printf_P( fdGPRS,PSTR("AT+CGPADDR\r\0"));
	xCom_printf_P( fdGPRS,PSTR("AT+IPADDR\r\0"));
	vTaskDelay( (portTickType)( 2000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

	// Extraigo la IP del token. Voy a usar el buffer  de print ya que la respuesta
	// puede ser grande.
	memcpy(gprs_printfBuff, pub_gprs_rxbuffer_getPtr(), sizeof(gprs_printfBuff) );

	ts = strchr( gprs_printfBuff, ':');
	ts++;
	while ( (c= *ts) != '\r') {
		systemVars.dlg_ip_address[i++] = c;
		ts++;
	}
	systemVars.dlg_ip_address[i++] = '\0';

	xprintf_P( PSTR("GPRS: ip address=[%s]\r\n\0"), systemVars.dlg_ip_address);

}
//------------------------------------------------------------------------------------
