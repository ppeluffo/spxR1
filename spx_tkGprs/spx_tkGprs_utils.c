/*
 * spx_tkGprs_utils.c
 *
 *  Created on: 25 de oct. de 2017
 *      Author: pablo
 */

#include <spx_tkGprs.h>
#include "spx.h"

//------------------------------------------------------------------------------------
void pub_gprs_open_socket(void)
{
	// Envio el comando AT para abrir el socket

	if ( systemVars.debug == DEBUG_GPRS ) {
		FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: try to open socket\r\n\0"));
		CMD_write( gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	pub_gprs_flush_RX_buffer();
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT+CIPOPEN=0,\"TCP\",\"%s\",%s\r\n\0"),systemVars.server_ip_address,systemVars.server_tcp_port);
	frtos_write( fdGPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 1500 / portTICK_RATE_MS ) );

	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}
}
//------------------------------------------------------------------------------------
t_socket_status pub_gprs_check_socket_status(void)
{
	// El socket esta abierto si el modem esta prendido y
	// el DCD esta en 0.
	// Cuando el modem esta apagado pin_dcd = 0
	// Cuando el modem esta prendido y el socket cerrado pin_dcd = 1
	// Cuando el modem esta prendido y el socket abierto pin_dcd = 0.

uint8_t pin_dcd;
t_socket_status socket_status = SOCK_CLOSED;

	pin_dcd = IO_read_DCD();

	if ( ( GPRS_stateVars.modem_prendido == true ) && ( pin_dcd == 0 ) ){
	//if ( ( GPRS_stateVars.modem_prendido == true ) && ( GPRS_stateVars.dcd == 1 ) ){

		if ( systemVars.debug == DEBUG_GPRS ) {
			FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: sckt is open (dcd=%d)\r\n\0"),pin_dcd);
			CMD_write( gprs_printfBuff, sizeof(gprs_printfBuff) );
		}
		socket_status = SOCK_OPEN;

	} else {

		if ( systemVars.debug == DEBUG_GPRS ) {
			FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: sckt is close (dcd=%d)\r\n\0"),pin_dcd);
			CMD_write( gprs_printfBuff, sizeof(gprs_printfBuff) );
		}
		socket_status = SOCK_CLOSED;
	}

	return(socket_status);

}
//------------------------------------------------------------------------------------
void pub_gprs_config_timerdial ( char *s_timerdial )
{
	// El timer dial puede ser 0 si vamos a trabajar en modo continuo o mayor a
	// 15 minutos.
	// Es una variable de 32 bits para almacenar los segundos de 24hs.

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.timerDial = atoi(s_timerdial);

	// Controlo que este en los rangos permitidos
	if ( (systemVars.timerDial > 0) && (systemVars.timerDial < 900) ) {
		systemVars.timerDial = 900;
	}

	xSemaphoreGive( sem_SYSVars );
	return;
}
//------------------------------------------------------------------------------------
void pub_gprs_modem_pwr_off(void)
{
	// Apago la fuente del modem

	IO_clr_GPRS_SW();	// Es un FET que lo dejo cortado
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	IO_clr_GPRS_PWR();	// Apago la fuente.
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------------
void pub_gprs_modem_pwr_on(void)
{
	// Prendo la fuente del modem

	IO_clr_GPRS_SW();	// GPRS=0V, PWR_ON pullup 1.8V )
	IO_set_GPRS_PWR();											// Prendo la fuente ( alimento al modem ) HW
	vTaskDelay( (portTickType)( 2000 / portTICK_RATE_MS ) );	// Espero 2s que se estabilize la fuente.


}
//------------------------------------------------------------------------------------
void pub_gprs_modem_pwr_sw(void)
{
	// Genera un pulso en la linea PWR_SW. Como tiene un FET la senal se invierte.
	// En reposo debe la linea estar en 0 para que el fet flote y por un pull-up del modem
	// la entrada PWR_SW esta en 1.
	// El PWR_ON se pulsa a 0 saturando el fet.
	IO_set_GPRS_SW();	// GPRS_SW = 3V, PWR_ON = 0V.
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	IO_clr_GPRS_SW();	// GPRS_SW = 0V, PWR_ON = pullup, 1.8V

}
//------------------------------------------------------------------------------------
void pub_gprs_load_defaults(void)
{

	systemVars.timerDial = 900;

	FRTOS_snprintf_P( systemVars.apn, APN_LENGTH, PSTR("SPYMOVIL.VPNANTEL\0") );
	strncpy_P(systemVars.server_ip_address, PSTR("192.168.0.9\0"),16);
	strncpy_P(systemVars.server_tcp_port, PSTR("80\0"),PORT_LENGTH	);
	strncpy_P(systemVars.passwd, PSTR("spymovil123\0"),PASSWD_LENGTH);
	strncpy_P(systemVars.serverScript, PSTR("/cgi-bin/spx/spx.pl\0"),SCRIPT_LENGTH);


}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS DEL BUFFER DE RECEPCION DEL GPRS
//------------------------------------------------------------------------------------
char *pub_gprs_rxbuffer_getPtr(void)
{
	// Retorna la direccion de comienzo del buffer de recepcion del GPRS.
	// Esto permite su manipulacion.

	return( pv_gprsRxCbuffer.buffer);
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS DEL BUFFER DE RECEPCION DEL GPRS
//------------------------------------------------------------------------------------
void pv_gprs_rxbuffer_flush(void)
{

	memset( pv_gprsRxCbuffer.buffer, '\0', UART_GPRS_RXBUFFER_LEN);
	pv_gprsRxCbuffer.ptr = 0;

}
//------------------------------------------------------------------------------------
void pv_gprs_rxbuffer_push(char c)
{

	pv_gprsRxCbuffer.buffer[pv_gprsRxCbuffer.ptr] = c;
	// Avanzo en modo circular
	pv_gprsRxCbuffer.ptr = ( pv_gprsRxCbuffer.ptr  + 1 ) % ( UART_GPRS_RXBUFFER_LEN );
/*
	if ( c == '\r') {

		pv_gprsRxCbuffer.buffer[pv_gprsRxCbuffer.ptr] = '\n';
		pv_gprsRxCbuffer.ptr = ( pv_gprsRxCbuffer.ptr  + 1 ) % ( UART_GPRS_RXBUFFER_LEN );
	}
*/
}
//------------------------------------------------------------------------------------
void pv_gprs_init_system(void)
{
	// Aca hago toda la inicializacion que requiere el sistema del gprs.

	memset(buff_gprs_imei,'\0',IMEIBUFFSIZE);
	strncpy_P(systemVars.dlg_ip_address, PSTR("000.000.000.000\0"),16);

	// Configuracion inicial de la tarea
	// Configuro la interrupcion del DCD ( PORTD.6)
	// La variable GPRS_stateVars.dcd

	// Los pines ya estan configurados como entradas.
	//
//	PORTD.PIN2CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_BOTHEDGES_gc;	// Sensa both edge
//	PORTD.INT0MASK = PIN6_bm;
//	PORTD.INTCTRL = PORT_INT0LVL0_bm;


}
//------------------------------------------------------------------------------------
bool pub_gprs_check_response( const char *rsp )
{
bool retS = false;

	if ( strstr( pv_gprsRxCbuffer.buffer, rsp) != NULL ) {
		retS = true;
	}
	return(retS);
}
//------------------------------------------------------------------------------------
void pub_gprs_print_RX_response(void)
{
	// Imprime la respuesta del server.
	// Utiliza el buffer de RX.
	// Solo muestra el payload, es decir lo que esta entre <h1> y </h1>

	char *start, *end;
	uint8_t largo;
	uint8_t pos;

	start = strstr(pv_gprsRxCbuffer.buffer,"<h1>");
	end = strstr(pv_gprsRxCbuffer.buffer, "</h1>");

	if ( ( start != NULL ) && ( end != NULL) ) {
		start += 4;
		largo = end - start;
		memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
		pos = FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: rsp> "));
		strncpy( &gprs_printfBuff[pos], start, largo);
		FRTOS_snprintf_P( &gprs_printfBuff[pos+largo],sizeof(gprs_printfBuff),PSTR("\r\n\0"));
		CMD_write( gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
void pub_gprs_flush_RX_buffer(void)
{

	frtos_ioctl( fdGPRS,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	frtos_ioctl( fdGPRS,ioctl_UART_CLEAR_TX_BUFFER, NULL);

	memset(pv_gprsRxCbuffer.buffer,0, UART_GPRS_RXBUFFER_LEN );
	pv_gprsRxCbuffer.ptr = 0;

}
//------------------------------------------------------------------------------------
void pub_gprs_print_RX_Buffer(void)
{

	// Imprime la respuesta a un comando.
	// Utiliza el buffer de RX.

//	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: rxbuff> \r\n\0"));
//	CMD_write( gprs_printfBuff, sizeof(gprs_printfBuff) );
	// Imprimo todo el buffer de RX ( 640b). Sale por \0.
	frtos_write( fdGPRS, pv_gprsRxCbuffer.buffer, UART_GPRS_RXBUFFER_LEN );
	// Agrego un CRLF por las dudas
//	CMD_write( gprs_printfBuff, sizeof(gprs_printfBuff) );
	CMD_write( "\r\n\0", sizeof("\r\n\0") );

}
//------------------------------------------------------------------------------------
void pub_gprs_ask_sqe(void)
{
	// Veo la calidad de senal que estoy recibiendo

char csqBuffer[32];
char *ts = NULL;

	// AT+CSQ
	pub_gprs_flush_RX_buffer();
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT+CSQ\r\0"));
	frtos_write( fdGPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );

	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

	memcpy(csqBuffer, pub_gprs_rxbuffer_getPtr(), sizeof(csqBuffer) );
	if ( (ts = strchr(csqBuffer, ':')) ) {
		ts++;
		systemVars.csq = atoi(ts);
		systemVars.dbm = 113 - 2 * systemVars.csq;
	}

	// LOG & DEBUG
	FRTOS_snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GPRS: signalQ CSQ=%d,DBM=%d\r\n\0"), systemVars.csq,systemVars.dbm );
	CMD_write( gprs_printfBuff, sizeof(gprs_printfBuff) );

}
//------------------------------------------------------------------------------------
bool pub_modem_prendido(void)
{
	return(GPRS_stateVars.modem_prendido);
}
//------------------------------------------------------------------------------------
