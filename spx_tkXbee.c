/*
 * spx_tkXbee.c
 *
 *  Created on: 5 de jun. de 2018
 *      Author: pablo
 */

#include "spx.h"

static char xbee_printfBuff[80];

#define UART_XBEE_RXBUFFER_LEN 128
struct {
	char buffer[UART_XBEE_RXBUFFER_LEN];
	uint16_t ptr;
	
} pv_xbeeRxCbuffer;

static void pub_xbee_flush_RX_buffer(void);
static void pv_xbee_rxbuffer_push(char c);
static void pv_xbee_parse_rxframe(void);

st_remote_values remote_val;

// La tarea puede estar hasta 10s en standby
#define WDG_XBEE_TIMEOUT	30

//------------------------------------------------------------------------------------
void tkXbee(void * pvParameters)
{

( void ) pvParameters;
char c;
uint8_t channel;

	while ( !startTask )
	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

//	FRTOS_snprintf_P( xbee_printfBuff,sizeof(xbee_printfBuff),PSTR("starting tkXbee..\r\n\0"));
//	CMD_write(xbee_printfBuff, sizeof(xbee_printfBuff) );

	// XBEE
	// Para master o slave, prendo el XBEE.
	if ( systemVars.xbee != XBEE_OFF ) {
		// Prendo el XBEE
		IO_set_XBEE_PWR();
		// EL reset debe estar en ON
		IO_set_XBEE_RESET();
		FRTOS_snprintf_P( xbee_printfBuff,sizeof(xbee_printfBuff),PSTR("Activo XBEE pwr\r\n\0"));
		CMD_write(xbee_printfBuff, sizeof(xbee_printfBuff) );
	}

	// Inicializo
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++ ) {
		remote_val.analog_val[channel] = 0;
	}
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
		remote_val.digital_val[channel] = 0;
	}
				
	for( ;; )
	{

loop:
		pub_watchdog_kick(WDG_XBEE, WDG_XBEE_TIMEOUT);
		
		// En modo slave u off no hago nada y entro en tickless
		if ( systemVars.xbee != XBEE_MASTER ) {
			vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );
			goto loop;
		}
		
//		while ( uXBEE_read( &c, 1 ) == 1 ) {
//			CMD_writeChar(c);
//			pv_xbee_rxbuffer_push(c);
//			if ( c == '\n') {
//				FRTOS_snprintf_P( xbee_printfBuff,sizeof(xbee_printfBuff),PSTR("RcvdXbee>%s"), pv_xbeeRxCbuffer.buffer );
//				CMD_write(xbee_printfBuff, sizeof(xbee_printfBuff) );
//				pv_xbee_parse_rxframe();
//				pub_xbee_flush_RX_buffer();
//			}
//		}

	}
}
//------------------------------------------------------------------------------------
static void pv_xbee_parse_rxframe(void)
{
	// El frame es del tipo HT=0.000,Q0=0.00.
	// Debo separarlo primero por el ',' y luego por el '='.
	// Con esto obtengo un para {name,valor} que lo paso a 
	// la tkData.
	
char *delim1 = ",";
char *delim2 = "=";
char *stringp;
char *token;
char *name;
double val;
uint8_t channel;

	stringp = &pv_xbeeRxCbuffer.buffer;
	while (1) {
		// Separo por ',' para obtener el primer token.
		token = strsep(&stringp,delim1);	// HT=0.000
		if ( token == NULL) {
			return;
		}
//		FRTOS_snprintf_P( xbee_printfBuff,sizeof(xbee_printfBuff),PSTR("Tk>%s\r\n\0"), token );
//		CMD_write(xbee_printfBuff, sizeof(xbee_printfBuff) );
				
		// Separo por '=' para tener el par {nombre,valor}
		name = strsep(&token,delim2);
		val = atof(token);
//		FRTOS_snprintf_P( xbee_printfBuff,sizeof(xbee_printfBuff),PSTR("NAME=%s,VAL=%.02f\r\n\0"), name,val );
//		CMD_write(xbee_printfBuff, sizeof(xbee_printfBuff) );	
		
		// Veo a que canal corresponde para asignarlo
		// Empiezo por los analogicos.
		for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++ ) {
			if ( ( systemVars.a_ch_modo[channel] == 'R') && ( strcmp(name,systemVars.an_ch_name[channel] == 0 ))) {
				remote_val.analog_val[channel] = val;
			}
		}
		// Sigo con los digitales
		for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
			if ( ( systemVars.d_ch_modo[channel] == 'R') && ( strcmp(name,systemVars.d_ch_name[channel] == 0 ) )) {
				remote_val.digital_val[channel] = val;
			}
		}
	}
}
//------------------------------------------------------------------------------------
void pub_xbee_flush_RX_buffer(void)
{

//	FreeRTOS_ioctl( &pdUART_XBEE,ioctl_UART_CLEAR_RX_BUFFER, NULL);
//	FreeRTOS_ioctl( &pdUART_XBEE,ioctl_UART_CLEAR_TX_BUFFER, NULL);

//	memset(pv_xbeeRxCbuffer.buffer,0, UART_XBEE_RXBUFFER_LEN );
	pv_xbeeRxCbuffer.ptr = 0;

}
//------------------------------------------------------------------------------------
static void pv_xbee_rxbuffer_push(char c)
{
	pv_xbeeRxCbuffer.buffer[pv_xbeeRxCbuffer.ptr] = c;
	// Avanzo en modo circular
	pv_xbeeRxCbuffer.ptr = ( pv_xbeeRxCbuffer.ptr  + 1 ) % (UART_XBEE_RXBUFFER_LEN );

}
//------------------------------------------------------------------------------------
st_remote_values *pub_xbee_get_remote_values_ptr(void)
{
	return(&remote_val);
}
//------------------------------------------------------------------------------------
