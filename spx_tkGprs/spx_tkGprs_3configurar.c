/*
 * sp5KV5_tkGprs_configurar.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "spx_tkGprs.h"

static bool pv_gprs_CPIN(void);
static bool pv_gprs_CGREG(void);
static bool pv_gprs_CGATT(void);
static bool pv_gprs_CGACT(void);

static void pv_gprs_CSPN(void);
static void pv_gprs_CNSMOD(void);
static void pv_gprs_CCINFO(void);
static void pv_gprs_CNTI(void);
static void pg_gprs_APN(void);
static void pg_gprs_CIPMODE(void);
static void pg_gprs_DCDMODE(void);

// La tarea no puede demorar mas de 180s.
#define WDG_GPRS_TO_CONFIG	180
//------------------------------------------------------------------------------------
bool st_gprs_configurar(void)
{

	// Configuro los parametros opertativos, la banda GSM y pido una IP de modo que el
	// modem quede listo para
	// No atiendo mensajes ya que no requiero parametros operativos.
	// WATCHDOG: No demoro mas de 2 minutos en este estado

bool exit_flag = bool_RESTART;

// Entry:

	pub_ctl_watchdog_kick(WDG_GPRSTX, WDG_GPRS_TO_CONFIG);

	GPRS_stateVars.state = G_CONFIGURAR;

	xprintf_P( PSTR("GPRS: configurar.\r\n\0"));

	// Vemos que halla un pin presente.
	if ( !pv_gprs_CPIN() ) {
		exit_flag = bool_RESTART;
		goto EXIT;
	}

	// Vemos que el modem este registrado en la red
	if ( !pv_gprs_CGREG() ) {
		exit_flag = bool_RESTART;
		goto EXIT;
	}

	// Recien despues de estar registrado puedo leer la calidad de señal.
	pub_gprs_ask_sqe();

	// Me debo atachear a la red. Con esto estoy conectado a la red movil pero
	// aun no puedo trasmitir datos.
	if ( !pv_gprs_CGATT() ) {
		exit_flag = bool_RESTART;
		goto EXIT;
	}

	// Configuro parametros operativos
	//pv_gprs_CSPN();
	//pv_gprs_CNSMOD();
	//pv_gprs_CCINFO();
	//pv_gprs_CNTI();
	pg_gprs_CIPMODE();	// modo transparente.
	pg_gprs_DCDMODE();	// UART para utilizar las 7 lineas
	pg_gprs_APN();		// Configuro el APN.

	exit_flag = bool_CONTINUAR;

EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static bool pv_gprs_CPIN(void)
{
	// Chequeo que el SIM este en condiciones de funcionar.
	// AT+CPIN?

bool retS = false;

	if ( systemVars.debug == DEBUG_GPRS ) {
		xprintf_P( PSTR("GPRS: CPIN\r\n\0"));
	}

	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CPIN?\r\0"));
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

	pub_gprs_check_response("+CPIN: READY\0") ? (retS = true ): (retS = false) ;
	return(retS);

}
//------------------------------------------------------------------------------------
static bool pv_gprs_CGREG(void)
{
	/* Chequeo que el TE este registrado en la red.
	 Esto deberia ser automatico.
	 Normalmente el SIM esta para que el dispositivo se registre automaticamente
	 Esto se puede ver con el comando AT+COPS? el cual tiene la red preferida y el modo 0
	 indica que esta para registrarse automaticamente.
	 Este comando se usa para de-registrar y registrar en la red.
	 Conviene dejarlo automatico de modo que si el TE se puede registrar lo va a hacer.
	 Solo chequeamos que este registrado con CGREG.
	 AT+CGREG?
	 +CGREG: 0,1
	*/

bool retS = false;
uint8_t tryes;

	xprintf_P( PSTR("GPRS: NET registation\r\n\0"));

	for ( tryes = 0; tryes < 3; tryes++ ) {
		pub_gprs_flush_RX_buffer();
		xCom_printf_P( fdGPRS,PSTR("AT+CREG?\r\0"));
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		if ( systemVars.debug == DEBUG_GPRS ) {
			pub_gprs_print_RX_Buffer();
		}

		vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );

		pub_gprs_check_response("+CREG: 0,1\0") ? (retS = true ): (retS = false) ;

		if ( retS ) {
			break;
		}

	}
	return(retS);

}
//------------------------------------------------------------------------------------
static bool pv_gprs_CGATT(void)
{
	/* Una vez registrado, me atacheo a la red
	 AT+CGATT=1

	 AT+CGATT?
	 +CGATT: 1
	*/

bool retS = false;
uint8_t tryes;

	xprintf_P( PSTR("GPRS: NET attach\r\n\0"));

	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CGATT=1\r\0"));
	vTaskDelay( (portTickType)( 2000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

	for ( tryes = 0; tryes < 3; tryes++ ) {
		pub_gprs_flush_RX_buffer();
		xCom_printf_P( fdGPRS,PSTR("AT+CGATT?\r\0"));
		vTaskDelay( (portTickType)( 2000 / portTICK_RATE_MS ) );
		if ( systemVars.debug == DEBUG_GPRS ) {
			pub_gprs_print_RX_Buffer();
		}

		pub_gprs_check_response("+CGATT: 1\0") ? (retS = true ): (retS = false) ;

		if ( retS ) {
			break;
		}

	}
	return(retS);

}
//------------------------------------------------------------------------------------
static bool pv_gprs_CGACT(void)
{

	/*
	Incluso si GPRS Attach es exitoso, no significa que se haya establecido la llamada de datos.
  	En GPRS, un Protocolo de paquetes de datos (PDP) define la sesión de datos.
  	El contexto PDP establece la ruta de datos entre el dispositivo y el GGSN (Nodo de soporte Gateway GPRS).
  	GGSN actúa como una puerta de enlace entre el dispositivo y el resto del mundo.
  	Por lo tanto, debe establecer un contexto PDP antes de que pueda enviar / recibir datos en Internet.
  	El GGSN se identifica a través del nombre del punto de acceso (APN).
  	Cada proveedor tendrá sus propias APN y generalmente están disponibles en Internet.
  	El dispositivo puede definir múltiples contextos PDP que se almacenan de manera única
  	en los ID de contexto.

	Ahora que los contextos PDP están definidos, utilizo el contexto PDP correcto que coincida
	con la tarjeta SIM.
	Ahora para configurar la sesión, el contexto PDP apropiado debe estar activado.
	AT + CGACT = 1,1
	El primer parámetro (1) es activar el contexto y el segundo parámetro es el ID de contexto.
	Una vez que el Contexto PDP se activa con éxito, el dispositivo puede enviar / recibir
	datos en Internet.
	*/

bool retS = false;
uint8_t tryes;

	for ( tryes = 0; tryes < 5; tryes++ ) {

		pub_gprs_flush_RX_buffer();
		xCom_printf_P( fdGPRS,PSTR("AT+CGACT=1,1\r\0"));
		vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
		if ( systemVars.debug == DEBUG_GPRS ) {
			pub_gprs_print_RX_Buffer();
		}

		vTaskDelay( ( TickType_t)( 3000 / portTICK_RATE_MS ) );

		pub_gprs_check_response("+CGATT: 1\0") ? (retS = true ): (retS = false) ;

		if ( retS ) {
			break;
		}

	}
	return(retS);

}
//------------------------------------------------------------------------------------
static void pv_gprs_CSPN(void)
{

	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CSPN?\r\0"));
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

}
//------------------------------------------------------------------------------------
static void pv_gprs_CNSMOD(void)
{
	// Funcion informativa que indica el modo de red

	// AT+CNSMOD?
	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CNSMOD?\r\0"));
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

}
//------------------------------------------------------------------------------------
static void pv_gprs_CCINFO(void)
{
	// Funcion informativa que indica los datos de la celda en que esta trabajando
	// AT+CCINFO

	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CCINFO\r\0"));
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

}
//------------------------------------------------------------------------------------
static void pv_gprs_CNTI(void)
{
	// Funcion informativa que indica los datos de la celda en que esta trabajando

	// AT*CNTI?
	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT*CNTI?\r\0"));
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

}
//------------------------------------------------------------------------------------
static void pg_gprs_CIPMODE(void)
{
	// Funcion que configura el modo transparente.

	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CIPMODE=1\r\0"));
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

}
//------------------------------------------------------------------------------------
static void pg_gprs_DCDMODE(void)
{
	// Funcion que configura el UART para utilizar las 7 lineas y el DCD indicando
	// el estado del socket.

	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT&D1\r\0"));
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CSUART=1\r\0"));
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CDCDMD=0\r\0"));
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	pub_gprs_print_RX_Buffer();

	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT&C1\r\0"));
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	pub_gprs_print_RX_Buffer();


}
//------------------------------------------------------------------------------------
static void pg_gprs_APN(void)
{
	//Defino el PDP indicando cual es el APN.

	xprintf_P( PSTR("GPRS: Set APN\r\n\0") );

	// AT+CGDCONT
/*	pub_gprs_flush_RX_buffer();
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT+CGDCONT=1,\"IP\",\"%s\"\r\0"),systemVars.apn);
	FreeRTOS_write( &pdUART_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	pub_gprs_print_RX_Buffer();
*/

	// AT+CGDCONT
	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CGSOCKCONT=1,\"IP\",\"%s\"\r\0"),systemVars.apn);
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}

	// Como puedo tener varios PDP definidos, indico cual va a ser el que se deba activar
	// al usar el comando NETOPEN.
	pub_gprs_flush_RX_buffer();
	xCom_printf_P( fdGPRS,PSTR("AT+CSOCKSETPN=1\0"));
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	if ( systemVars.debug == DEBUG_GPRS ) {
		pub_gprs_print_RX_Buffer();
	}


}
//------------------------------------------------------------------------------------
