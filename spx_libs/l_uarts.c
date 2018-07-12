/*
 * uarts_sp5K.c
 *
 *  Created on: 15 de nov. de 2016
 *      Author: pablo
 */


#include "l_uarts.h"
#include "frtos-io.h"

// En las operaciones de READ NO USAMOS SEMAFOROS YA QUE no tienen poerque ser
// thread-safe ya que es una unica tarea que maneja las lecturas en forma exclusiva.
// Para usar semaforo, deberiamos tener uno especifico de RX y otro de TX !!!

//------------------------------------------------------------------------------------
int CMD_write( const char *pvBuffer, const uint16_t xBytes )
{
	// En el SPX el USB y BT operan juntos como I/O de la tarea de comando
	// Para simplificar la escritura usamos esta funcion de modo que en el programa
	// no tenemos que escribir en ambos handles.

int bytes2wr = 0;

	// SI la terminal esta desconectada salgo.
	if ( IO_read_TERMCTL_PIN() == 1 )
		return(bytes2wr);

	frtos_ioctl (fdUSB,ioctl_OBTAIN_BUS_SEMPH, NULL );
	bytes2wr = frtos_write( fdUSB, pvBuffer, xBytes );
	frtos_ioctl (fdUSB,ioctl_RELEASE_BUS_SEMPH, NULL);
	return(bytes2wr);
}
//------------------------------------------------------------------------------------
int CMD_read( char *pvBuffer, const uint16_t xBytes )
{

	// Como el USB y BT operan en paralelo para el modo comando, los caracteres pueden entrar
	// por cualquiera de los handles.
	// Lee caracteres de ambas FIFO de recepcion de la USB y BT
	// No considera el caso que los handles sean QUEUES !!!!

int bytes2rd = 0;

	// SI la terminal esta desconectada salgo.
	if ( IO_read_TERMCTL_PIN() == 1 )
		return(bytes2rd);

	// VER NOTA AL PPIO.SOBRE SEMAFOROS EN READ !!!!

	//frtos_ioctl (fdUSB,ioctlOBTAIN_BUS_SEMPH, NULL );
	bytes2rd = frtos_read( fdUSB, pvBuffer, xBytes );
	//frtos_ioctl (fdUSB,ioctlRELEASE_BUS_SEMPH, NULL);
	return(bytes2rd);

}
//------------------------------------------------------------------------------------
void CMD_writeChar (unsigned char c)
{
	// Funcion intermedia necesaria por cmdline para escribir de a 1 caracter en consola
	// El tema es que el prototipo de funcion que requiere cmdlineSetOutputFunc no se ajusta
	// al de FreeRTOS_UART_write, por lo que defino esta funcion intermedia.

char cChar;

	cChar = c;
	CMD_write( &cChar, sizeof(char));
}
//------------------------------------------------------------------------------------
// USB xCOM
//------------------------------------------------------------------------------------
int USB_write( const char *pvBuffer, const uint16_t xBytes )
{
int bytes2wr = 0;

	// SI la terminal esta desconectada salgo.
	if ( IO_read_TERMCTL_PIN() == 1 )
		return(bytes2wr);

	frtos_ioctl (fdUSB,ioctl_OBTAIN_BUS_SEMPH, NULL );
	bytes2wr = frtos_write( fdUSB, pvBuffer, xBytes );
	frtos_ioctl (fdUSB,ioctl_RELEASE_BUS_SEMPH, NULL);
	return(bytes2wr);

}
//------------------------------------------------------------------------------------
int USB_read( char *pvBuffer, const uint16_t xBytes )
{
int bytes2rd = 0;

	// SI la terminal esta desconectada salgo.
	if ( IO_read_TERMCTL_PIN() == 1 )
		return(bytes2rd);

	bytes2rd = frtos_read( fdUSB, pvBuffer, xBytes );
	return(bytes2rd);

}
//------------------------------------------------------------------------------------
void USB_flushRX(void)
{
	frtos_ioctl( fdUSB, ioctl_UART_CLEAR_RX_BUFFER, NULL );
}
//------------------------------------------------------------------------------------
void USB_flushTX(void)
{
	frtos_ioctl (fdUSB, ioctl_OBTAIN_BUS_SEMPH, NULL );
	frtos_ioctl( fdUSB, ioctl_UART_CLEAR_TX_BUFFER, NULL );
	frtos_ioctl (fdUSB, ioctl_RELEASE_BUS_SEMPH, NULL);
}
//------------------------------------------------------------------------------------
// GPRS xCOM
//------------------------------------------------------------------------------------
int GPRS_write( const char *pvBuffer, const uint16_t xBytes )
{
int bytes2wr = 0;

	frtos_ioctl (fdGPRS,ioctl_OBTAIN_BUS_SEMPH, NULL );
	bytes2wr = frtos_write( fdGPRS, pvBuffer, xBytes );
	frtos_ioctl (fdGPRS,ioctl_RELEASE_BUS_SEMPH, NULL);
	return(bytes2wr);

}
//------------------------------------------------------------------------------------
int GPRS_read( char *pvBuffer, const uint16_t xBytes )
{
int bytes2rd = 0;

	bytes2rd = frtos_read( fdGPRS, pvBuffer, xBytes );
	return(bytes2rd);

}
//------------------------------------------------------------------------------------
void GPRS_flushRX(void)
{
	frtos_ioctl( fdGPRS, ioctl_UART_CLEAR_RX_BUFFER, NULL );
}
//------------------------------------------------------------------------------------
void GPRS_flushTX(void)
{
	frtos_ioctl (fdGPRS, ioctl_OBTAIN_BUS_SEMPH, NULL );
	frtos_ioctl( fdGPRS, ioctl_UART_CLEAR_TX_BUFFER, NULL );
	frtos_ioctl (fdGPRS, ioctl_RELEASE_BUS_SEMPH, NULL);
}
//------------------------------------------------------------------------------------
