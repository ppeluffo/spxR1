/*
 * uarts_sp5K.h
 *
 *  Created on: 15 de nov. de 2016
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_UARTS_H_
#define SRC_SPX_LIBS_L_UARTS_H_

#include "l_iopines.h"

//------------------------------------------------------------------------------------
int USB_write( const char *pvBuffer, const uint16_t xBytes );
int USB_read( char *pvBuffer, const uint16_t xBytes );
void USB_flushRX(void);
void USB_flushTX(void);
//------------------------------------------------------------------------------------
int GPRS_write( const char *pvBuffer, const uint16_t xBytes );
int GPRS_read( char *pvBuffer, const uint16_t xBytes );
void GPRS_flushRX(void);
void GPRS_flushTX(void);

//------------------------------------------------------------------------------------
// Las funciones CMD leen y escriben desde la terminal ( puertos USB/BT indiferentemente )
int CMD_write( const char *pvBuffer, const uint16_t xBytes );
int CMD_read( char *pvBuffer, const uint16_t xBytes );
void CMD_writeChar (unsigned char c);

//------------------------------------------------------------------------------------

#endif /* SRC_SPX_LIBS_L_UARTS_H_ */
