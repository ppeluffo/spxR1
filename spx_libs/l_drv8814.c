/*
 * l_drv8814.c
 *
 *  Created on: 21 de oct. de 2017
 *      Author: pablo
 */

#include "../spx_libs/l_drv8814.h"
#include <ctype.h>

//------------------------------------------------------------------------------------
bool DRV8814_test_pulse(char *s0, char *s1, char *s2)
{

char channel;
char phase;
uint16_t pulse_width;

	// Prendo la fuente de 12V
//	IO_set_OUTS_12V_CTL();
//	vTaskDelay( ( TickType_t)(500 / portTICK_RATE_MS ) );

	// Determino el canal: A o B
	switch ( toupper(s0[0]) ) {
	case 'A': channel = 'A';
			break;
	case 'B': channel = 'B';
			break;
	default:
			return(false);
	}

	// Determino la fase: + o -
	switch (s1[0]) {
	case '+': phase = '+';
			break;
	case '-': phase = '-';
			break;
	default:
			return(false);
	}

	pulse_width = atoi(s2);

	DRV8814_pulse(channel, phase, pulse_width);

	vTaskDelay( ( TickType_t)(500 / portTICK_RATE_MS ) );
	// Apago la fuente de 12V
//	IO_clr_OUTS_12V_CTL();
	return(true);
}
//------------------------------------------------------------------------------------
void DRV8814_pulse(char channel, char phase, uint16_t pulse_width)
{
	// channel_id: puede ser 0 ( valvula conectada a la salida 0, A1,A2) o 1 ( B1, B2 )
	// fase: '+' (P0=1,P1=0), '-' (P0=0,P1=1)
	// duracion: ms.de duracion del pulso.

	IO_set_SLP();
	IO_set_RES();
	vTaskDelay( ( TickType_t)(1) );

	switch(channel) {
	case 'A':	// Salidas AOUT1,AOUT2
		if ( phase == '+') {
			IO_set_PHA();
		} else if ( phase == '-') {
			IO_clr_PHA();
		} else {
			break;
		}
		IO_set_ENA();		// A1ENABL = 1.
		vTaskDelay( ( TickType_t)(pulse_width / portTICK_RATE_MS ) );
		IO_clr_ENA();		// A1ENABL = 0. disable bridge
		break;

	case 'B': // Salidas BOUT1, BOUT2
		if ( phase == '+') {
			IO_set_PHB();
		} else if ( phase == '-') {
			IO_clr_PHB();
		} else {
			break;
		}
		IO_set_ENB();		// B1ENABL = 1.
		vTaskDelay( ( TickType_t)(pulse_width / portTICK_RATE_MS ) );
		IO_clr_ENB();		// B1ENABL = 0. disable bridge
		break;
	}

	// Dejo el sistema en reposo
	IO_clr_PHA();
	IO_clr_PHB();
	IO_clr_ENA();
	IO_clr_ENB();
	IO_clr_SLP();
	IO_clr_RES();
}
//------------------------------------------------------------------------------------
void DRV8814_open_valve( const char valveId )
{

	// Prendo la fuente de 12V
	IO_set_SENS_12V_CTL();
	vTaskDelay( ( TickType_t)(500 / portTICK_RATE_MS ) );

	switch(valveId) {
	case 'A':
		DRV8814_pulse('A', '+', 100);
		break;
	case 'B':
		DRV8814_pulse('B', '+', 100);
		break;
	}

	vTaskDelay( ( TickType_t)(200 / portTICK_RATE_MS ) );
	// Apago la fuente de 12V
	IO_clr_SENS_12V_CTL();

}
//------------------------------------------------------------------------------------
void DRV8814_close_valve( const char valveId )
{
	// Prendo la fuente de 12V
	IO_set_SENS_12V_CTL();
	vTaskDelay( ( TickType_t)(500 / portTICK_RATE_MS ) );

	switch(valveId) {
	case 'A':
		DRV8814_pulse('A', '-', 100);
		break;
	case 'B':
		DRV8814_pulse('B', '-', 100);
		break;
	}

	vTaskDelay( ( TickType_t)(200 / portTICK_RATE_MS ) );
	// Apago la fuente de 12V
	IO_clr_SENS_12V_CTL();
}
//------------------------------------------------------------------------------------
