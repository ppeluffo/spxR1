/*
 * l_outputs.c
 *
 *  Created on: 14 jul. 2018
 *      Author: pablo
 */
#include "l_outputs.h"

//------------------------------------------------------------------------------------
void OUT_config(void)
{
	// Configura los pines del micro que son interface del driver DRV8814.

	IO_config_ENA();
	IO_config_PHA();
	IO_config_ENB();
	IO_config_PHB();
	IO_config_V12_OUTS_CTL();
	IO_config_RES();
	IO_config_SLP();

}
//------------------------------------------------------------------------------------
void OUT_power_on(void)
{
	// Prende la fuente de 12V que alimenta el DRV8814

	IO_set_V12_OUTS_CTL();
}
//------------------------------------------------------------------------------------
void OUT_power_off(void)
{
	IO_clr_V12_OUTS_CTL();
}
//------------------------------------------------------------------------------------
void OUT_sleep_pin( uint8_t modo )
{
	switch(modo) {
	case 0:
		IO_clr_SLP();
		break;
	case 1:
		IO_set_SLP();
		break;
	default:
		break;
	}
}
//------------------------------------------------------------------------------------
void OUT_reset_pin( uint8_t modo )
{
	switch(modo) {
	case 0:
		IO_clr_RES();
		break;
	case 1:
		IO_set_RES();
		break;
	default:
		break;
	}
}
//------------------------------------------------------------------------------------
void OUT_enable_pin( char driver_id, uint8_t modo )
{

	switch (driver_id) {

	case 'A':
		switch(modo) {
		case 0:
			IO_clr_ENA();
			break;
		case 1:
			IO_set_ENA();
			break;
		default:
			break;
		}
		break;

	case 'B':
		switch(modo) {
		case 0:
			IO_clr_ENB();
			break;
		case 1:
			IO_set_ENB();
			break;
		default:
			break;
		}
		break;

	default:
		break;
	}
}
//------------------------------------------------------------------------------------
void OUT_phase_pin( char driver_id, uint8_t modo )
{

	switch (driver_id) {

	case 'A':
		switch(modo) {
		case 0:
			IO_clr_PHA();
			break;
		case 1:
			IO_set_PHA();
			break;
		default:
			break;
		}
		break;

	case 'B':
		switch(modo) {
		case 0:
			IO_clr_PHB();
			break;
		case 1:
			IO_set_PHB();
			break;
		default:
			break;
		}
		break;

	default:
		break;
	}
}
//------------------------------------------------------------------------------------
// Driver outputs
// enable,disable,sleep,awake,set{01,10}
// Driver A: Salidas AOUT1,AOUT2
// Driver B: Salidas BOUT1,BOUT2
// SET_01: xOUT1 = 0, xOUT2 = 1
// SET_10: xOUT1 = 1, xOUT2 = 0

void OUT_driver( char driver_id, uint8_t cmd )
{

	switch (cmd) {
		case OUT_ENABLE:
			OUT_reset_pin(1);
			OUT_enable_pin(driver_id, 1);
			break;
		case OUT_DISABLE:
			OUT_reset_pin(0);
			OUT_enable_pin(driver_id, 0);
			break;
		case OUT_SLEEP:
			OUT_sleep_pin(0);
			break;
		case OUT_AWAKE:
			OUT_reset_pin(1);
			OUT_sleep_pin(1);
			break;
		case OUT_SET_01:
			OUT_reset_pin(1);
			OUT_sleep_pin(1);
			OUT_phase_pin(driver_id, 0);	// SET_01: xOUT1 = 0, xOUT2 = 1
			OUT_enable_pin(driver_id, 1);
			break;
		case OUT_SET_10:
			OUT_reset_pin(1);
			OUT_sleep_pin(1);
			OUT_phase_pin(driver_id, 1);	// SET_10: xOUT1 = 1, xOUT2 = 0
			OUT_enable_pin(driver_id, 1);
			break;
		default:
			break;
	}
}
//------------------------------------------------------------------------------------
// Valvulas
// open,close, pulse
void OUT_valve( char driver_id, uint8_t cmd, uint8_t duracion )
{

	switch (cmd) {
		case OPEN:
			OUT_driver(driver_id, OUT_SET_10 );
			break;
		case CLOSE:
			OUT_driver(driver_id, OUT_SET_01 );
			break;
		case PULSE:
			OUT_driver(driver_id, OUT_SET_10 );
			vTaskDelay( ( TickType_t)( duracion / portTICK_PERIOD_MS ) );
			OUT_driver(driver_id, OUT_SET_01 );
			break;

		default:
			break;
	}

}
//------------------------------------------------------------------------------------



