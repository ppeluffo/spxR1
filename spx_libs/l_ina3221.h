/*
 * l_ina3221.h
 *
 *  Created on: 13 de oct. de 2017
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_INA3221_H_
#define SRC_SPX_LIBS_L_INA3221_H_

#include "frtos-io.h"
#include "stdio.h"

//------------------------------------------------------------------------------------

#define CONF_INAS_SLEEP		0x7920
#define CONF_INAS_AVG128	0x7927

#define INA3221_ADDR_0			0x80
#define INA3221_ADDR_1			0x82

#define INA3231_CONF			0x00
#define INA3221_CH1_SHV			0x01
#define INA3221_CH1_BUSV		0x02
#define INA3221_CH2_SHV			0x03
#define INA3221_CH2_BUSV		0x04
#define INA3221_CH3_SHV			0x05
#define INA3221_CH3_BUSV		0x06
#define INA3221_MFID			0xFE
#define INA3221_DIEID			0xFF

bool INA3221_read( uint8_t devAddress, uint8_t regAddress, char *data, uint8_t length );
bool INA3221_write( uint8_t devAddress, uint8_t regAddress, char *data, uint8_t length );
uint8_t INA3221_id2busaddr( uint8_t id );

#endif /* SRC_SPX_LIBS_L_INA3221_H_ */
