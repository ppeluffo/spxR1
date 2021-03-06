/*
 * spx.h
 *
 *  Created on: 20 de oct. de 2016
 *      Author: pablo
 */

#ifndef SRC_SPXR1_H_
#define SRC_SPXR1_H_

//------------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------------
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <avr/sleep.h>
#include <ctype.h>
#include "avr_compiler.h"
#include "clksys_driver.h"
#include <inttypes.h>

#include "TC_driver.h"
#include "pmic_driver.h"
#include "wdt_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "frtos-io.h"

#include "l_iopines.h"
#include "l_eeprom.h"
#include "l_nvm.h"
#include "l_ina3221.h"
#include "l_rtc79410.h"
#include "l_file.h"
#include "l_printf.h"
#include "l_ain.h"
#include "l_outputs.h"

//------------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------------
#define SPX_FW_REV "1.0.13"
#define SPX_FW_DATE "@ 20181013"

#define SPX_HW_MODELO "spxR1 HW:xmega256A3B R1.0"
#define SPX_FTROS_VERSION "FW:FRTOS10 TICKLESS"

// Compatibilidad del protocolo
#define CONFIG_SPX_SPYMOVIL
//#define CONFIG_SPX_TAHONA
//#define CONFIG_SP5K_SPYMOVIL
//#define CONFIG_SP5K_OSE
//#define CONFIG_UTE
//----------------------------------------------------------------------------------
#ifdef CONFIG_SPX_SPYMOVIL
	// Protocolo SPX,App SPYMOVIL
	#define PROTO_SPX
	#define APP_SPX_SPYMOVIL
#endif

#ifdef CONFIG_SPX_TAHONA
	// Protocolo SPX, App LA_TAHONA
	#define PROTO_SPX
	#define APP_SPX_LATAHONA
#endif

#ifdef CONFIG_SP5K_SPYMOVIL
	// Protocolo SP5K,App SPYMOVIL
	#define PROTO_SP5K
	#define APP_SP5K_SPYMOVIL
#endif

#ifdef CONFIG_SP5K_OSE
	// Protocolo SP5K,App OSE
	#define PROTO_SP5K
	#define APP_SP5K_OSE
#endif

#ifdef CONFIG_UTE
	// Protocolo SP5K_UTE,App UTE
	#define PROTO_SPX_UTE
	#define APP_SPX_UTE
#endif
// El datalogger tiene 6 canales fisicos pero 5 disponibles
// ya que uno esta para monitorear la bateria.
//
#define NRO_ANALOG_CHANNELS		5
#define NRO_DIGITAL_CHANNELS	4

#define F_CPU (32000000UL)

//#define SYSMAINCLK 2
//#define SYSMAINCLK 8
#define SYSMAINCLK 32

#define CHAR32	32
#define CHAR64	64
#define CHAR128	128
#define CHAR256	256

#define tkCtl_STACK_SIZE		512
#define tkCmd_STACK_SIZE		512
#define tkData_STACK_SIZE		512
#define tkDigital_STACK_SIZE	512
#define tkGprs_rx_STACK_SIZE	1024
#define tkGprs_tx_STACK_SIZE	1024
#define tkOutputs_STACK_SIZE	512
#define tkXbee_STACK_SIZE		512

#define tkCtl_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkData_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkDigital_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkGprs_rx_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define tkGprs_tx_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define tkOutputs_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define tkXbee_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

#define DLGID_LENGTH		10
#define PARAMNAME_LENGTH	5
#define IP_LENGTH			24
#define APN_LENGTH			32
#define PORT_LENGTH			7
#define SCRIPT_LENGTH		64
#define PASSWD_LENGTH		15
#define PARAMNAME_LENGTH	5

TaskHandle_t xHandle_idle, xHandle_tkCtl,xHandle_tkCmd, xHandle_tkData, xHandle_tkDigital, xHandle_tkGprsRx, xHandle_tkGprsTx, xHandle_tkOutputs, xHandle_tkXbee;

char stdout_buff[CHAR64];

#define MODO_DISCRETO ( (systemVars.timerDial > 0 ) ? true : false )

// Mensajes entre tareas
#define TK_FRAME_READY			0x01	//
#define TK_REDIAL				0x04	//

//------------------------------------------------------------------------------------
typedef enum { DEBUG_NONE = 0, DEBUG_GPRS, DEBUG_RANGEMETER, DEBUG_DIGITAL, DEBUG_XBEE } t_debug;
typedef enum { OUT_OFF = 0, OUT_CONSIGNA, OUT_NORMAL } t_outputs;
typedef enum { CONSIGNA_DIURNA = 0, CONSIGNA_NOCTURNA } t_consigna_aplicada;
typedef enum { modoPWRSAVE_OFF = 0, modoPWRSAVE_ON } t_pwrSave;
typedef enum { XBEE_OFF = 0, XBEE_MASTER, XBEE_SLAVE } t_modoXbee;
typedef enum { USER_NORMAL, USER_TECNICO } usuario_t;
typedef enum { MODO_SP5K, MODO_SPX } modo_t;
typedef enum { modoRANGEMETER_OFF = 0, modoRANGEMETER_ON } t_rangeMeter;
//------------------------------------------------------------------------------------

// Estructura para manejar la hora de aplicar las consignas
typedef struct {
	uint8_t hour;
	uint8_t min;
} time_t;

typedef struct {
	uint8_t modo;
	time_t hora_start;
	time_t hora_fin;
} pwrsave_t;

// Estructura para manejar las OUTPUTS
typedef struct {
	uint8_t modo;
	uint8_t out_A;
	uint8_t out_B;
	time_t consigna_diurna;
	time_t consigna_nocturna;
	uint8_t consigna_aplicada;
} outputs_t;

// Estructura para manejar los canales ANALOGICOS
typedef struct {
	float mag_val[NRO_ANALOG_CHANNELS];
} st_analog_frame;

// Estructura para manejar los canales DIGITALES.
typedef struct {
	uint8_t level[NRO_DIGITAL_CHANNELS];
	uint16_t counter[NRO_DIGITAL_CHANNELS];
	float magnitud[NRO_DIGITAL_CHANNELS];
} st_digital_frame;

typedef struct {
	float analog_val[NRO_ANALOG_CHANNELS];
	float digital_val[NRO_DIGITAL_CHANNELS];
} st_remote_values;

// Estructura de datos manejados por la tarea DATA = ANALOGICO + DIGITAL + RANGE_METER.
typedef struct {
	RtcTimeType_t rtc;
	st_analog_frame analog_frame;
	float battery;
	st_digital_frame digital_frame;
	int16_t range;
} st_data_frame;

typedef struct {
	// Variables de trabajo.

	modo_t modo;

	char dlgId[DLGID_LENGTH];
	char apn[APN_LENGTH];
	char server_tcp_port[PORT_LENGTH];
	char server_ip_address[IP_LENGTH];
	char dlg_ip_address[IP_LENGTH];
	char serverScript[SCRIPT_LENGTH];
	char passwd[PASSWD_LENGTH];

	// Configuracion de Canales analogicos
	uint16_t coef_calibracion[NRO_ANALOG_CHANNELS];
	uint8_t imin[NRO_ANALOG_CHANNELS];	// Coeficientes de conversion de I->magnitud (presion)
	uint8_t imax[NRO_ANALOG_CHANNELS];
	float mmin[NRO_ANALOG_CHANNELS];
	float mmax[NRO_ANALOG_CHANNELS];
	char an_ch_name[NRO_ANALOG_CHANNELS][PARAMNAME_LENGTH];
	char a_ch_modo[NRO_ANALOG_CHANNELS];

	// Configuracion de canales digitales
	// Niveles logicos
	char d_ch_name[NRO_DIGITAL_CHANNELS][PARAMNAME_LENGTH];
	float d_ch_magpp[NRO_DIGITAL_CHANNELS];
	char d_ch_type[NRO_DIGITAL_CHANNELS];
	char d_ch_modo[NRO_DIGITAL_CHANNELS];

	uint8_t pwr_settle_time;

	uint16_t timerPoll;
	uint32_t timerDial;

	uint8_t csq;
	uint8_t dbm;
	t_debug debug;

	outputs_t outputs;

	pwrsave_t pwrSave;

	t_rangeMeter rangeMeter_enabled;
	uint16_t rangeMeter_factor;

	t_modoXbee xbee;

	// El checksum DEBE ser el ultimo byte del systemVars !!!!
	uint8_t checksum;

} systemVarsType;

systemVarsType systemVars;

bool startTask;
//------------------------------------------------------------------------------------
// PROTOTIPOS
//------------------------------------------------------------------------------------
void tkCtl(void * pvParameters);
void tkCmd(void * pvParameters);
void tkData(void * pvParameters);
void tkDigital(void * pvParameters);
void tkGprsRx(void * pvParameters);
void tkGprsTx(void * pvParameters);
void tkOutputs(void * pvParameters);
void tkXbee(void * pvParameters);

xSemaphoreHandle sem_SYSVars;
StaticSemaphore_t SYSVARS_xMutexBuffer;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

// Utils
void u_configure_systemMainClock(void);
void u_configure_RTC32(void);
void initMCU(void);
void pub_load_defaults (modo_t modo);
uint8_t pub_save_params_in_NVMEE(void);
bool u_load_params_from_NVMEE(void);
void u_convert_str_to_time_t ( char *time_str, time_t *time_struct );
void u_convert_int_to_time_t ( int int16time, time_t *time_struct );
void pub_configPwrSave(uint8_t modoPwrSave, char *s_startTime, char *s_endTime);
void pub_convert_str_to_time_t ( char *time_str, time_t *time_struct );
void pub_control_string( char *s_name );

// analog
void pub_analog_config_INAS( uint16_t conf_reg_value );
void pub_analog_load_defaults(void);
void pub_analog_config_channel( uint8_t channel,char *_s_aname,char *s_imin,char *s_imax,char *s_mmin,char *s_mmax );
void pub_analog_config_timerpoll ( char *s_timerpoll );
void pub_analog_config_sensortime ( char *s_sensortime );
void pub_analog_config_spanfactor ( uint8_t channel, char *s_spanfactor );
void pub_analog_read_channel ( uint8_t channel, uint16_t *raw_val, float *mag_val );
void pub_analog_read_battery ( float *mag_val );
void pub_analog_read_frame(st_analog_frame *analog_frame );
void pub_analog_prender_12vsensor ( void );
void pub_analog_apagar_12vsensor ( void );

// tkData
void pub_data_print_frame(bool wdg_control );
void pub_data_read_frame(bool wdg_control);

// digital
void pub_digital_read_frame( st_digital_frame * dframe, bool reset_counters );
void pub_digital_load_defaults(modo_t modo);
//bool pub_digital_config_channel( uint8_t channel,char *s_type, char *s_dname, char *s_magPP );
bool pub_digital_config_channel( uint8_t channel,char *s_param0, char *s_param1, char *s_param2 );

// tkCtl
bool pub_ctl_terminal_is_on(void);
void pub_ctl_watchdog_kick(uint8_t taskWdg, uint16_t timeout_in_secs );
void pub_ctl_print_wdg_timers(void);
void pub_ctl_print_stack_watermarks(void);
uint16_t pub_ctl_readTimeToNextPoll(void);
void pub_ctl_reload_timerPoll(void);

// tkGprs
int32_t pub_gprs_readTimeToNextDial(void);
void pub_gprs_redial(void);
void pub_gprs_config_timerdial ( char *s_timerdial );
void pub_gprs_load_defaults(modo_t modo);
bool pub_modem_prendido(void);

// tkOutputs
void pub_output_load_defaults(void);
void pub_output_config( t_outputs modo, uint16_t hhmm1, uint16_t hhmm2 );
void pub_output_set_consigna_diurna(void);
void pub_output_set_consigna_nocturna(void);
void pub_output_set_outputs( char id_output, uint8_t value);

// rangeMeter
void pub_rangeMeter_init(void);
void pub_rangeMeter_ping(int16_t *range);
void pub_rangeMeter_load_defaults(void);
bool pub_rangeMeter_config( uint8_t modo, uint16_t factor );

// xBee
st_remote_values *pub_xbee_get_remote_values_ptr(void);

// WATCHDOG
#define WDG_CMD			0
#define WDG_CTL			1
#define WDG_DIN			2
#define WDG_DAT			3
#define WDG_OUT			4
#define WDG_GPRSRX		5
#define WDG_GPRSTX		6
#define WDG_XBEE		7

#define NRO_WDGS		8

#endif /* SRC_SPXR1_H_ */
