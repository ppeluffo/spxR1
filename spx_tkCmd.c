/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */
#include "spx_tkGprs.h"
#include "FRTOS-CMD.h"
#include <spx.h>


static char cmd_printfBuff[256];

//static char test_buffer[FF_RECD_SIZE];
//static char cmp_buffer[FF_RECD_SIZE];

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdHelpFunction(void);
static void cmdClearScreen(void);
static void cmdResetFunction(void);
static void cmdWriteFunction(void);
static void cmdReadFunction(void);
static void cmdStatusFunction(void);
static void cmdConfigFunction(void);
static void cmdKillFunction(void);
//static void cmdTestEEpromFunction(void);

static void pv_cmd_ina3221(uint8_t cmd_mode );
static void pv_cmd_sens12V(void);
static void pv_cmd_rwEE(uint8_t cmd_mode );
static void pv_cmd_rwNVMEE(uint8_t cmd_mode );
static void pv_cmd_rwRTC(uint8_t cmd_mode );
static void pv_cmd_rwRTC_SRAM(uint8_t cmd_mode );
static void pv_cmd_rdBATTERY(void);
static void pv_cmd_rdDIN(void);
static void pv_cmd_rdMEMORY(void);
static void pv_cmd_wrOUT8814(void);
static void pv_cmd_rwGPRS(uint8_t cmd_mode );
static void pv_cmd_pulse(void);
static void pv_cmd_bt(void);
static void pv_cmd_range(void);
static void pv_cmd_rwXBEE(uint8_t cmd_mode );
static void pv_config_modo( char *tipo_canal, char *nro_canal, char *modo );

#define WR_CMD 0
#define RD_CMD 1

#define WDG_CMD_TIMEOUT	60

//------------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

uint8_t c;
uint8_t ticks;

( void ) pvParameters;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	FRTOS_CMD_init();

	// Registro los comandos y los mapeo al cmd string.
	FRTOS_CMD_register( "cls\0", cmdClearScreen );
	FRTOS_CMD_register( "reset\0", cmdResetFunction);
	FRTOS_CMD_register( "write\0", cmdWriteFunction);
	FRTOS_CMD_register( "read\0", cmdReadFunction);
	FRTOS_CMD_register( "help\0", cmdHelpFunction );
	FRTOS_CMD_register( "status\0", cmdStatusFunction );
	FRTOS_CMD_register( "config\0", cmdConfigFunction );
	FRTOS_CMD_register( "kill\0", cmdKillFunction );
//	FRTOS_CMD_register( "test\0", cmdTestEEpromFunction );

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("starting tkCmd..\r\n\0"));
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Fijo el timeout del READ
	ticks = 5;
	frtos_ioctl( fdUSB,ioctl_SET_TIMEOUT, &ticks );
//	frtos_ioctl( fdBT,ioctl_SET_TIMEOUT, &ticks );

	//FRTOS_CMD_regtest();
	// loop
	for( ;; )
	{

		pub_watchdog_kick(WDG_CMD, WDG_CMD_TIMEOUT);

		// Si no tengo terminal conectada, duermo 5s lo que me permite entrar en tickless.
		if ( ! pub_terminal_is_on() ) {
			vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );

		} else {

			c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
			// el read se bloquea 50ms. lo que genera la espera.
			while ( CMD_read( &c, 1 ) == 1 ) {
				FRTOS_CMD_process(c);
			}
		}
	}
}
//------------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{

char aux_str[32];
uint8_t channel;
FAT_t l_fat;
uint8_t pos;

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SIGNATURE ID
	memset(&aux_str,'\0', sizeof(aux_str));
	NVM_readID(aux_str);
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("signature:1 %s\r\n\0"), aux_str);
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Fecha y Hora
	pv_cmd_rwRTC( RD_CMD );

	// DlgId
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("dlgid: %s\r\n\0"), systemVars.dlgId );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Memoria
	FAT_read(&l_fat);
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("memory: wrPtr=%d,rdPtr=%d,delPtr=%d,r4wr=%d,r4rd=%d,r4del=%d \r\n\0"), l_fat.wrPTR,l_fat.rdPTR, l_fat.delPTR,l_fat.rcds4wr,l_fat.rcds4rd,l_fat.rcds4del );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SERVER
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Server:\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// APN
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn: %s\r\n\0"), systemVars.apn );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SERVER IP:SERVER PORT
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server ip:port: %s:%s\r\n\0"), systemVars.server_ip_address,systemVars.server_tcp_port );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SERVER SCRIPT
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server script: %s\r\n\0"), systemVars.serverScript );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SERVER PASSWD
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  passwd: %s\r\n\0"), systemVars.passwd );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// MODEM
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Modem:\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// DLG IP ADDRESS
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ip address: %s\r\n\0"), systemVars.dlg_ip_address);
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CSQ */
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  signalQ: csq=%d, dBm=%d\r\n\0"), systemVars.csq, systemVars.dbm );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// GPRS STATE
	pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  state: "));
	switch (GPRS_stateVars.state) {
	case G_ESPERA_APAGADO:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("await_off\r\n"));
		break;
	case G_PRENDER:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("prendiendo\r\n"));
		break;
	case G_CONFIGURAR:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("configurando\r\n"));
		break;
	case G_MON_SQE:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("mon_sqe\r\n"));
		break;
	case G_GET_IP:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("ip\r\n"));
		break;
	case G_INIT_FRAME:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("init frame\r\n"));
		break;
	case G_DATA:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("data\r\n"));
		break;
	default:
		pos += FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), PSTR("ERROR\r\n"));
		break;
	}
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// CONFIG
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Config:\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	switch(systemVars.xbee) {
	case XBEE_OFF:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  xbee: off\r\n\0") );
		break;
	case XBEE_MASTER:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  xbee: master\r\n\0") );
		break;
	case XBEE_SLAVE:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  xbee: slave\r\n\0") );
		break;
	}
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	switch(systemVars.debug) {
	case DEBUG_NONE:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debug: none\r\n\0") );
		break;
	case DEBUG_GPRS:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debug: gprs\r\n\0") );
		break;
	case DEBUG_RANGEMETER:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debug: range\r\n\0") );
		break;
	case DEBUG_DIGITAL:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debug: digital\r\n\0") );
		break;
	}
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerDial: [%lu s]/%li\r\n\0"),systemVars.timerDial, pub_gprs_readTimeToNextDial() );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerPoll: [%d s]\r\n\0"),systemVars.timerPoll );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// PULSE WIDTH
	if ( systemVars.rangeMeter_enabled ) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  RangeMeter: ON\r\n"));
	} else {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  RangeMeter: OFF\r\n"));
	}
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// PWR SAVE:
	if ( systemVars.pwrSave.modo ==  modoPWRSAVE_OFF ) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  pwrsave=off\r\n\0"));
	} else {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff), PSTR("  pwrsave=on start[%02d:%02d], end[%02d:%02d]\r\n\0"), systemVars.pwrSave.hora_start.hour, systemVars.pwrSave.hora_start.min, systemVars.pwrSave.hora_fin.hour, systemVars.pwrSave.hora_fin.min);
	}
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// OUTPUTS:
	switch( systemVars.outputs.modo ) {
	case OUT_OFF:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: OFF\r\n"));
		break;
	case OUT_CONSIGNA:
		switch( systemVars.outputs.consigna_aplicada ) {
		case CONSIGNA_DIURNA:
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: CONSIGNA [diurna] (c_dia=%02d:%02d, c_noche=%02d:%02d)\r\n"), systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
			break;
		case CONSIGNA_NOCTURNA:
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: CONSIGNA [nocturna] (c_dia=%02d:%02d, c_noche=%02d:%02d)\r\n"), systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
			break;
		default:
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: CONSIGNA [error] (c_dia=%02d:%02d, c_noche=%02d:%02d)\r\n"), systemVars.outputs.consigna_diurna.hour, systemVars.outputs.consigna_diurna.min, systemVars.outputs.consigna_nocturna.hour, systemVars.outputs.consigna_nocturna.min );
			break;
		}
		break;
	case OUT_NORMAL:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: NORMAL (out_A=%d, out_B=%d)\r\n"), systemVars.outputs.out_A, systemVars.outputs.out_B );
		break;
	default:
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  Outputs: ERROR(%d) (out_A=%d, out_B=%d)\r\n"), systemVars.outputs.modo, systemVars.outputs.out_A, systemVars.outputs.out_B );
		break;
	}
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Configuracion de canales analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		if ( systemVars.a_ch_modo[channel] == 'R') {
			pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  a%d(*)\0"),channel );
		} else {
			pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  a%d( )\0"),channel );
		}
		FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" [%d-%d mA/ %.02f,%.02f | %04d | %s]\r\n\0"), systemVars.imin[channel],systemVars.imax[channel],systemVars.mmin[channel],systemVars.mmax[channel], systemVars.coef_calibracion[channel], systemVars.an_ch_name[channel] );
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	// Configuracion de canales digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		if ( systemVars.d_ch_modo[channel] == 'R') {
			pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  d%d(*)\0"),channel );
		} else {
			pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  d%d( )\0"),channel );
		}

		if ( systemVars.d_ch_type[channel] == 'C') {
			// Los canales de contadores de pulsos 'C' muestran el factor de conversion
			FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" [ C | %s | %.02f ]\r\n\0"), systemVars.d_ch_name[channel],systemVars.d_ch_magpp[channel] );
		} else {
			// Los canales de nivel solo muestran el nombre.
			FRTOS_snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" [ L | %s ]\r\n\0"), systemVars.d_ch_name[channel] );
		}
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	// Valores actuales:
	pub_tkData_print_frame( pub_tkData_get_data_frame_ptr() );

}
//-----------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
	// Resetea al micro prendiendo el watchdog

	FRTOS_CMD_makeArgv();

	// ALARM
	// reset alarm
	if (!strcmp_P( strupr(argv[1]), PSTR("ALARM\0"))) {
		RTC79410_alarm0_reset();
		return;
	}

	// Reset memory ??
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))) {

		// Nadie debe usar la memoria !!!
		vTaskSuspend( xHandle_tkData );

		if (!strcmp_P( strupr(argv[2]), PSTR("SOFT\0"))) {
			FF_format(false, cmd_printfBuff );
		}
		if (!strcmp_P( strupr(argv[2]), PSTR("HARD\0"))) {
			FF_format(true, cmd_printfBuff);
		}
	}

	cmdClearScreen();

	CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */


}
//------------------------------------------------------------------------------------
static void cmdWriteFunction(void)
{

	FRTOS_CMD_makeArgv();

	// ALARM
	// write alarm secs
	if (!strcmp_P( strupr(argv[1]), PSTR("ALARM\0")) ) {
		RTC79410_alarm0_set(argv[2]);
		return;
	}

	// RTC
	// write rtc YYMMDDhhmm
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0")) ) {
		pv_cmd_rwRTC(WR_CMD);
		return;
	}

	// INA
	// write ina confReg Value
	// Solo escribimos el registro 0 de configuracion.
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0")) ) {
		pv_cmd_ina3221(WR_CMD);
		return;
	}

	// SENS12V
	if (!strcmp_P( strupr(argv[1]), PSTR("SENS12V\0")) ) {
		pv_cmd_sens12V();
		return;
	}

	// NVMEE
	// write nvmee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE\0"))) {
		pv_cmd_rwNVMEE(WR_CMD);
		return;
	}

	// EE
	// write ee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		pv_cmd_rwEE(WR_CMD);
		return;
	}

	// RTC SRAM
	// write rtcram pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("RTCRAM\0")) ) {
		pv_cmd_rwRTC_SRAM(WR_CMD);
		return;
	}

	// CLRD
	// write clrd {0|1}
	if (!strcmp_P( strupr(argv[1]), PSTR("CLRD\0")) ) {
		if ( atoi( argv[2]) == 0 ) { IO_clr_CLRD(); }
		if ( atoi( argv[2]) == 1 ) { IO_set_CLRD(); }
		return;
	}

	// OUT 8814
	// write out sleep|reset|phase(A/B)|enable(A/B)| {0|1}
	//       out pulse (A/B) (+/-) (ms)
	//       out power {on|off}
	if (!strcmp_P( strupr(argv[1]), PSTR("OUT\0")) ) {
		pv_cmd_wrOUT8814();
		return;
	}

	// GPRS
	// write gprs pwr|sw|rts {on|off}
	// write gprs cmd {atcmd}
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0")) ) {
		pv_cmd_rwGPRS(WR_CMD);
		return;
	}

	// XBEE
	// write xbee pwr {on|off}
	// write xbee msg
	if (!strcmp_P( strupr(argv[1]), PSTR("XBEE\0")) ) {
		pv_cmd_rwXBEE(WR_CMD);
		return;
	}

	// BT
	if (!strcmp_P( strupr(argv[1]), PSTR("BT\0")) ) {
		pv_cmd_bt();
		return;
	}

	// PULSE
	if (!strcmp_P( strupr(argv[1]), PSTR("PULSE\0")) ) {
		pv_cmd_pulse();
		return;
	}

	// CONSIGNA
	// write consigna {diurna|nocturna}
	if (!strcmp_P( strupr(argv[1]), PSTR("CONSIGNA\0")) ) {

		if (!strcmp_P( strupr(argv[2]), PSTR("DIURNA\0")) ) {
			pub_output_set_consigna_diurna();
			pv_snprintfP_OK();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("NOCTURNA\0")) ) {
			pub_output_set_consigna_nocturna();
			pv_snprintfP_OK();
			return;
		}

		pv_snprintfP_ERR();
		return;
	}

	// OUTPUTS
	// write outputs {x,x}
	// Debo esperar para que se carguen los condensadores
	if (!strcmp_P( strupr(argv[1]), PSTR("OUTPUTS\0")) ) {
		pub_output_set_outputs( 'A', atoi(argv[2]) );
		vTaskDelay( ( TickType_t)( 3000 / portTICK_RATE_MS ) );
		pub_output_set_outputs( 'B', atoi(argv[3]) );
		pv_snprintfP_OK();
		return;
	}

	// CMD NOT FOUND
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
//------------------------------------------------------------------------------------
static void cmdReadFunction(void)
{

uint16_t raw_val;
float mag_val;
st_data_frame data_frame;

	FRTOS_CMD_makeArgv();

	// WMK
 	if (!strcmp_P( strupr(argv[1]), PSTR("WMK\0"))) {
 		pub_print_stack_watermarks();
 		return;
 	}

 	// WDT
 	if (!strcmp_P( strupr(argv[1]), PSTR("WDT\0"))) {
 		pub_print_wdg_timers();
 		return;
 	}

	// RTC
	// read rtc
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0")) ) {
		pv_cmd_rwRTC(RD_CMD);
		return;
	}

	// FRAME
	// read frame
	if (!strcmp_P( strupr(argv[1]), PSTR("FRAME\0")) ) {
		pub_tkdata_read_frame(&data_frame);
		pub_tkData_print_frame(&data_frame);
		return;
	}

	// SIGNATURE
	// read id
	if (!strcmp_P( strupr(argv[1]), PSTR("ID\0"))) {
		NVM_readID(cmd_printfBuff);
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// INA3221
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0"))) {
		pv_cmd_ina3221(RD_CMD);
		return;
	}

	// NVMEE
	// read nvmee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE\0"))) {
		pv_cmd_rwNVMEE(RD_CMD);
		return;
	}

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		pv_cmd_rwEE(RD_CMD);
		return;
	}

	// RTC SRAM
	// read rtcram address length
	if (!strcmp_P( strupr(argv[1]), PSTR("RTCRAM\0")) ) {
		pv_cmd_rwRTC_SRAM(RD_CMD);
		return;
	}

	// BATTERY
	// read battery
	if (!strcmp_P( strupr(argv[1]), PSTR("BATTERY\0")) ) {
		pv_cmd_rdBATTERY();
		return;
	}

	// AN { 0..8}
	if (!strcmp_P( strupr(argv[1]), PSTR("AN\0"))) {
		pub_analog_read_channel( atoi(argv[2]),&raw_val, &mag_val );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("CH[%02d] raw=%d,mag=%.02f\r\n\0"),atoi(argv[2]),raw_val, mag_val );
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// DIN
	// read din {0,1} {P,L}
	if (!strcmp_P( strupr(argv[1]), PSTR("DIN\0")) ) {
		pv_cmd_rdDIN();
		return;
	}

	// MEMORY
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0")) ) {
		pv_cmd_rdMEMORY();
		return;
	}

	// GPRS
	// read gprs (rsp,cts,dcd,ri)
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0")) ) {
		pv_cmd_rwGPRS(RD_CMD);
		return;
	}

	// PULSES
	// read pulses
	if (!strcmp_P( strupr(argv[1]), PSTR("PULSES\0")) ) {
		pv_cmd_range();
		return;
	}

	// CMD NOT FOUND
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;

}
//------------------------------------------------------------------------------------
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\x1B[2J\0"));
	CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
}
//------------------------------------------------------------------------------------
static void cmdConfigFunction(void)
{

bool retS = false;

	FRTOS_CMD_makeArgv();

	// DLGID
	if (!strcmp_P( strupr(argv[1]), PSTR("DLGID\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
			} else {
				memcpy(systemVars.dlgId, argv[2], sizeof(systemVars.dlgId));
				systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
				retS = true;
			}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// modo
	// config modo {analog|digital} {0..N} {L|R}
	if (!strcmp_P( strupr(argv[1]), PSTR("MODO\0"))) {
		pv_config_modo( argv[2], argv[3], argv[4] );
		return;
	}

	// xbee
	if (!strcmp_P( strupr(argv[1]), PSTR("XBEE\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0"))) {
			systemVars.xbee = XBEE_OFF;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("MASTER\0"))) {
			systemVars.xbee = XBEE_MASTER;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("SLAVE\0"))) {
			systemVars.xbee = XBEE_SLAVE;
			retS = true;
		} else {
			retS = false;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// config outputs
	if (!strcmp_P( strupr(argv[1]), PSTR("OUTPUTS\0")) ) {
		pub_outputs_config( argv[2], argv[3], argv[4] );
		pv_snprintfP_OK();
		return;
	}

	// rangemeter {on|off}
	if (!strcmp_P( strupr(argv[1]), PSTR("RANGEMETER\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON\0"))) {
			systemVars.rangeMeter_enabled = true;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0"))) {
			systemVars.rangeMeter_enabled = false;
			retS = true;
		} else {
			retS = false;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// config debug
	if (!strcmp_P( strupr(argv[1]), PSTR("DEBUG\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("NONE\0"))) {
			systemVars.debug = DEBUG_NONE;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("GPRS\0"))) {
			systemVars.debug = DEBUG_GPRS;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("RANGE\0"))) {
			systemVars.debug = DEBUG_RANGEMETER;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("DIGITAL\0"))) {
			systemVars.debug = DEBUG_DIGITAL;
			retS = true;
		} else {
			retS = false;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// config save
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE\0"))) {
		pub_save_params_in_EE();
		pv_snprintfP_OK();
		return;
	}

	// config analog {0..2} aname imin imax mmin mmax
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOG\0")) ) {
		pub_analog_config_channel( atoi(argv[2]), argv[3], argv[4], argv[5], argv[6], argv[7] );
		pv_snprintfP_OK();
		return;
	}

	// config digital {0..3} type dname magPP
	if (!strcmp_P( strupr(argv[1]), PSTR("DIGITAL\0")) ) {
		if ( pub_tkDigital_config_channel( atoi(argv[2]), argv[3], argv[4], argv[5] ) ) {
			pv_snprintfP_OK();
		} else {
			pv_snprintfP_ERR();
		}
		return;
	}

	// config timerpoll
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL\0")) ) {
		pub_analog_config_timerpoll( argv[2] );
		pv_snprintfP_OK();
		return;
	}

	// config timerdial
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERDIAL\0")) ) {
		pub_gprs_config_timerdial( argv[2] );
		pv_snprintfP_OK();
		return;
	}

	// config calibrar
	if (!strcmp_P( strupr(argv[1]), PSTR("CFACTOR\0"))) {
		pub_analog_config_spanfactor( atoi(argv[2]), argv[3] );
		pv_snprintfP_OK();
		return;
	}

	// config default
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULT\0"))) {
		pub_load_defaults();
		pv_snprintfP_OK();
		return;
	}

	// apn
	if (!strcmp_P( strupr(argv[1]), PSTR("APN\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.apn, '\0', sizeof(systemVars.apn));
			memcpy(systemVars.apn, argv[2], sizeof(systemVars.apn));
			systemVars.apn[APN_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// port ( SERVER IP PORT)
	if (!strcmp_P( strupr(argv[1]), PSTR("PORT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.server_tcp_port, '\0', sizeof(systemVars.server_tcp_port));
			memcpy(systemVars.server_tcp_port, argv[2], sizeof(systemVars.server_tcp_port));
			systemVars.server_tcp_port[PORT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// ip (SERVER IP ADDRESS)
	if (!strcmp_P( strupr(argv[1]), PSTR("IP\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.server_ip_address, '\0', sizeof(systemVars.server_ip_address));
			memcpy(systemVars.server_ip_address, argv[2], sizeof(systemVars.server_ip_address));
			systemVars.server_ip_address[IP_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// script ( SERVER SCRIPT SERVICE )
	if (!strcmp_P( strupr(argv[1]), PSTR("SCRIPT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.serverScript, '\0', sizeof(systemVars.serverScript));
			memcpy(systemVars.serverScript, argv[2], sizeof(systemVars.serverScript));
			systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// passwd
	if (!strcmp_P( strupr(argv[1]), PSTR("PASSWD\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.passwd, '\0', sizeof(systemVars.passwd));
			memcpy(systemVars.passwd, argv[2], sizeof(systemVars.passwd));
			systemVars.passwd[PASSWD_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PWRSAVE
	if (!strcmp_P( strupr(argv[1]), PSTR("PWRSAVE\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR( "ON"))) { pub_configPwrSave ( modoPWRSAVE_ON, argv[3], argv[4] ); }
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) { pub_configPwrSave ( modoPWRSAVE_OFF, argv[3], argv[4] ); }
		pv_snprintfP_OK();
		return;
	}

	pv_snprintfP_ERR();
	return;

}
//------------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

	FRTOS_CMD_makeArgv();

	// HELP WRITE
	if (!strcmp_P( strupr(argv[1]), PSTR("WRITE\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-write\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc YYMMDDhhmm\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  consigna {diurna|nocturna}, outputs {x,x}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee,nvmee,rtcram {pos} {string}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ina {id} conf {value}, sens12V {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  clrd {0|1}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out sleep|reset|phase(A/B)|enable(A/B) {0|1}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out pulse (A/B) (+/-) (ms)\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  out power {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  bt {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprs (pwr|sw|cts|dtr) {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprs cmd {atcmd}, redial\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  xbee (pwr|sleep|reset) {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  xbee msg\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pulse {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  alarm (secs)\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;

	}

	// HELP READ
	else if (!strcmp_P( strupr(argv[1]), PSTR("READ\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-read\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc, frame\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee,nvmww,rtcram {pos} {lenght}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ina {id} {conf|chXshv|chXbusv|mfid|dieid}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  memory\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  an {0..4}, battery\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  din\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprs (rsp,rts,dcd,ri)\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pulse\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;

	}

	// HELP RESET
	else if (!strcmp_P( strupr(argv[1]), PSTR("RESET\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-reset\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  memory {soft|hard}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  alarm\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;

	}

	// HELP CONFIG
	else if (!strcmp_P( strupr(argv[1]), PSTR("CONFIG\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-config\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  analog {0..4} aname imin imax mmin mmax\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  cfactor {ch} {coef}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  digital {0..3} type(L,C) dname magPP\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rangemeter {on|off}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  modo {analog|digital} {0..n} {local|remoto}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  xbee {off|master|slave}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  outputs {off}|{normal o0 o1}|{consigna hhmm_dia hhmm_noche}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerpoll, timerdial, dlgid {name}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  pwrsave modo [{on|off}] [{hhmm1}, {hhmm2}]\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn, port, ip, script, passwd\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debug {none,gprs,digital,range}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  default\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  save\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;

	}

	// HELP KILL
	else if (!strcmp_P( strupr(argv[1]), PSTR("KILL\0"))) {
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-kill {data,digi,gprstx,gprsrx,outputs}\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;

	} else {

		// HELP GENERAL
		memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
		CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );
		CMD_write(cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Available commands are:\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-cls\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-help\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-status\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-reset...\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-kill...\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-write...\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-read...\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-config...\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	}

//	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-test eeprom {pages}\r\n\0"));
//	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

}
//------------------------------------------------------------------------------------
static void cmdKillFunction(void)
{

	FRTOS_CMD_makeArgv();

	// KILL DATA
	if (!strcmp_P( strupr(argv[1]), PSTR("DATA\0"))) {
		vTaskSuspend( xHandle_tkData );
		pub_watchdog_kick(WDG_DAT, 0xFFFF);
		return;
	}

	// KILL DIGITAL
	if (!strcmp_P( strupr(argv[1]), PSTR("DIGI\0"))) {
		vTaskSuspend( xHandle_tkDigital );
		pub_watchdog_kick(WDG_DIN, 0xFFFF);
		return;
	}

	// KILL GPRS
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSTX\0"))) {
		vTaskSuspend( xHandle_tkGprsTx );
		pub_watchdog_kick(WDG_GPRSTX, 0xFFFF);
		// Dejo la flag de modem prendido para poder leer comandos
		GPRS_stateVars.modem_prendido = true;
		return;
	}

	// KILL RX
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSRX\0"))) {
		vTaskSuspend( xHandle_tkGprsRx );
		pub_watchdog_kick(WDG_GPRSRX, 0xFFFF);
		return;
	}

	// KILL OUTPUTS
	if (!strcmp_P( strupr(argv[1]), PSTR("OUTPUTS\0"))) {
		vTaskSuspend( xHandle_tkOutputs );
		pub_watchdog_kick(WDG_OUT, 0xFFFF);
		return;
	}

	pv_snprintfP_OK();
	return;
}
//------------------------------------------------------------------------------------
/*
static void cmdTestEEpromFunction(void)
{
	// Prueba la escritura/lectura de paginas de la eeprom
uint8_t argc;
uint16_t page;
uint8_t i;
char wrChar = '0';
uint8_t cks;
uint16_t eeAddress;
int8_t bytes_w = -1;
int8_t bytes_r = -1;
	argc = FRTOS_CMD_makeArgv();
	// Reset memory ??
	if (!strcmp_P( strupr(argv[1]), PSTR("EEPROM\0"))) {
		page = atoi(argv[2]);
		// Escribo la pagina con un pattern dado
		for ( i = 0; i < (FF_RECD_SIZE - 2); i++) {
			test_buffer[i] = wrChar++;
		}
		// Calculo el checksum
		cks = pv_memChecksum(&test_buffer,(FF_RECD_SIZE - 2) );
		test_buffer[FF_RECD_SIZE - 2] = cks;
		test_buffer[FF_RECD_SIZE - 1] = FF_WRTAG;
		eeAddress = FF_ADDR_START + page * FF_RECD_SIZE;
		bytes_w = EE_write( eeAddress, &test_buffer, FF_RECD_SIZE );
		// Leo
		vTaskDelay( ( TickType_t)( 10 / portTICK_RATE_MS ) );
		bytes_r = EE_read( eeAddress, &cmp_buffer, FF_RECD_SIZE);
		// Imprimo.
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nPage=%03d, wrAddress=%d, bytes_wtn=%d, cks=0x%02x\r\n\0 "),page, eeAddress, bytes_w, cks);
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		for ( i = 0; i < (FF_RECD_SIZE - 2); i++) {
			if ( (i % 16) == 0 ) {
				FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
				CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
			}
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("[%c ]"),test_buffer[i]);
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		}
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("[%02x][%02x]\r\n\r\n\0"),test_buffer[(FF_RECD_SIZE - 2)],test_buffer[(FF_RECD_SIZE - 1)] );
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		for ( i = 0; i < (FF_RECD_SIZE - 2); i++) {
			if ( (i % 16) == 0 ) {
				FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
				CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
			}
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("[%c ]"),cmp_buffer[i]);
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		}
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("[%02x][%02x]\r\n\0"),cmp_buffer[(FF_RECD_SIZE - 2)],cmp_buffer[(FF_RECD_SIZE - 1)] );
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
}
*/
//------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ok\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("error\r\n\0"));
	CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
}
//------------------------------------------------------------------------------------
static void pv_cmd_ina3221(uint8_t cmd_mode )
{

uint16_t val;
uint8_t ina_id;
char data[3];

	// write ina id conf {value}
	if ( cmd_mode == WR_CMD ) {

		if (!strcmp_P( strupr(argv[3]), PSTR("CONF\0")) ) {
			ina_id = atoi(argv[2]);
			val = atoi( argv[4]);
			data[0] = ( val & 0xFF00 ) >> 8;
			data[1] = ( val & 0x00FF );
			INA3221_write( INA3221_id2busaddr(ina_id), INA3231_CONF, data, 2 );
			pv_snprintfP_OK();
			return;
		}
	}

	// read ina id {conf|chxshv|chxbusv|mfid|dieid}
	if ( cmd_mode == RD_CMD ) {

		ina_id = atoi(argv[2]);

		if (!strcmp_P( strupr(argv[3]), PSTR("CONF\0"))) {
			INA3221_read(  INA3221_id2busaddr(ina_id), INA3231_CONF, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH1SHV\0"))) {
			INA3221_read(  INA3221_id2busaddr(ina_id), INA3221_CH1_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH1BUSV\0"))) {
			INA3221_read(  INA3221_id2busaddr(ina_id), INA3221_CH1_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH2SHV\0"))) {
			INA3221_read(  INA3221_id2busaddr(ina_id), INA3221_CH2_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH2BUSV\0"))) {
			INA3221_read(  INA3221_id2busaddr(ina_id), INA3221_CH2_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH3SHV\0"))) {
			INA3221_read(  INA3221_id2busaddr(ina_id), INA3221_CH3_SHV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("CH3BUSV\0"))) {
			INA3221_read(  INA3221_id2busaddr(ina_id), INA3221_CH3_BUSV, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("MFID\0"))) {
			INA3221_read(  INA3221_id2busaddr(ina_id), INA3221_MFID, data, 2 );
		} else if (!strcmp_P( strupr(argv[3]), PSTR("DIEID\0"))) {
			INA3221_read(  INA3221_id2busaddr(ina_id), INA3221_DIEID, data, 2 );
		} else {
			pv_snprintfP_ERR();
			return;
		}

		val = ( data[0]<< 8 ) + data	[1];
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("VAL=0x%04x\r\n\0"), val);
		CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
		pv_snprintfP_OK();
		return;

	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_sens12V(void)
{
	// sens12V on|off
	if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
		IO_set_SENS_12V_CTL();
		pv_snprintfP_OK();
		return;
	}

	if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
		IO_clr_SENS_12V_CTL();
		pv_snprintfP_OK();
		return;
	}

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("cmd ERROR: ( write sens12V on{off} )\r\n\0"));
	CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_bt(void)
{
	// sens12V on|off
	if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
		IO_set_BT_PWR_CTL();
		pv_snprintfP_OK();
		return;
	}

	if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
		IO_clr_BT_PWR_CTL();
		pv_snprintfP_OK();
		return;
	}

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("cmd ERROR: ( write sens12V on{off} )\r\n\0"));
	CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_pulse(void)
{
	// pulse on|off
	if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
		IO_set_UPULSE_RUN();
		pv_snprintfP_OK();
		return;
	}

	if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
		IO_clr_UPULSE_RUN();
		pv_snprintfP_OK();
		return;
	}

	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("cmd ERROR: ( write pulse on{off} )\r\n\0"));
	CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_rwEE(uint8_t cmd_mode )
{

bool retS;

	// read ee {pos} {lenght}
	if ( cmd_mode == RD_CMD ) {
		memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		retS = EE_test_read( argv[2], cmd_printfBuff, argv[3] );
		if ( retS ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			FRTOS_snprintf_P( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff),PSTR( "\r\n\0"));
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// write ee pos string
	if ( cmd_mode == WR_CMD ) {
		retS = EE_test_write( argv[2], argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwNVMEE(uint8_t cmd_mode )
{
	// Hace prueba de lectura y escritura de la memoria internan EE del micro
	// que es la que usamos para guardar la configuracion.

	// read nvmee {pos} {lenght}
	if ( cmd_mode == RD_CMD ) {
		memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		NVM_EEPROM_test_read( argv[2], cmd_printfBuff, argv[3] );
		// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
		FRTOS_snprintf_P( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff),PSTR( "\r\n\0"));
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		pv_snprintfP_OK();
		return;
	}

	// write nvmee pos string
	if ( cmd_mode == WR_CMD ) {
		NVM_EEPROM_test_write( argv[2], argv[3]);
		pv_snprintfP_OK();
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwRTC(uint8_t cmd_mode )
{

char datetime[24];
RtcTimeType_t rtc;
bool retS;

	if ( cmd_mode == WR_CMD ) {
		RTC_str2rtc(argv[2], &rtc);				// Convierto el string YYMMDDHHMM a RTC.
		retS = RTC79410_write_dtime(&rtc);		// Grabo el RTC
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if ( cmd_mode == RD_CMD ) {
		RTC79410_read_dtime(&rtc);
		RTC_rtc2str(datetime,&rtc);
		FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("%s\r\n\0"), datetime );
		CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwRTC_SRAM(uint8_t cmd_mode )
{
	// Como se usa para leer memoria, la impresion la hacemos en hex

uint8_t rtc_sram_buffer[32];
uint8_t pos,i;

bool retS;

	// read ee {pos} {lenght}
	if ( cmd_mode == RD_CMD ) {
		memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		memset(rtc_sram_buffer, '\0', sizeof(rtc_sram_buffer));
		retS = RTC79410_test_read( argv[2], rtc_sram_buffer, argv[3] );
		if ( retS ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			pos = FRTOS_snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff),PSTR( "\r\n"));
			for (i=0; i < atoi(argv[3]); i++ ) {
				pos += FRTOS_snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),PSTR("[0x%02x]"),rtc_sram_buffer[i]);
			}
			FRTOS_snprintf_P( &cmd_printfBuff[pos], (sizeof(cmd_printfBuff) - pos),PSTR( "\r\n\0"));
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// write ee pos string
	if ( cmd_mode == WR_CMD ) {
		retS = RTC79410_test_write( argv[2], argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rdBATTERY(void)
{

float battery;

	pub_analog_read_battery(&battery);
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("BATTERY=%.02f\r\n\0"), battery );
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_rdDIN(void)
{
	// Leo todas las entradas digitales.

st_digital_frame digital_frame;
uint8_t i,pos;

	// Leo los valores
	pub_tkDigital_read_frame( &digital_frame , false );

	// Armo la respuesta
	pos = FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("DIN: "));

	for (i = 0; i < NRO_DIGITAL_CHANNELS; i++) {
		// Canales contadores
		if ( systemVars.d_ch_type[i] == 'C' ) {
			pos += FRTOS_snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),PSTR("d%d[C:%s,%.02f] "), i, systemVars.d_ch_name[i], digital_frame.magnitud[i] );
		}
		// Canales de niveles logicos
		if ( systemVars.d_ch_type[i] == 'L' ) {
			pos += FRTOS_snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),PSTR("d%d[L:%s,%d] "), i, systemVars.d_ch_name[i],digital_frame.level[i] );
		}
	}

	pos += FRTOS_snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),PSTR("\r\n\0"));
	CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

}
//------------------------------------------------------------------------------------
static void pv_cmd_rdMEMORY(void)
{
	// Leemos la memoria e imprimo los datos.
	// El problema es que si hay muchos datos puede excederse el tiempo de watchdog y
	// resetearse el dlg.
	// Para esto, cada 32 registros pateo el watchdog.

//FAT_t l_fat;
st_data_frame data_frame;
int16_t bytes_read;

	FF_rewind();
	while(1) {
		// Necesito esperar un poco entre c/ciclo
		vTaskDelay( ( TickType_t)( 10 / portTICK_RATE_MS ) );
		bytes_read = FF_readRcd( &data_frame, sizeof(st_data_frame));

		// imprimo stats
		//FAT_read(&l_fat);
		//FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("RD: wrPtr=%d,rdPtr=%d,delPtr=%d,r4wr=%d,r4rd=%d,r4del=%d \r\n\0"),l_fat.wrPTR,l_fat.rdPTR, l_fat.delPTR,l_fat.rcds4wr,l_fat.rcds4rd,l_fat.rcds4del );
		//CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );

		// Controlo errores:
		switch( FF_errno() ) {
		case pdFF_ERRNO_MEMEMPTY:
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Memory Empty.\r\n\0") );
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
			return;
			break;
		case pdFF_ERRNO_MEMRD:
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR RD size\r\n\0") );
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
			break;
		case pdFF_ERRNO_RDCKS:
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR RD checksum\r\n\0") );
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
			break;
		case pdFF_ERRNO_RDNOTAG:
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR RD no tag\r\n\0") );
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
			break;
		default:
			//FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("RD OK: %d bytes\r\n\0"),bytes_read );
			//CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
			pub_tkData_print_frame(&data_frame);
			break;
		}
	}
}
//------------------------------------------------------------------------------------
static void pv_cmd_wrOUT8814(void)
{
	// write out sleep|reset|phase(A/B)|enable(A/B)| {0|1}
	//       out pulse (A/B) (+/-) (ms)
	//       out power {on|off}

	// write out sleep 0,1
	if (!strcmp_P( strupr(argv[2]), PSTR("SLEEP\0")) ) {
		switch(atoi(argv[3])) {
		case 0:
			IO_clr_SLP();
			pv_snprintfP_OK();
			break;
		case 1:
			IO_set_SLP();
			pv_snprintfP_OK();
			break;
		default:
			pv_snprintfP_ERR();
		}
		return;
	}

	// write out reset 0,1
	if (!strcmp_P( strupr(argv[2]), PSTR("RESET\0")) ) {
		switch(atoi(argv[3])) {
		case 0:
			IO_clr_RES();
			pv_snprintfP_OK();
			break;
		case 1:
			IO_set_RES();
			pv_snprintfP_OK();
			break;
		default:
			pv_snprintfP_ERR();
		}
		return;
	}

	// write out phase (a/b) (0/1)
	if (!strcmp_P( strupr(argv[2]), PSTR("PHASE\0")) ) {
		switch (toupper(argv[3][0])) {
		case 'A':
			if (atoi(argv[4]) == 0) {
				IO_clr_PHA();
				pv_snprintfP_OK();
				return;
			}
			if (atoi(argv[4]) == 1) {
				IO_set_PHA();
				pv_snprintfP_OK();
				return;
			}
			pv_snprintfP_ERR();
			return;
		case 'B':
			if (atoi(argv[4]) == 0) {
				IO_clr_PHB();
				pv_snprintfP_OK();
				return;
			}
			if (atoi(argv[4]) == 1) {
				IO_set_PHB();
				pv_snprintfP_OK();
				return;
			}
			pv_snprintfP_ERR();
			return;
		}
	}

	// write out enable (a/b) (0/1)
	if (!strcmp_P( strupr(argv[2]), PSTR("ENABLE\0")) ) {
		switch (toupper(argv[3][0])) {
		case 'A':
			if (atoi(argv[4]) == 0) {
				IO_clr_ENA();
				pv_snprintfP_OK();
				return;
			}
			if (atoi(argv[4]) == 1) {
				IO_set_ENA();
				pv_snprintfP_OK();
				return;
			}
			pv_snprintfP_ERR();
			return;
		case 'B':
			if (atoi(argv[4]) == 0) {
				IO_clr_ENB();
				pv_snprintfP_OK();
				return;
			}
			if (atoi(argv[4]) == 1) {
				IO_set_ENB();
				pv_snprintfP_OK();
				return;
			}
			pv_snprintfP_ERR();
			return;
		}
	}

	// write out pulse (A/B) (+/-) (ms)
	if (!strcmp_P( strupr(argv[2]), PSTR("PULSE\0")) ) {
		DRV8814_test_pulse(argv[3],argv[4],argv[5]);
		pv_snprintfP_OK();
		return;
	}

	// write out power on|off
	if (!strcmp_P( strupr(argv[2]), PSTR("POWER\0")) ) {

		if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
			IO_set_V12_OUTS_CTL();
			pv_snprintfP_OK();
			return;
		}
		if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
			IO_clr_V12_OUTS_CTL();
			pv_snprintfP_OK();
			return;
		}
		pv_snprintfP_ERR();
		return;
	}

	pv_snprintfP_ERR();
	return;

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwGPRS(uint8_t cmd_mode )
{

uint8_t pin;
char *p;

	if ( cmd_mode == WR_CMD ) {

		// write gprs (pwr|sw|rts|dtr) {on|off}

		if (!strcmp_P( strupr(argv[2]), PSTR("PWR\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_PWR(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_PWR(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("SW\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_SW();
				pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_SW(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		if (!strcmp_P( strupr(argv[2]), PSTR("CTS\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// Por ahora cableo DTR a CTS.

		if (!strcmp_P( strupr(argv[2]), PSTR("DTR\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_GPRS_CTS(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// write gprs redial
		if (!strcmp_P( strupr(argv[2]), PSTR("REDIAL\0")) ) {
			pub_gprs_redial();
			return;
		}
		// ATCMD
		// // write gprs cmd {atcmd}
		if (!strcmp_P(strupr(argv[2]), PSTR("CMD\0"))) {
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("%s\r\0"),argv[3] );

			pub_gprs_flush_RX_buffer();
			frtos_write( fdGPRS, cmd_printfBuff, sizeof(cmd_printfBuff) );

			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("sent->%s\r\n\0"),argv[3] );
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
			return;
		}

		return;
	}

	if ( cmd_mode == RD_CMD ) {
		// read gprs (rsp,cts,dcd,ri)

			// ATCMD
			// read gprs rsp
			if (!strcmp_P(strupr(argv[2]), PSTR("RSP\0"))) {
				//p = FreeRTOS_UART_getFifoPtr(&pdUART_GPRS);
				CMD_write( "rx->", sizeof("rx->")  );
				p = pub_gprs_rxbuffer_getPtr();
				CMD_write( p, UART_GPRS_RXBUFFER_LEN );
				CMD_write( "\r\n\0", sizeof("\r\n\0")  );
				return;
			}

			// DCD
			if (!strcmp_P( strupr(argv[2]), PSTR("DCD\0")) ) {
				pin = IO_read_DCD();
				FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("DCD=%d\r\n\0"),pin);
				CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
				pv_snprintfP_OK();
				return;
			}

			// RI
			if (!strcmp_P( strupr(argv[2]), PSTR("RI\0")) ) {
				pin = IO_read_RI();
				FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("RI=%d\r\n\0"),pin);
				CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
				pv_snprintfP_OK();
				return;
			}

			// RTS
			if (!strcmp_P( strupr(argv[2]), PSTR("RTS\0")) ) {
				pin = IO_read_RTS();
				FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("RTS=%d\r\n\0"),pin);
				CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
				pv_snprintfP_OK();
				return;
			}


			pv_snprintfP_ERR();
			return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwXBEE(uint8_t cmd_mode )
{

//uint8_t pin;
//char *p;

	if ( cmd_mode == WR_CMD ) {

		// write xbee pwr {on|off}
		if (!strcmp_P( strupr(argv[2]), PSTR("PWR\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_XBEE_PWR(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_XBEE_PWR(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// write xbee sleep {on|off}
		if (!strcmp_P( strupr(argv[2]), PSTR("SLEEP\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_XBEE_SLEEP(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_XBEE_SLEEP(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// write xbee reset {on|off}
		if (!strcmp_P( strupr(argv[2]), PSTR("RESET\0")) ) {
			if (!strcmp_P( strupr(argv[3]), PSTR("ON\0")) ) {
				IO_set_XBEE_RESET(); pv_snprintfP_OK(); return;
			}
			if (!strcmp_P( strupr(argv[3]), PSTR("OFF\0")) ) {
				IO_clr_XBEE_RESET(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// MSG
		// write xbee msg
		// Transmito el msg por el puerto del XBEE al dispositivo remoto.

		if (!strcmp_P(strupr(argv[2]), PSTR("MSG\0"))) {
			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("%s\r\0"),argv[3] );
//			FreeRTOS_write( &pdUART_XBEE, cmd_printfBuff, sizeof(cmd_printfBuff) );

			FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("xbee_sent->%s\r\n\0"),argv[3] );
			CMD_write( cmd_printfBuff, sizeof(cmd_printfBuff) );
			return;
		}

		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_range(void)
{
int16_t range;

	pub_rangeMeter_ping(&range);
	FRTOS_snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("RANGE=%d\r\n\0"),range);
	CMD_write(  cmd_printfBuff, sizeof(cmd_printfBuff) );
	pv_snprintfP_OK();
	return;

}
//------------------------------------------------------------------------------------
static void pv_config_modo( char *tipo_canal, char *nro_canal, char *modo )
{
uint8_t channel;

	channel = atoi(nro_canal);

	if ( !strcmp_P( strupr(tipo_canal), PSTR("ANALOG\0"))) {
		if ( channel < NRO_ANALOG_CHANNELS ) {
			if ( !strcmp_P( strupr(modo), PSTR("LOCAL\0"))) {
				systemVars.a_ch_modo[channel] = 'L';
				pv_snprintfP_OK();
				return;
			} else if ( !strcmp_P( strupr(modo), PSTR("REMOTO\0"))) {
				systemVars.a_ch_modo[channel] = 'R';
				pv_snprintfP_OK();
				return;
			} else {
				pv_snprintfP_ERR();
				return;
			}
		}
	}

	if ( !strcmp_P( strupr(tipo_canal), PSTR("DIGITAL\0"))) {
		if ( channel < NRO_DIGITAL_CHANNELS ) {
			if ( !strcmp_P( strupr(modo), PSTR("LOCAL\0"))) {
				systemVars.d_ch_modo[channel] = 'L';
				pv_snprintfP_OK();
				return;
			} else if ( !strcmp_P( strupr(modo), PSTR("REMOTO\0"))) {
				systemVars.d_ch_modo[channel] = 'R';
				pv_snprintfP_OK();
				return;
			} else {
				pv_snprintfP_ERR();
				return;
			}
		}
	}

}
