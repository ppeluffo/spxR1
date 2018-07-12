/*
 * spx_tkGprs.h
 *
 *  Created on: 25 de oct. de 2017
 *      Author: pablo
 */

#ifndef SRC_SPX_TKGPRS_SPX_TKGPRS_H_
#define SRC_SPX_TKGPRS_SPX_TKGPRS_H_

#include "frtos-io.h"
#include "spx.h"

char gprs_printfBuff[CHAR256];

#define MAX_HW_TRIES_PWRON 		3	// Intentos de prender HW el modem
#define MAX_SW_TRIES_PWRON 		3	// Intentos de prender SW el modem
#define MAX_TRYES_NET_ATTCH		6	// Intentos de atachearme a la red GPRS
#define MAX_IP_QUERIES 			4	// Intentos de conseguir una IP
#define MAX_INIT_TRYES			4	// Intentos de procesar un frame de INIT
#define MAX_TRYES_OPEN_SOCKET	4 	// Intentos de abrir un socket
#define MAX_RCDS_WINDOW_SIZE	10	// Maximos registros enviados en un bulk de datos
#define MAX_TX_WINDOW_TRYES		4	// Intentos de enviar el mismo paquete de datos

#define UART_GPRS_RXBUFFER_LEN 256
struct {
	char buffer[UART_GPRS_RXBUFFER_LEN];
	uint16_t ptr;
} pv_gprsRxCbuffer;

#define IMEIBUFFSIZE	24
char buff_gprs_imei[IMEIBUFFSIZE];

typedef enum { G_ESPERA_APAGADO = 0, G_PRENDER, G_CONFIGURAR, G_MON_SQE, G_GET_IP, G_INIT_FRAME, G_DATA } t_gprs_states;
typedef enum { SOCK_CLOSED = 0, SOCK_OPEN } t_socket_status;

struct {
	bool modem_prendido;
	bool signal_redial;
	bool signal_frameReady;
	bool monitor_sqe;
	uint8_t state;
	uint8_t	dcd;

} GPRS_stateVars;

#define bool_CONTINUAR	true
#define bool_RESTART	false

//bool gprs_esperar_apagado(void);
bool st_gprs_esperar_apagado(void);
bool st_gprs_prender(void);
bool st_gprs_configurar(void);
bool st_gprs_monitor_sqe(void);
bool st_gprs_get_ip(void);
bool st_gprs_init_frame(void);
bool st_gprs_data(void);

void pv_gprs_rxbuffer_flush(void);
void pv_gprs_rxbuffer_push(char c);
void pv_gprs_init_system(void);

bool pub_gprs_check_response( const char *rsp );
void pub_gprs_modem_pwr_off(void);
void pub_gprs_modem_pwr_on(void);
void pub_gprs_modem_pwr_sw(void);
void pub_gprs_flush_RX_buffer(void);

char *pub_gprs_rxbuffer_getPtr(void);

void pub_gprs_open_socket(void);

bool pub_gprs_try_to_open_socket(void);

t_socket_status pub_gprs_check_socket_status(void);
void pub_gprs_print_RX_response(void);
void pub_gprs_print_RX_Buffer(void);
void pub_gprs_ask_sqe(void);

#endif /* SRC_SPX_TKGPRS_SPX_TKGPRS_H_ */
