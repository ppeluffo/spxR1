/*
 * main.c
 *
 *  Created on: 18 de oct. de 2016
 *      Author: pablo
 *
 *  avrdude -v -Pusb -c avrispmkII -p x256a3 -F -e -U flash:w:spx.hex
 *  avrdude -v -Pusb -c avrispmkII -p x256a3 -F -e
 *
 *  REFERENCIA: /usr/lib/avr/include/avr/iox256a3b.h
 *
 *  El watchdog debe estar siempre prendido por fuses.
 *  FF 0A FD __ F5 D4
 *
 *  PROGRAMACION FUSES:
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -u -Uflash:w:spx.hex:a -Ufuse0:w:0xff:m -Ufuse1:w:0x0:m -Ufuse2:w:0xff:m -Ufuse4:w:0xff:m -Ufuse5:w:0xff:m
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -u -Ufuse0:w:0xff:m -Ufuse1:w:0x0:m -Ufuse2:w:0xff:m -Ufuse4:w:0xff:m -Ufuse5:w:0xff:m
 *
 *  Para ver el uso de memoria usamos
 *  avr-nm -n spxR1.elf | more
 *
 *------------------------------------------------------------------------------------------
 * 2018-07-24:
 * - Cuando configuro las salidas solo lo hago en el systemVars. No las aplico sino que dejo
 * que la propia tarea lo haga luego.
 * Ver como mostrar la consigna aplicada !!!
 * - Incorporo la funcion de dailyReset.
 *------------------------------------------------------------------------------------------
 * 2018-07-20:
 * - El problema de perder datos se daria por el uso de las secciones criticas el leer datos
 * de los ringbuffers. Agrego un elemento a estos que indique si estan llegando datos.
 * La ISR de RX c/byte que llega lo prende.
 * Cuando voy a leer el RB veo si esta prendida. En este caso la apago y espero 10ms para volver
 * a leerla. Si esta apagada, indica que no hay acrtividad y leo. Si se prendio, salgo simulando
 * que aun no llego nada.
 * Los resultados son positivos ya que el frame de init ( 390 bytes ) se recibe por completo.
 *
 *------------------------------------------------------------------------------------------
 * 2018-07-19:
 * - La rutina de recepcion a veces pierde un caracter. Hay que perfeccionarla.
 * - Por la razon anterior, c/comando conviene chequearlo 3 veces antes de cancelar.
 * - Si no se conecta, reintentar a los 10 minutos una vez mas.
 *------------------------------------------------------------------------------------------
 * 2018-07-16:
 * Problema al imprimir un buffer (RX) mas grande que el de xprintf !!!
 * Reviso port.c para que trabaje con memoria extendida
 * Log de errores de I2C.
 * Incorporo la libreria l_ouptputs y revisar tkOutputs ( config modo )
 * Disminuyo el tiempo en el intercambio de consignas.
 * Revisar funciones de Cmd.
 *------------------------------------------------------------------------------------------
 * 2018-07-15:
 * Arreglar el tema de la lectura de los INA.
 * El tiempo por defecto para tomar los semaforos pasa a 5 ticks.
 * Modem: Analisis de rxdata/tamanio de buffers, etc, netopen fail
 *------------------------------------------------------------------------------------------
 * 2018-07-14:
 * Agrego buffers y estructura para BT y XBEE.
 * Modifico la libreria l_printf y elimino l_uarts.
 * Pongo un semaforo en l_file para acceder a la FAT
 * Paso los semaforos a estructuras estaticas.
 * Agrego la libreria l_ain
 * Modifico el menu para incorporar el concepto de usuario tecnico.
 * Los mensajes de init de las tareas los paso a luego de inicializarlas.
 * Reviso libreria NVM para poder leer el id
 * Agrego que se pueda ver el tiempo que falta para el siguiente poleo
 *------------------------------------------------------------------------------------------
 * 2018-07-13:
 * Implemento un sistema de impresion en consola xPrintf.
 * Eliminamos todos los buffers locales.
 * Elimino FRTOS-stdio
 * Revisamos las librerias l_i2c/l_eeprom/l_ina3221/l_rtc79410 y las funciones asociadas en tkCmd.
 *------------------------------------------------------------------------------------------
 * 2018-07-12:
 * - Previo elimino todo lo que tiene que ver con SPI que aun no esta implementado.
 * - En test probe el nuevo modelo de drivers y frtos en capas con servicios verticales.
 *   Hago la migracion.
 * - Migro el FRTOS a version 10.
 *------------------------------------------------------------------------------------------
 * 2018-06-27:
 * - Trasmito en los init la potencia del modem en DBM y no CSQ
 * - No trasmito la configuracion de los canales que estan apagados
 *
 * - Las lineas del CGI terminan en \r\n y no en \n. ( Apache 2.4)
 * 	https://en.wikipedia.org/wiki/HTTP_message_body
 * 	http://forum.arduino.cc/index.php?topic=200753.0
 * 	https://www3.ntu.edu.sg/home/ehchua/programming/webprogramming/HTTP_Basics.html
 * 	https://stackoverflow.com/questions/5757290/http-header-line-break-style
 *
 *------------------------------------------------------------------------------------------
 * SCHEDULE:
 * *********
 *  Ampliar el buffer de GPRS_TX
 *
 * HACER:
 * Leer Memoria BD
 *
 * NOTAS DE DISENO:
 * ****************
 *  1- En FRTOS_Write_UART cambio el taskYIELD por taskDelay porque sino se cuelga.
 *     Este delay hacia que los mensajes del cmdmode fuesen lentos y entonces cambio en cmdline.c
 *     la forma de mostrarlos usando directamente FRTOS-IO.
 *
 * PENDIENTE:
 * **********
 * Hacer andar el watchdog
 * Cambiar la velocidad y reconffigurar el BT
 * Configuro el RTC.
 * Rutinas de calendario.
 *
 *
 * Features V0.0.1
 * - Si la terminal no esta conectada, CMD_write no escribe nada.
 * - El FRTOS siempre opera en modo tickless.
 *
 *  XBEE:
 *  Cuando el modo es slave, no uso la tarea de GPRS ( siempre duerme ) y transmito
 *  el frame al remoto.
 *  En esta version, los canales de slave se mapean en los del remoto de modo que el
 *  servidor no se entera de donde vienen.
 
 */


#include "spx.h"

//----------------------------------------------------------------------------------------
// http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_softreset.html
// http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
//
// Function Pototype
//uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
//void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

// Function Implementation
/*
void wdt_init(void)
{
    // Como los fusibles estan para que el WDG siempre este prendido, lo reconfiguro a 8s lo
    // antes posible
	WDT_EnableAndSetTimeout(  WDT_PER_8KCLK_gc );

    return;
}
*/
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
int main( void )
{
	// Clock principal del sistema
	u_configure_systemMainClock();
	u_configure_RTC32();

	// Configuramos y habilitamos el watchdog a 8s.
	WDT_EnableAndSetTimeout(  WDT_PER_8KCLK_gc );
	if ( WDT_IsWindowModeEnabled() )
		WDT_DisableWindowMode();

	set_sleep_mode(SLEEP_MODE_PWR_SAVE);

	initMCU();

	frtos_open(fdUSB, 115200);
	frtos_open(fdGPRS, 115200);
	frtos_open(fdBT, 9600);
	frtos_open(fdI2C, 100 );

	// Creo los semaforos
	sem_SYSVars = xSemaphoreCreateMutexStatic( &SYSVARS_xMutexBuffer );
	xprintf_init();
	FAT_init();

	startTask = false;

	xTaskCreate(tkCtl, "CTL", tkCtl_STACK_SIZE, NULL, tkCtl_TASK_PRIORITY,  &xHandle_tkCtl );
	xTaskCreate(tkCmd, "CMD", tkCmd_STACK_SIZE, NULL, tkCmd_TASK_PRIORITY,  &xHandle_tkCmd);
	xTaskCreate(tkData, "DATA", tkData_STACK_SIZE, NULL, tkData_TASK_PRIORITY,  &xHandle_tkData);
	xTaskCreate(tkDigital, "DIGI", tkDigital_STACK_SIZE, NULL, tkDigital_TASK_PRIORITY,  &xHandle_tkDigital);
	xTaskCreate(tkGprsRx, "RX", tkGprs_rx_STACK_SIZE, NULL, tkGprs_rx_TASK_PRIORITY,  &xHandle_tkGprsRx );
	xTaskCreate(tkGprsTx, "TX", tkGprs_tx_STACK_SIZE, NULL, tkGprs_tx_TASK_PRIORITY,  &xHandle_tkGprsTx );
	xTaskCreate(tkOutputs, "OUT", tkOutputs_STACK_SIZE, NULL, tkOutputs_TASK_PRIORITY,  &xHandle_tkOutputs );
	xTaskCreate(tkXbee, "XBEE", tkXbee_STACK_SIZE, NULL, tkXbee_TASK_PRIORITY,  &xHandle_tkXbee );

	/* Arranco el RTOS. */
	vTaskStartScheduler();

	// En caso de panico, aqui terminamos.
	exit (1);

}
//-----------------------------------------------------------
void vApplicationIdleHook( void )
{
	// Como trabajo en modo tickless no entro mas en modo sleep aqui.
//	if ( sleepFlag == true ) {
//		sleep_mode();
//	}
}

//-----------------------------------------------------------
/* Define the function that is called by portSUPPRESS_TICKS_AND_SLEEP(). */
//------------------------------------------------------------------------------------
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
	// Es invocada si en algun context switch se detecta un stack corrompido !!
	// Cuando el sistema este estable la removemos.
	// En FreeRTOSConfig.h debemos habilitar
	// #define configCHECK_FOR_STACK_OVERFLOW          2

	xprintf_P( PSTR("PANIC:%s !!\r\n\0"),pcTaskName);

}
//------------------------------------------------------------------------------------

/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
//------------------------------------------------------------------------------------

