// STANDARD INCLUDES
#include <stdio.h>l.g 
#include <conio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// KERNEL INCLUDES
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

// HARDWARE SIMULATOR UTILITY FUNCTIONS  
#include "HW_access.h"


// SERIAL SIMULATOR CHANNEL TO USE 
#define COM_CH_0 (0)


// TASK PRIORITIES 
#define	TASK_SERIAL_SEND_PRI		( tskIDLE_PRIORITY + 2 )
#define TASK_SERIAl_REC_PRI			( tskIDLE_PRIORITY + 3 )
#define	SERVICE_TASK_PRI			( tskIDLE_PRIORITY + 1 )


// TASKS: FORWARD DECLARATIONS 
void LEDBar_Task(void* pvParameters);
void SerialSend_Task(void* pvParameters);
void SerialReceive_Task(void* pvParameters);


// TRASNMISSION DATA - CONSTANT IN THIS APPLICATION 
const char trigger[] = "XYZ";
unsigned volatile t_point;


// RECEPTION DATA BUFFER - COM 0
#define R_BUF_SIZE (32)
uint8_t r_buffer[R_BUF_SIZE];
unsigned volatile r_point;


// 7-SEG NUMBER DATABASE - ALL HEX DIGITS [ 0 1 2 3 4 5 6 7 8 9 A B C D E F ]
static const char hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };


// GLOBAL OS-HANDLES 
SemaphoreHandle_t LED_INT_BinarySemaphore;
SemaphoreHandle_t TBE_BinarySemaphore;
SemaphoreHandle_t RXC_BinarySemaphore;
QueueHandle_t LEDBar_Queue;


// STRUCTURES


// INTERRUPTS //
static uint32_t OnLED_ChangeInterrupt(void) {	// OPC - ON INPUT CHANGE - INTERRUPT HANDLER 
	BaseType_t xHigherPTW = pdFALSE;

	xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &xHigherPTW);
	portYIELD_FROM_ISR(xHigherPTW);
}

static uint32_t prvProcessTBEInterrupt(void) {	// TBE - TRANSMISSION BUFFER EMPTY - INTERRUPT HANDLER 
	BaseType_t xHigherPTW = pdFALSE;

	xSemaphoreGiveFromISR(TBE_BinarySemaphore, &xHigherPTW);
	portYIELD_FROM_ISR(xHigherPTW);
}

static uint32_t prvProcessRXCInterrupt(void) {	// RXC - RECEPTION COMPLETE - INTERRUPT HANDLER 
	BaseType_t xHigherPTW = pdFALSE;

	if (get_RXC_status(0) != 0)
		xSemaphoreGiveFromISR(RXC_BinarySemaphore, &xHigherPTW);

	portYIELD_FROM_ISR(xHigherPTW);
}


// MAIN - SYSTEM STARTUP POINT 
void main_demo(void) {
	// INITIALIZATION OF THE PERIPHERALS
	//init_7seg_comm();
	init_LED_comm();
	init_serial_uplink(COM_CH_0);		// inicijalizacija serijske TX na kanalu 0
	init_serial_downlink(COM_CH_0);	// inicijalizacija serijske RX na kanalu 0


	// INTERRUPT HANDLERS
	vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);		// ON INPUT CHANGE INTERRUPT HANDLER 
	vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);	// SERIAL TRANSMITT INTERRUPT HANDLER 
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);	// SERIAL RECEPTION INTERRUPT HANDLER 

	// BINARY SEMAPHORES
	LED_INT_BinarySemaphore = xSemaphoreCreateBinary();	// CREATE LED INTERRUPT SEMAPHORE 
	TBE_BinarySemaphore = xSemaphoreCreateBinary();		// CREATE TBE SEMAPHORE - SERIAL TRANSMIT COMM 
	RXC_BinarySemaphore = xSemaphoreCreateBinary();		// CREATE RXC SEMAPHORE - SERIAL RECEIVE COMM

	// QUEUES
	LEDBar_Queue = xQueueCreate(2, sizeof(uint8_t));

	// TASKS 
	xTaskCreate(SerialSend_Task, "STx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);	// SERIAL TRANSMITTER TASK 
	xTaskCreate(SerialReceive_Task, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);	// SERIAL RECEIVER TASK 
	r_point = 0;
	xTaskCreate(LEDBar_Task, "ST", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);				// CREATE LED BAR TASK  


	// START SCHEDULER
	vTaskStartScheduler();
	while (1);
}

// TASKS: IMPLEMENTATIONS
void LEDBar_Task(void* pvParameters) {
	uint8_t LEDsPattern;
	while (1) {
		xQueueReceive(LEDBar_Queue, &LEDsPattern, portMAX_DELAY);		
		set_LED_BAR(0, LEDsPattern);
	}
}

void SerialSend_Task(void* pvParameters) {
	t_point = 0;
	while (1) {
		if (t_point > (sizeof(trigger) - 1))
			t_point = 0;
		send_serial_character(COM_CH_0, trigger[t_point++]);
		xSemaphoreTake(TBE_BinarySemaphore, portMAX_DELAY);// kada se koristi predajni interapt
		//vTaskDelay(pdMS_TO_TICKS(100));// kada se koristi vremenski delay
	}
}

void SerialReceive_Task(void* pvParameters) {
	uint8_t cc = 0;
	uint8_t dataToSend;
	while (1) {
		xSemaphoreTake(RXC_BinarySemaphore, portMAX_DELAY);// ceka na serijski prijemni interapt
		get_serial_character(COM_CH_0, &cc);//ucitava primljeni karakter u promenjivu cc
		printf("KANAL 0: primio karakter: %u\n", (unsigned)cc);// prikazuje primljeni karakter u cmd prompt

		if (cc == 0xef) {	// EF oznacava POCETAK poruke
			r_point = 0;
		}
		else if (cc == 0xff) {	// za svaki KRAJ poruke, prikazati primljenje bajtove direktno na displeju 3-4
			dataToSend = (uint8_t)r_buffer[0];
			xQueueSend(LEDBar_Queue, &dataToSend, portMAX_DELAY);
		}
		else if (r_point < R_BUF_SIZE) { // pamti karaktere izmedju EF i FF
			r_buffer[r_point++] = cc;
		}
	}
}