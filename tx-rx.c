/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Periodic Advertisement Program
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "sl_sleeptimer.h"

/* Timer and GPIO libraries*/
#include "em_timer.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_msc.h"

#include "app.h"
#include "em_rtcc.h"


// Default clock value
#define HFPERCLK_IN_MHZ 19

// Number of timer overflows since last interrupt;
static volatile uint32_t overflowCount = 0;


/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;

static uint8_t connecting = 0;

struct connection_params {
	bd_addr     local_addr;
	bd_addr	    connection_addr;
	uint8_t     connection_handle;
	uint16 		min_interval;
	uint16 		max_interval;
	uint16 		latency;
	uint16 		timeout;
	uint16		min_cte_length;
	uint16		max_cte_length;
};

struct CTE_RX_params {
	uint16 		interval;
	uint8 		cte_length;
	uint8 		cte_type;
	uint8 		slot_durations;
	uint8 		s_len;
	uint8 		sa[1];
};
struct CTE_TX_params {
	uint8 		cte_types;
	uint8 		s_len;
	uint8 		sa[1];
};

static struct connection_params conn_params;
static struct CTE_RX_params RX_params;
static struct CTE_TX_params TX_params;


bool compare_bd_addr(bd_addr *address1, bd_addr *address2){
	for(int i = 0;i<5;i++){
		if(address1->addr[i] != address2->addr[5-i]){
			return false;
		}
	}
	return true;
}
bool becomeRX(){
	for(int i = 0;i<5;i++){
		if(conn_params.connection_addr.addr[5-i] < conn_params.local_addr.addr[5-i]){
			return true;
		}
		else if(conn_params.connection_addr.addr[5-i] > conn_params.local_addr.addr[5-i]){
			return false;
		}
	}
	return true;
}
bd_addr test_device1 = {
		.addr = {0x58,0x8E,0x81,0xA5,0x53,0xB0}
};
bd_addr test_device2 = {
		.addr = {0x58,0x8E,0x81,0xA5,0x47,0xC8}
};

void initTimer(void){

	// Enable clock to GPIO and TIMER0
	//CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);

	// Configure PA6 as input
	//GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);

	// Initialize the timer
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	// Configure TIMER0 Compare/Capture for output compare
	//TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;

	timerInit.prescale = timerPrescale64;
	timerInit.enable = false;
	//timerCCInit.edge = timerEdgeFalling;
	//timerCCInit.mode = timerCCModeCapture;

	//TIMER_IntEnable(TIMER0, TIMER_IF_OF);
	//NVIC_EnableIRQ(TIMER0_IRQn);

	//TIMER_TopSet(TIMER0,CMU_ClockFreqGet(cmuClock_TIMER0));
	TIMER_Init(TIMER0, &timerInit);

	// Route Timer0 CC0 output to PA3
	//GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
	//GPIO->TIMERROUTE[0].CC0ROUTE = (gpioPortD << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
	//				| (3 << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

	//TIMER_InitCC(TIMER0, 0, &timerCCInit);
}

uint32_t calculatePeriod(void){

	// Pause timer
	TIMER_Enable(TIMER0, false);

	uint32_t current_edge = TIMER_CounterGet(TIMER0);

	uint32_t period = ((TIMER_TopGet(TIMER0) + 2) * overflowCount + current_edge);
					// / (HFPERCLK_IN_MHZ * (1 << timerPrescale1));

	//lastCapturedEdge = current_edge;

	// Reset overflow count and timer
	overflowCount = 0;
	TIMER_CounterSet(TIMER0, 0);
	TIMER_Enable(TIMER0, true);

	return period;
}


int itr = 0;
void TIMER0_IRQHandler(void){
	TIMER_IntClear(TIMER0, TIMER_IF_OF);
	//printf("Overflow: %lu\r\n", overflowCount);
	overflowCount++;
	return;
	/*
	if(itr == 0){
		itr++;
		//switch tx and rx
		if(becomeRX()){
			gecko_cmd_cte_receiver_disable_connection_cte(conn_params.connection_handle);
			uint16 tx_result = gecko_cmd_cte_transmitter_enable_connection_cte(conn_params.connection_handle, TX_params.cte_types,
															  TX_params.s_len, TX_params.sa)->result;
			//printf("Result of SECOND tx_enable_connection_cte: 0x%x\r\n", tx_result);
		}
		else {
			gecko_cmd_cte_transmitter_disable_connection_cte(conn_params.connection_handle);
			uint16 rx_result = gecko_cmd_cte_receiver_enable_connection_cte(conn_params.connection_handle, RX_params.interval,
												  RX_params.cte_length, RX_params.cte_type, RX_params.slot_durations, RX_params.s_len, RX_params.sa)->result;
			//printf("Result of SECOND rx_enable_connection_cte: 0x%x\r\n", rx_result);
		}
	}
	else{
		//printf("Exiting Program\r\n");
		exit(0);
	}
	*/
}

/* Main application */
void appMain(gecko_configuration_t *pconfig){

#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

	uint32_t delta_t;


	/* Set maximum number of periodic advertisers */
	pconfig->bluetooth.max_advertisers = 1;

	/* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
	initLog();

	/* Initialize stack */
	gecko_init(pconfig);

	// Initialize CTE
	gecko_bgapi_class_cte_transmitter_init();
	gecko_bgapi_class_cte_receiver_init();

	// Initialize Connection Parameters
	conn_params.min_interval 	= 0x0006;
	conn_params.max_interval 	= 0x0006;
	conn_params.latency 		= 0x0000;
	conn_params.timeout 		= 0x000a;
	conn_params.min_cte_length 	= 0x0000;
	conn_params.max_cte_length 	= 0xffff;

	// Initialize CTE RX Parameters
	RX_params.interval 			= 1;
	RX_params.cte_length 		= 0x14;
	RX_params.cte_type 			= 1;
	RX_params.slot_durations 	= 1;
	RX_params.s_len 			= 1;
	RX_params.sa[0] 			= 0;

	// Initialize CTE TX Parameters
	TX_params.cte_types 		= 0x07;
	TX_params.s_len 			= 1;
	TX_params.sa[0] 			= 0;
	int counter = 0;

	initTimer();


	while (1) {
		/* Event pointer for handling events */
		struct gecko_cmd_packet* evt;

		/* if there are no events pending then the next call to gecko_wait_event() may cause
		   * device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
		if (!gecko_event_pending()) {
			flushLog();
		}

		/* Check for stack event. */
		evt = gecko_wait_event();
		uint16 result;


		/* Handle events */
		switch (BGLIB_MSG_ID(evt->header)) {
			/* This boot event is generated when the system boots up after reset.
			 * Do not call any stack commands before receiving the boot event.
			 * Here the system is set to start advertising immediately after boot procedure. */
			case gecko_evt_system_boot_id:
				bootMessage(&(evt->data.evt_system_boot));

				uint8_t channel_map_data[5] = {3, 0, 0, 0, 0};
				result = gecko_cmd_le_gap_set_data_channel_classification(5, channel_map_data)->result;
				//printLog("set data channel classification: %d\r\n", result);

				gecko_cmd_system_set_tx_power(100);
				gecko_cmd_le_gap_set_advertise_tx_power(0,30);
				gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);
				result = gecko_cmd_le_gap_set_conn_timing_parameters(conn_params.min_interval,
						conn_params.max_interval, conn_params.latency,conn_params.timeout, conn_params.min_cte_length, conn_params.max_cte_length)->result;
				printf("Result of gecko_cmd_le_gap_set_conn_timing_parameters: 0x%x\r\n", result);
				//Start scanning
				gecko_cmd_le_gap_set_discovery_timing(le_gap_phy_1m, 20, 10);
				gecko_cmd_le_gap_start_discovery(le_gap_phy_1m, le_gap_discover_generic);

				// Start extended advertising
				result = gecko_cmd_le_gap_start_advertising(0,le_gap_general_discoverable, le_gap_connectable_scannable)->result;
				//printf("le_gap_start_advertising() returns 0x%X\r\n", result);
			break;

			case gecko_evt_le_gap_scan_response_id: {
				if(compare_bd_addr(&conn_params.local_addr, &evt->data.evt_le_gap_scan_response.address)){
					//printf("Same device\r\n");
				break;
				}
				bool is_right_device = compare_bd_addr(&test_device1,&evt->data.evt_le_gap_scan_response.address)
									 | compare_bd_addr(&test_device2,&evt->data.evt_le_gap_scan_response.address);
				if (!connecting && is_right_device){
					//printLog("\r\nDevice found\r\n");
					//printLog("Initiating connection\r\n");

					/*
					printf("advertisement/scan response from: ");
					for (int i = 0; i < 5; i++) {
					printLog("%2.2x:", conn_params.connection_addr.addr[5 - i]);
					}
					printLog("%2.2x\r\n", conn_params.connection_addr.addr[0]);
					*/

					// RX initiates connection
					memcpy(&conn_params.connection_addr, &evt->data.evt_le_gap_scan_response.address,sizeof(bd_addr));
					if(becomeRX())
					  gecko_cmd_le_gap_connect(evt->data.evt_le_gap_scan_response.address, evt->data.evt_le_gap_scan_response.address_type, 1);

					/* Connection is under way. As we are scanning in the background the next advertisement packet will also be reported.
					This flag ensures that we don't enter this piece of code again when the connection is being established */
					connecting = 1;
				}
			  }
			break;

			case gecko_evt_le_connection_opened_id:
				//printf("We got a CONNECTION\r\n");
				connecting = 0;
				// Stop scanning & advertising
				result = gecko_cmd_le_gap_end_procedure()->result;
				printf("Result of gecko_cmd_le_gap_end_procedure: 0x%x\r\n", result);

				result = gecko_cmd_le_gap_stop_advertising(0)->result;
				printf("Result of gecko_cmd_le_gap_stop_advertising: 0x%x\r\n", result);

				//conn_params.connection_handle = evt->data.evt_le_connection_opened.connection;
				//conn_params.connection_addr = evt->data.evt_le_connection_opened.address;
				memcpy(&conn_params.connection_handle, &evt->data.evt_le_connection_opened.connection, 1);
				memcpy(&conn_params.connection_addr, &evt->data.evt_le_connection_opened.address, sizeof(bd_addr));

				// Set connection parameters
				//result = gecko_cmd_le_connection_set_timing_parameters(conn_params.connection_handle, conn_params.min_interval,
					//	conn_params.max_interval, conn_params.latency,conn_params.timeout, conn_params.min_cte_length, conn_params.max_cte_length)->result;
				//printf("Result of gecko_cmd_le_connection_set_timing_parameters: 0x%x\r\n", result);
				result = gecko_cmd_le_connection_set_preferred_phy(conn_params.connection_handle, 0x02, 0x02)->result;
				printf("Result of gecko_cmd_le_connection_set_preferred_phy: 0x%x\r\n", result);
				/*
				printLog("LOCAL device address: ");
				for (int i = 0; i < 5; i++) {
					printLog("%2.2x:", conn_params.local_addr.addr[5 - i]);
				}
				printLog("%2.2x\r\n", conn_params.local_addr.addr[0]);

				printLog("REMOTE device address: ");
				for (i = 0; i < 5; i++) {
					printLog("%2.2x:", conn_params.connection_addr.addr[5 - i]);
				}
				printLog("%2.2x\r\n", conn_params.connection_addr.addr[0]);
				*/
				if(becomeRX()){
					uint16 rx_result = gecko_cmd_cte_receiver_enable_connection_cte(conn_params.connection_handle, RX_params.interval,
										  RX_params.cte_length, RX_params.cte_type, RX_params.slot_durations, RX_params.s_len, RX_params.sa)->result;
					printf("Result of rx_enable_connection_cte: 0x%x\r\n", rx_result);
				}
				else{
					uint16 tx_result = gecko_cmd_cte_transmitter_enable_connection_cte(conn_params.connection_handle, TX_params.cte_types,
														  TX_params.s_len, TX_params.sa)->result;
					printf("Result of tx_enable_connection_cte: 0x%x\r\n", tx_result);
				}
				//if(becomeRX())
					//TIMER_Init(TIMER0, &timerInit);

			break;


			case gecko_evt_le_connection_closed_id:
				/* Check if need to boot to dfu mode */
				if (boot_to_dfu) {
					/* Enter to DFU OTA mode */
					gecko_cmd_system_reset(2);
				}
				else {
					//disable CTE
					if(becomeRX())
						gecko_cmd_cte_transmitter_disable_connection_cte(conn_params.connection_handle);
					else
						gecko_cmd_cte_receiver_disable_connection_cte(conn_params.connection_handle);
					// Restart advertising and scanning
					gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
					gecko_cmd_le_gap_start_discovery(le_gap_phy_1m,le_gap_discover_observation);
				}
			break;


			case gecko_evt_le_connection_parameters_id: {
				printf("SET CONN PARAM \r\n");
				struct gecko_msg_le_connection_parameters_evt_t* r = &(evt->data.evt_le_connection_parameters);

				printf("Interval: %d \r\n", r->interval);
				//printf("Interval: %d \r\n", r->interval);
				//printf("Interval: %d \r\n", r->interval);
			}
			break;

			case gecko_evt_cte_receiver_connection_iq_report_id: {

				if(itr == 1){

					delta_t = calculatePeriod();

					struct gecko_msg_cte_receiver_connection_iq_report_evt_t *report =
							&(evt->data.evt_cte_receiver_connection_iq_report);


					uint32_t offset = (4+3+report->samples.len);


					uint8_t * userDataPage = (uint8_t *) RAM_MEM_BASE;
					//uint8_t *temp = (uint8_t *) &delta_t;
					uint8_t preData[] = {
						0xFF,
						report->channel,
						report->samples.len,

					};
					memcpy(RAM_MEM_BASE+offset*counter, preData, sizeof(preData));
					memcpy(RAM_MEM_BASE+offset*counter+sizeof(preData), &delta_t, sizeof(delta_t));
					memcpy(RAM_MEM_BASE+offset*counter+sizeof(preData)+sizeof(delta_t), &report->samples.data, report->samples.len);
					//printf("dt: %lu\r\n", delta_t);

					counter++;
					if(counter == 8){

						for(int k=0; k<counter; k++){
							uint8_t* f = malloc(1);
							uint8_t* ch = malloc(1);
							uint8_t* len = malloc(1);
							uint32_t* dt = malloc(4);
							uint8_t* data = malloc(report->samples.len);

							memcpy(f, userDataPage + 0*sizeof(uint8_t) + offset*k,sizeof(uint8_t));
							for(int i=0; i< 4; i++)
								RETARGET_WriteChar(*f);

							memcpy(ch, userDataPage + 1*sizeof(uint8_t) + offset*k,sizeof(uint8_t));
							RETARGET_WriteChar(*ch);

							memcpy(len, userDataPage + 2*sizeof(uint8_t) + offset*k,sizeof(uint8_t));
							RETARGET_WriteChar(*len);

							memcpy(dt, userDataPage + 3*sizeof(uint8_t) + offset*k,sizeof(uint32_t));
							uint8_t* temp = (uint8_t *) dt;
							for (int i=0; i<4; i++) {
								RETARGET_WriteChar(temp[i]);
							}
							memcpy(data, userDataPage + 3*sizeof(uint8_t) + sizeof(uint32_t) + offset*k, report->samples.len);
							for (int i=0; i<report->samples.len; i++) {
								RETARGET_WriteChar(data[i]);
							}

							RETARGET_WriteChar('\r');
							RETARGET_WriteChar('\n');
							free(f);
							free(ch);
							free(len);
							free(dt);
							free(data);

						}
						//counter=0;
						exit(0);
					}
				}
				else{
					itr++;
					TIMER_Enable(TIMER0, true);
				}




				//RETARGET_WriteChar(ch);

				/*
				RETARGET_WriteChar(report->channel);
				RETARGET_WriteChar(report->samples.len);
				uint8_t* temp = (uint8_t *) &delta_t;
				for (int i=0; i<4; i++) {
					RETARGET_WriteChar(temp[i]);
				}


				for (int i=0; i<report->samples.len; i++) {
					RETARGET_WriteChar(report->samples.data[i]);
				}
				RETARGET_WriteChar('\r');
				RETARGET_WriteChar('\n');
				*/
				/*
				printf("GOT CONNECTION IQ report\r\n");

				printf("status: %d, event:%d, phy: %d, ch: %d, rssi: %d, ant:%d, cte_type:%d, slot_duration:%d, delta_t:%lu, len:%d \r\n",
						report->status,report->event_counter, report->phy, report->channel, report->rssi, report->rssi_antenna_id,
						report->cte_type, report->slot_durations, delta_t, report->samples.len);

				for (int i=0; i < report->samples.len; i++) {
					RETARGET_WriteChar(report->samples.data[i]);
				}
				*/


				//printf("%lu \r\n", delta_t);




			}
			break;

			default:
			  break;
		}
	}
}

/* Print stack version and local Bluetooth address as boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{

#if DEBUG_LEVEL
  int i;

  //printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor, bootevt->patch);
  conn_params.local_addr = gecko_cmd_system_get_bt_address()->address;

  //printLog("local BT device address: ");
  for (i = 0; i < 5; i++) {
    //printLog("%2.2x:", conn_params.local_addr.addr[5 - i]);
  }
  //printLog("%2.2x\r\n", conn_params.local_addr.addr[0]);
#endif
}


