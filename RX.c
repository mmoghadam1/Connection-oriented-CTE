/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Periodic Advertisement Scanner Program
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
#include "app.h"

/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

/* Flag for indicating DFU Reset must be performed */
static uint8_t boot_to_dfu = 0;

static uint8_t     connection;
static uint8_t     connecting = 0;

const uint8 periodicSyncService[16] = {0x81,0xc2,0x00,0x2d,0x31,0xf4,0xb0,0xbf,0x2b,0x42,0x49,0x68,0xc7,0x25,0x71,0x41};

bool compare_bd_addr(bd_addr *address1, bd_addr *address2){
	for(int i = 0;i<5;i++){
		if(address1->addr[i] != address2->addr[5-i]){
			return false;
		}
	}
	return true;
}
bd_addr test_device = {
		.addr = {0x58,0x8E,0x81,0xA5,0x47,0xC8}
};

/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

  /* Set the maximum number of periodic sync allowed*/
  pconfig->bluetooth.max_periodic_sync = 1;

  /* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
  initLog();

  /* Initialize stack */
  gecko_init(pconfig);

  //Initialize CTE Reciever
  gecko_bgapi_class_cte_receiver_init();


  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;


    /* if there are no events pending then the next call to gecko_wait_event() may cause
     * device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
    if (!gecko_event_pending()) {
      flushLog();
    }

    /* Check for stack event. This is a blocking event listener. If you want non-blocking please see UG136. */
    evt = gecko_wait_event();

    /* Handle events */
	switch (BGLIB_MSG_ID(evt->header)) {
	  /* This boot event is generated when the system boots up after reset.
	   * Do not call any stack commands before receiving the boot event.
	   * Here the system is set to start advertising immediately after boot procedure. */
	  case gecko_evt_system_boot_id:
		  bootMessage(&(evt->data.evt_system_boot));
		  printLog("Connected BT Boot\r\n");

		  gecko_cmd_le_gap_set_discovery_timing(le_gap_phy_1m, 20, 10);
		  gecko_cmd_le_gap_start_discovery(le_gap_phy_1m, le_gap_discover_generic);
		  printLog("\r\nScanning\r\n");

		  gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
		  printLog("\r\nAdvertising...\r\n");
		break;

	  case gecko_evt_le_connection_opened_id:
		  printf("We got a connection!\r\n");
		  connecting = 0;
		  gecko_cmd_le_gap_end_procedure();
		  uint16 interval = 5;
		  uint8 cte_length = 0x02;
		  uint8 cte_type = 0;
		  uint8 slot_durations = 1;
		  uint8 s_len = 1;
		  uint8 sa[1] = { 0 };
		  uint16 rx_result = gecko_cmd_cte_receiver_enable_connection_cte(connection, interval, cte_length, cte_type, slot_durations, s_len, sa)->result;
		  printf("Result of rx_enable_connection_cte: 0x%x\r\n", rx_result);
		  break;
	  case gecko_evt_le_connection_closed_id:
		  printf("Connection Closed\r\n");
		/* Check if need to boot to dfu mode */
		if (boot_to_dfu) {
		  /* Enter to DFU OTA mode */
		  gecko_cmd_system_reset(2);
		} else {
		  /* Restart advertising after client has disconnected */
		  gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
		}
		break;

	  case gecko_evt_le_gap_scan_response_id:
	  {
		  bd_addr *remote_address = &evt->data.evt_le_gap_scan_response.address;
		  if (connecting == 0 && compare_bd_addr(&test_device,&evt->data.evt_le_gap_scan_response.address)){

			  /* Aggregating print statements for better code readibility */

			  printf("advertisement/scan response from %2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X\r\n", remote_address->addr[5],remote_address->addr[4], remote_address->addr[3], remote_address->addr[2], remote_address->addr[1],  remote_address->addr[0]);

			  printLog("\r\nHTM device found\r\n");
			  printLog("Initiate connection\r\n");

			  connection = gecko_cmd_le_gap_connect(evt->data.evt_le_gap_scan_response.address, evt->data.evt_le_gap_scan_response.address_type, 1)->connection;

			  /* Connection is under way. As we are scanning in the background the next advertisement packet will also be reported.
				This flag ensures that we don't enter this piece of code again when the connection is being established */
			  connecting = 1;


			  /* Set fail safe 10 second timeout to stop connection process if it doesn't go through
			  * for a variety of reasons (e.g. device going out of range) */
			  gecko_cmd_hardware_set_soft_timer(10 * 32768, 0, true);

		  }
		   break;
	  }

	  case gecko_evt_gatt_server_user_write_request_id:

		if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
		  /* Set flag to enter to OTA mode */
		  boot_to_dfu = 1;
		  /* Send response to Write Request */
		  gecko_cmd_gatt_server_send_user_write_response(
			evt->data.evt_gatt_server_user_write_request.connection,
			gattdb_ota_control,
			bg_err_success);

		  /* Close connection to enter to DFU OTA mode */
		  gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
		}
		break;
	  case gecko_evt_cte_receiver_connectionless_iq_report_id: {
		    printf("GOT CONNECTIONLESS IQ report\r\n");
		    struct gecko_msg_cte_receiver_connectionless_iq_report_evt_t report = evt->data.evt_cte_receiver_connectionless_iq_report;
			printf("status: %d, ch: %d, rssi: %d, ant:%d, cte:%d, duration:%d, len:%d\r\n", report.status,
					report.channel, report.rssi, report.rssi_antenna_id, report.cte_type, report.slot_durations,
					report.samples.len);
			for (int i=0; i<report.samples.len; i++) {
				RETARGET_WriteChar(report.samples.data[i]);
			}
			printf("\r\n");
		  break;
	  }
	  case gecko_evt_cte_receiver_connection_iq_report_id:{
		  printf("GOT CONNECTION IQ report\r\n");
		 struct gecko_msg_cte_receiver_connection_iq_report_evt_t report = evt->data.evt_cte_receiver_connection_iq_report;
		 uint8* data = malloc(report.samples.len);
		 memcpy(report.samples.data, data, report.samples.len);
		 printf("status: %d, ch: %d, rssi: %d, ant:%d, cte:%d, duration:%d, len:%d\r\n", report.status,
				report.channel, report.rssi, report.rssi_antenna_id, report.cte_type, report.slot_durations,
				report.samples.len);
		for (int i=0; i<report.samples.len; i++) {
			RETARGET_WriteChar(report.samples.data[i]);
		}
		printf("\r\n");



		break;
	  }
	  default:
		break;
	}
  }
}

/* Print stack version and local Bluetooth address as boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{
#if DEBUG_LEVEL
  bd_addr local_addr;
  int i;

  printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor, bootevt->patch);
  local_addr = gecko_cmd_system_get_bt_address()->address;

  printLog("local BT device address: ");
  for (i = 0; i < 5; i++) {
    printLog("%2.2x:", local_addr.addr[5 - i]);
  }
  printLog("%2.2x\r\n", local_addr.addr[0]);
#endif
}
