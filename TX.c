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
#include "app.h"

/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;

static uint8 	   connection;
static uint8_t     connecting = 0;

bool compare_bd_addr(bd_addr *address1, bd_addr *address2){
	for(int i = 0;i<5;i++){
		if(address1->addr[i] != address2->addr[5-i]){
			return false;
		}
		else{
		}
	}
	return true;
}
bd_addr test_device = {
		.addr = {0x58,0x8E,0x81,0xA5,0x53,0xB0}
};

/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

  /* Set maximum number of periodic advertisers */
  pconfig->bluetooth.max_advertisers = 1;

  /* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
  initLog();


  /* Initialize stack */
  gecko_init(pconfig);

  //Initialize CTE
  gecko_bgapi_class_cte_transmitter_init();

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

      /* Handle events */
      switch (BGLIB_MSG_ID(evt->header)) {

      uint16 result;

        /* This boot event is generated when the system boots up after reset.
         * Do not call any stack commands before receiving the boot event.
         * Here the system is set to start advertising immediately after boot procedure. */
        case gecko_evt_system_boot_id:

          bootMessage(&(evt->data.evt_system_boot));
      	  gecko_cmd_system_set_tx_power(100);
      	  gecko_cmd_le_gap_set_advertise_tx_power(0,30);
		  gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);


		  //Start scanning
		  gecko_cmd_le_gap_set_discovery_timing(le_gap_phy_1m, 20, 10);
		  gecko_cmd_le_gap_start_discovery(le_gap_phy_1m, le_gap_discover_generic);
		  printLog("\r\nScanning\r\n");

		  // Start extended advertising
		  result = gecko_cmd_le_gap_start_advertising(0,le_gap_general_discoverable, le_gap_connectable_scannable)->result;
		  printf("le_gap_start_advertising() returns 0x%X\r\n", result);


		  break;

        case gecko_evt_le_gap_scan_response_id: {
        	bd_addr *remote_address = &evt->data.evt_le_gap_scan_response.address;


        	  if (connecting == 0 && compare_bd_addr(&test_device,&evt->data.evt_le_gap_scan_response.address)){

				  printLog("\r\nDevice found\r\n");
				  printLog("Initiating connection\r\n");

    			  printf("advertisement/scan response from %2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X\r\n", remote_address->addr[5],remote_address->addr[4], remote_address->addr[3], remote_address->addr[2], remote_address->addr[1],  remote_address->addr[0]);

				  /* We found our device so let's connect to it */
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

        case gecko_evt_le_connection_opened_id:
        	printf("We got a CONNECTION\r\n");
        	connecting = 0;
        	gecko_cmd_le_gap_end_procedure();
        	uint8 cte_type = 0x5;  // AoA
        	uint8 s_len = 1;
        	uint8 sa[1] = { 0 };

        	uint16 tx_result = gecko_cmd_cte_transmitter_enable_connection_cte(connection, cte_type, s_len, sa)->result;
        	printf("Result of tx_enable_connection_cte: 0x%x\r\n", tx_result);
        	if (tx_result == 0x0)
        		gecko_cmd_le_gap_end_procedure();
        	break;

        case gecko_evt_le_connection_closed_id:

          /* Check if need to boot to dfu mode */
          if (boot_to_dfu) {
            /* Enter to DFU OTA mode */
            gecko_cmd_system_reset(2);
          } else {
            /* Restart advertising after client has disconnected */
            gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
          }
          break;

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



