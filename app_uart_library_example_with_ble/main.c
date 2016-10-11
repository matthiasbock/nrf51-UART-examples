/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

/** @file
 * @brief    UART over BLE application using the app_uart library (event driven).
 *
 * This UART example is configured with flow control enabled which is necessary when softdevice
 * is enabled, in order to prevent data loss. To connect the development kit with your PC via 
 * UART, connect the configured RXD, TXD, RTS and CTS pins to the RXD, TXD, RTS and CTS pins 
 * on header P15 on the motherboard. Then connect the RS232 port on the nRFgo motherboard to
 * your PC. Configuration for UART pins is defined in the uart_conf.h header file.
 *
 * This file contains source code for a sample application that uses the Nordic UART service.
 * Connect to the UART example via Master Control Panel and the PCA10000 USB dongle, or via 
 * nRF UART 2.0 app for Android, or nRF UART app for IOS, available on 
 * https://www.nordicsemi.com/Products/nRFready-Demo-APPS.
 *
 * This example should be operated in the same way as the UART example for the evaluation board
 * in the SDK. Follow the same guide for this example, given on:
 * https://devzone.nordicsemi.com/documentation/nrf51/6.0.0/s110/html/a00066.html#project_uart_nus_eval_test
 *
 * This example uses FIFO RX and FIFO TX buffer to operate with the UART. You can set the size
 * for the FIFO buffers by modifying the RX_BUFFER_SIZE and TX_BUFFER_SIZE constants.
 *
 * Documentation for the app_uart library is given in UART driver documentation in the SDK at:
 * https://devzone.nordicsemi.com/documentation/nrf51/6.1.0/s110/html/a00008.html
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "nrf_delay.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_uart.h"
#include "uart_conf.h"
#include "boards.h"
#include "ble_error_log.h"
#include "ble_debug_assert_handler.h"
#include "app_util_platform.h"

#include "ble_uart.h"
#include "bluetooth.h"


/**@brief  Application main function.
 */
int main(void)
{

    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint8_t newbyte;

    printf("Hello world\n");
    nrf_gpio_cfg_output(PIN_UART_ACTIVITY);

    ble_init();

    for (;;)
    {
//        asm("wfi");
        nrf_delay_ms(1000);
        char* s = "Interoberlin!";
        ble_attempt_to_send(s, strlen(s));
    }
/*
    // Enter main loop
    for (;;)
    { 
        // Stop reading new data if there are no ble buffers available
        if(ble_buffer_available)
        {
            if(app_uart_get(&newbyte) == NRF_SUCCESS)
            {
                data_array[index++] =  newbyte;
               
                if (index >= (BLE_NUS_MAX_DATA_LEN))
				{
                    ble_buffer_available=ble_attempt_to_send(&data_array[0],index);
                 	if(ble_buffer_available) index=0;
				}
            }
        }
        // Re-transmission if ble_buffer_available was set to false
        if(tx_complete)
        {
            tx_complete=false;
            
            ble_buffer_available=ble_attempt_to_send(&data_array[0],index);
            if(ble_buffer_available) index =0;
        }

        power_manage();
    }
*/
}

/** 
 * @}
 */
