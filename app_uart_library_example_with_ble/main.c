
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
//#include "app_uart.h"
//#include "uart_conf.h"
#include "ble_error_log.h"
#include "ble_debug_assert_handler.h"
#include "app_util_platform.h"

#include "ble_uart.h"
#include "bluetooth.h"

#include "board.h"
#include "led.h"

// LED strips
#define NUM_STRIPS      1
#define TOTAL_NUM_LEDS  5
neopixel_strip_t strip[NUM_STRIPS];
const uint8_t strip_at_pin[NUM_STRIPS]   = {30}; //, 28, 2, 0};
const uint8_t leds_per_strip[NUM_STRIPS] = {5};
volatile bool strip_changed[NUM_STRIPS]  = {false}; //, false, false, false};

// get the number of the strip from LED index
const uint8_t led_to_strip[TOTAL_NUM_LEDS]          = {1, 1, 1, 1, 1};
const uint8_t led_index_on_strip[TOTAL_NUM_LEDS]    = {0, 1, 2, 3, 4};


uint8_t led_memory[TOTAL_NUM_LEDS * 3];

/**
 * @brief Initializes all LED strips
 */
void init_ledstrips()
{
    strip[0].leds = led_memory;

    for (int strip_num=0; strip_num<NUM_STRIPS; strip_num++)
    {
        neopixel_init(&strip[strip_num], strip_at_pin[strip_num], leds_per_strip[strip_num]);
        // unnecessary, filled with zeroes by startup script anyways
        //neopixel_clear(&strip[strip_num]);
        neopixel_show(&strip[strip_num]);
    }
}

/**@brief  Application main function.
 */
int main(void)
{
    printf("Hello world\n");
    nrf_gpio_cfg_output(PIN_LED_ADVERTISING);
    nrf_gpio_cfg_output(PIN_LED_CONNECTED);
    nrf_gpio_cfg_output(PIN_LED_ACTIVITY);

    init_ledstrips();

    ble_init();

    while (true)
    {
        asm("wfi");
    }
/*
    for (;;)
    {
//        asm("wfi");
        nrf_delay_ms(1000);
        char* s = "Interoberlin!";
        ble_attempt_to_send(s, strlen(s));
    }
*/
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
