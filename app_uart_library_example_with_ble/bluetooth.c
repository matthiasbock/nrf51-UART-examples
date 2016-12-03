
#include "bluetooth.h"

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */

static bool ble_buffer_available = true;
static bool tx_complete = false;

extern void on_ble_connected();
extern void on_ble_disconnected();


/**@brief     Error handler function, which is called when an error has occurred.
 *
 * @warning   This handler is an example only and does not fit a final product. You need to analyze
 *            how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    //ble_debug_assert_handler(error_code, line_num, p_file_name);
    //printf("Error #%d. Restarting.\n", error_code);

    printf("Error %#x in %s, line %u\nRestarting...\n", (unsigned int) error_code, (char*) p_file_name, (unsigned int) line_num);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}


/**@brief       Assert macro callback function.
 *
 * @details     This function will be called in case of an assert in the SoftDevice.
 *
 * @warning     This handler is an example only and does not fit a final product. You need to
 *              analyze how your product is supposed to react in case of Assert.
 * @warning     On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief   Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(PIN_LED_ADVERTISING);
    // active low
    nrf_gpio_pin_set(PIN_LED_ADVERTISING);

    nrf_gpio_cfg_output(PIN_LED_CONNECTED);
    // active high
    nrf_gpio_pin_clear(PIN_LED_CONNECTED);
}

/**@brief   Function for Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
}


/**@brief   Function for the GAP initialization.
 *
 * @details This function will setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief   Function for the Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    /*
     * Randomize MAC address
     */
    ble_gap_addr_t gap_address;
    err_code = sd_ble_gap_address_get(&gap_address);
    printf("Dis' your MAC: %x:%x:%x:%x:%x:%x\n",
          gap_address.addr[5],
          gap_address.addr[4],
          gap_address.addr[3],
          gap_address.addr[2],
          gap_address.addr[1],
          gap_address.addr[0]
    );
    gap_address.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
    gap_address.addr[5] = 0x42;
    sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &gap_address);
//    gap_address.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE;
//    sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_AUTO, &gap_address);
    printf("MAC address, I choose you: %x:%x:%x:%x:%x:%x\n",
          gap_address.addr[5],
          gap_address.addr[4],
          gap_address.addr[3],
          gap_address.addr[2],
          gap_address.addr[1],
          gap_address.addr[0]
    );

#ifdef NORDIC_UUID_STYLE
    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};
#else
    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_BLE}};
#endif

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
//    advdata.include_appearance      = false;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}


bool ble_attempt_to_send(uint8_t * data, uint8_t length)
{
    uint32_t err_code;

    err_code = ble_nus_send_string(&m_nus, data, length);

    if(err_code == BLE_ERROR_NO_TX_BUFFERS)
    {
        /* ble tx buffer full*/
        return false;
    }
    else if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }

    return true;
}

/**
 * Notify the connected device using the provided message
 * if the device subscribed the RX characteristic
 */
uint32_t ble_notify(ble_nus_t* p_nus, uint8_t* data, uint16_t length)
{

    if (p_nus == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if ((p_nus->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_nus->is_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    // notification / indication
    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_nus->rx_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.p_data = data;
    hvx_params.p_len  = &length;

    return sd_ble_gatts_hvx(p_nus->conn_handle, &hvx_params);
}

extern neopixel_strip_t strip[];

/**@brief    Function for handling the data from the Nordic UART Service.
 *
 * @details  This function will process the data received from the Nordic UART BLE Service and send
 *           it to the UART module.
 */
/**@snippet [Handling the data received over BLE] */
void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    //ble_attempt_to_send(p_data, length);
//    neopixel_set_color_and_show(&strip[0], 2, 0xFF, 0xFF, 0xFF);
//    return;
/*
    if (length < 5)
        return;

    if (p_data[0] > 0)
        return;

    if (p_data[1] > 4)
        return;
*/
    //nrf_gpio_pin_toggle(PIN_LED_ACTIVITY);
    neopixel_set_color_and_show(&strip[p_data[0]], p_data[1], p_data[2], p_data[3], p_data[4]);
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void init_services(void)
{
    uint32_t         err_code;
    ble_nus_init_t   nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing security parameters.
 */
static void init_security_parameters(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief       Function for handling an event from the Connection Parameters Module.
 *
 * @details     This function will be called for all events in the Connection Parameters Module
 *              which are passed to the application.
 *
 * @note        All this function does is to disconnect. This could have been done by simply setting
 *              the disconnect_on_fail config parameter, but instead we use the event handler
 *              mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_connection_parameters_event(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief       Function for handling errors from the Connection Parameters module.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void init_connection_parameters(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_connection_parameters_event;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_set(PIN_LED_ADVERTISING);
}


/**@brief       Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;

    switch (p_ble_evt->header.evt_id)    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_clear(PIN_LED_ADVERTISING);
            nrf_gpio_pin_set(PIN_LED_CONNECTED);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            //on_ble_connected();

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(PIN_LED_CONNECTED);

            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            //on_ble_disconnected();

            advertising_start();

            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
                nrf_gpio_pin_clear(PIN_LED_CONNECTED);

                // Go to system-off mode (this function will not return; wakeup will cause a reset)
                //err_code = sd_power_system_off();
                printf("Timeout.\n");
                NVIC_SystemReset();
                APP_ERROR_CHECK(err_code);
            }
            break;
        case BLE_EVT_TX_COMPLETE:
            if(!ble_buffer_available) tx_complete = true;
            break;

        default:
            //printf("unimplemented event");
            // No implementation needed.
            break;
    }
}


/**@brief       Function for dispatching a S110 SoftDevice event to all modules with a S110
 *              SoftDevice event handler.
 *
 * @details     This function is called from the S110 SoftDevice event interrupt handler after a
 *              S110 SoftDevice event has been received.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief   Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


void ble_init()
{
    // Initialize
    leds_init();
    timers_init();
//    buttons_init();
//    uart_init();
    ble_stack_init();
    gap_params_init();
    init_services();
    advertising_init();
    init_connection_parameters();
    init_security_parameters();

    printf("init complete.\n");

    advertising_start();
}
