/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
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
#include "advertiser_beacon.h"
#include "app_button.h"
#include "app_timer.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "ble_nus.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h" //test
#include "nrf_drv_timer.h" //test
#include "nrf_gpio.h"
#include "softdevice_handler.h"
#include "shiftReg.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "BLE Cube"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64  //(MSEC_TO_UNITS(100, UNIT_0_625_MS))                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define START_STRING                    "Connected to BLE-Cube..\n"                                /**< The string that will be sent over the UART when the application starts. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/*  Buffer sizes for the RX and TX over the Ble uart    */
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

/*  Constants for the SAADC     */
#define SAMPLES_IN_BUFFER               5       
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                          /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                            /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  270
#define ADC_RES_10BIT                   1024   
#define ADC_COMPARE_RATE                100     //Compare ADC event every N milisec

// Eddystone non-connectable variables
#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                           /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables the time-out. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)           /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100 ms and 10.24 s). */

// Eddystone common data
#define APP_EDDYSTONE_UUID              0xFEAA                                      /**< UUID for Eddystone beacons according to specification. */
#define APP_EDDYSTONE_RSSI              0xEE                                        /**< 0xEE = -18 dB is the approximate signal strength at 0 m.*/

// Eddystone URL data
#define APP_EDDYSTONE_URL_FRAME_TYPE    0x10                                        /**< URL Frame type is fixed at 0x10. */
#define APP_EDDYSTONE_URL_SCHEME        0x03                                        /**< 0x03 = "https://" URL prefix scheme according to specification. */
#define APP_EDDYSTONE_URL_URL           0x67, 0x6f, 0x6f, 0x2e, \
                                        0x67, 0x6c, 0x2f, 0x35, \
                                        0x70, 0x6b, 0x73, 0x48, \
                                        0x52

////////////////////////// Static time variables for the ADC
static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_ppi_channel_t       m_ppi_channel;
//////////////////////////

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

static nrf_saadc_value_t                m_buffer_pool[2][SAMPLES_IN_BUFFER];
static uint8_t                          cube_shaking = 0;

/* Eddystone stasj */
static ble_gap_adv_params_t m_adv_params;                                 //< Parameters to be passed to the stack when starting advertising. 

static uint8_t eddystone_url_data[] =   //< Information advertised by the Eddystone URL frame type. 
{
    //APP_EDDYSTONE_URL_FRAME_TYPE,   // Eddystone URL frame type.
    //APP_EDDYSTONE_RSSI,             // RSSI value at 0 m.
    APP_EDDYSTONE_URL_SCHEME,       // Scheme or prefix for URL ("http", "http://www", etc.)
    APP_EDDYSTONE_URL_URL           // URL with a maximum length of 17 bytes. Last byte is suffix (".com", ".org", etc.)
};
static bool                         EDDYSTONE = false;
/////////////////////////


/**@brief Counters for buttonpress. 
 *
 * @details variables for checking how many times the different buttons are pressed
 *          also some test arrays for the webpage transmissions. 
 */
uint8_t dataReceived[8] =   {0, 0, 0, 0, 0, 0, 0, 0};
        
uint8_t solution[8]     =   {0, 0, 0, 0, 0, 0, 0, 0};
/**@brief info om soultion flags.      
    solution[6] 
        0 = NULL 
        1 = CUBE_RNG 
        2 = CUBE_SOLVE
    solution[7] 
        0 = NULL
        1 = Feil løsning
        3 = Rett løsning
*/


//Define holders for times a button is pressed
uint8_t btn1_counter = 0;
uint8_t btn2_counter = 0;
uint8_t btn3_counter = 0;
uint8_t btn4_counter = 0;
uint8_t btn5_counter = 0;
uint8_t btn6_counter = 0; 

bool knappSjekk = true;

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
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


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    printf("UART: %d", *p_data);
    for (uint32_t i = 0; i < 8; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
        dataReceived[i] = p_data[i];
        
        //Netsiden har gidd en ny oppgave, svaret var rett, og knapper resettes
		if (dataReceived[7] == 0x02) { //0x03 betyr at løsning er sjekket, og løsningen var rett. 
			btn1_counter = btn2_counter = btn3_counter = btn4_counter = btn5_counter = btn6_counter = 0;
            solution[7] = 0;
            solution[6] = 0;
            cube_shaking = 0;
		}
        else if (dataReceived[7] == 0x01){//0x03 betyr at løsning er sjekket, og løsningen var feil. 
            solution[7] = 0;
            solution[6] = 0;
            cube_shaking = 0;
        }
        //Man har motatt data for nytt mønster fra webside, 
		if (dataReceived[6] == 0x01) {
			btn1_counter = dataReceived[0];
            solution[6] = 0;
			solution[0] = dataReceived[0];
		}
		/* SENDING AV DATA
		ble_nus_string_send(&m_nus, string2send, sizeof(string2send));
		*/
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
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
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice 
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);   
}
/**@brief Function for dispatching system events from the SoftDevice.
*
* @details This function is called from the SoftDevice event interrupt handler after a
*          SoftDevice event has been received.
*
* @param[in] evt_id  System event id.
*/
static void sys_evt_dispatch(uint32_t evt_id)
{
    app_beacon_on_sys_evt(evt_id);
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC; //var ikke i alfaversionen?   
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
        
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    if(!EDDYSTONE){
        err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
        APP_ERROR_CHECK(err_code);
    }
    //Sørg for at softdevice_sys_evt_handler_set(sys_evt_dispatch) blir kallt i ble_stack_init()
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_ENABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;//var BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for placing the application in low power state while waiting for events.
 */
/*static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}*/


/////////////////////////////////////////////////////////////////////////
/**             Funktions for SAADC and timer                          */
/////////////////////////////////////////////////////////////////////////
void timer_handler(nrf_timer_event_t event_type, void* p_context){

}

void saadc_sampling_event_init(void){
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_timer_init(&m_timer, NULL, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 100ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, ADC_COMPARE_RATE);
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);
}
void saadc_sampling_event_enable(void){
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}
/**@brief Will fetch ADC result via ADC interrupt
*/
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event){
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE){
        static nrf_saadc_value_t  adc_result;
        static nrf_saadc_value_t  adc_result_last;
        static uint8_t movement_counter;
        //uint16_t adc_in_milli_volts;
        uint32_t err_code;

        int i;
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            adc_result = p_event->data.done.p_buffer[i];
            if(adc_result_last == 0){
                adc_result_last = adc_result;
            }
            if((adc_result > adc_result_last+20)||(adc_result < adc_result_last-20)){
                movement_counter++;
            }
            else{
                if(movement_counter != 0){
                    movement_counter--;
                }
            }
            if(movement_counter>3){
                cube_shaking = 1;
                movement_counter = 0;
            }
                
            adc_result_last = adc_result;
        }
        
        
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
        
        /*adc_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;*/


        if (
            (err_code != NRF_SUCCESS)
            &&
            (err_code != NRF_ERROR_INVALID_STATE)
            &&
            (err_code != BLE_ERROR_NO_TX_PACKETS)
            &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
           )
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
}

void saadc_init(void){
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
            NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}



/////////////////////////////////////////////////////////////////////////
/*                                                                     */
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
/*              EDDYSTONE SJÆSJ                                        */
/////////////////////////////////////////////////////////////////////////
/**@brief Function for initializing the advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void eddy_advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    ble_uuid_t    adv_uuids[] = {{APP_EDDYSTONE_UUID, BLE_UUID_TYPE_BLE}};

    uint8_array_t eddystone_data_array;                             // Array for Service Data structure.
/** @snippet [Eddystone data array] */
    eddystone_data_array.p_data = (uint8_t *) eddystone_url_data;   // Pointer to the data to advertise.
    eddystone_data_array.size = sizeof(eddystone_url_data);         // Size of the data to advertise.
/** @snippet [Eddystone data array] */

    ble_advdata_service_data_t service_data;                        // Structure to hold Service Data.
    service_data.service_uuid = APP_EDDYSTONE_UUID;                 // Eddystone UUID to allow discoverability on iOS devices.
    service_data.data = eddystone_data_array;                       // Array for service advertisement data.

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.flags                   = flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;
    advdata.p_service_data_array    = &service_data;                // Pointer to Service Data structure.
    advdata.service_data_count      = 1;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                                // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


/**@brief Function for starting advertising.
 */
static void eddy_advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}
//////////////////////////////////////////////////////////////////////
//                                                                  //
//////////////////////////////////////////////////////////////////////

//Declare the available colors
uint8_t ledMatriseA[] = {whiteA, redA, greenA, redA+greenA, blueA+greenA, blueA+redA, blueA};
uint8_t ledMatriseB[] = {whiteB, redB, greenB, redB+greenB, blueB+greenB, blueB+redB, blueB}; 

//////////////////////////////////////////////////////////////////////
//                      Joakim Tips test                            //
//////////////////////////////////////////////////////////////////////
static void beacon_advertiser_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
 
static uint32_t timeslot_init(void)
{
    static ble_beacon_init_t beacon_init;
    beacon_init.adv_interval  = 360;                                // Adv interval in ms
    beacon_init.p_data         = (uint8_t *) eddystone_url_data;    // Pointer to url data array (uint8_t)
    beacon_init.data_size      = sizeof(eddystone_url_data);        // Number of bytes in url data array
    beacon_init.error_handler = beacon_advertiser_error_handler;
 
    uint32_t err_code = sd_ble_gap_address_get(&beacon_init.beacon_addr);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
 
    beacon_init.beacon_addr.addr[0]++;              // Increment address, so that normal adv and eddystone url adv has different addr.
 
    app_beacon_init(&beacon_init);
    app_beacon_start();
 
    return NRF_SUCCESS;
}
 


 
//Sørg for at softdevice_sys_evt_handler_set(sys_evt_dispatch) blir kallt i ble_stack_init()

//////////////////////////////////////////////////////////////////////
//                                                                  //
//////////////////////////////////////////////////////////////////////

/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    
    
    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    
    // Eddystone del.. //
    if(EDDYSTONE){
        err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
        APP_ERROR_CHECK(err_code);
        ble_stack_init();
        eddy_advertising_init();
        //LEDS_ON(LEDS_MASK);
        // Start execution.
        eddy_advertising_start();
    }
    else if(!EDDYSTONE){  
        bool erase_bonds;
       
        uart_init();   
        buttons_leds_init(&erase_bonds);
        ble_stack_init();
        gap_params_init();
        services_init();
        advertising_init();
        conn_params_init();
        //SAADC - Analogue accellerometer
        //saadc_sampling_event_init();
        //saadc_init();
        //saadc_sampling_event_enable();
              
        //uint8_t  start_string[] = START_STRING;
        //SEGGER_RTT_printf("%s",start_string);
    
        err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
        timeslot_init();
        shiftRegInit();
        shiftRegWrite(whiteA + whiteB,whiteA + whiteB,whiteA + whiteB);
    }
    // Enter main loop.
    for (;;)
    {
        if(!EDDYSTONE){
            //Checking if the cube has been shaken, then sends solution to the webpage. 
            if(cube_shaking) {
                solution[6] = 2;
                ble_nus_string_send(&m_nus, solution, sizeof(solution));
            }
            //
            /*if(!(nrf_gpio_pin_read(14)) && knappSjekk) {
                solution[6] = 1;  //If button is clicked, kube asks the webpage for a new task. 
                ble_nus_string_send(&m_nus, solution, sizeof(solution));
                knappSjekk = false;
            }
            if (nrf_gpio_pin_read(14)) {//Funksjon for å hindre at for mye data sendes når knappen trykkes inn.
                solution[6] = 0;
                knappSjekk = true;
            }*/
            if(nrf_gpio_pin_read(btn1)) {
                btn1_counter = btnCounter(btn1, btn1_counter);
                solution[0] = btn1_counter;
            }
            else if(nrf_gpio_pin_read(btn2)) {
                btn2_counter = btnCounter(btn2, btn2_counter);
                solution[1] = btn2_counter;		
            }
            else if(nrf_gpio_pin_read(btn3)) {
                btn3_counter = btnCounter(btn3, btn3_counter);
                solution[2] = btn3_counter;			
            }
            else if(nrf_gpio_pin_read(btn4)) {
                btn4_counter = btnCounter(btn4, btn4_counter);
                solution[3] = btn4_counter;			
            }
            else if(nrf_gpio_pin_read(btn5)) {
                btn5_counter = btnCounter(btn5, btn5_counter);
                solution[4] = btn5_counter;			
            }
            else if(nrf_gpio_pin_read(btn6)) {
                btn6_counter = btnCounter(btn6, btn6_counter);
                solution[5] = btn6_counter;
            }
            shiftRegWrite(ledMatriseA[btn1_counter]+ledMatriseB[btn2_counter], ledMatriseA[btn3_counter]+ledMatriseB[btn4_counter], ledMatriseA[btn5_counter]+ledMatriseB[btn6_counter]);
        }  
		

        //power_manage();
    }
}


/** 
 * @}
 */
