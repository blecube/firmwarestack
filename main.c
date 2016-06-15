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
 * @defgroup main.c
 * @{
 * @brief    BLE-cube application main file.
 *
 * This file contains the source code for an application that uses the Nordic UART service and the Nordic timeslot API for muliactivity between peripheral and adverticer modes.
 * This application uses Eddystone-URL. 
 */

#include <stdint.h>
#include <string.h>
#include "advertiser_beacon.h"
#include "app_button.h"
#include "app_timer.h"
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


#define ABT_DEBUG_PRINT

#ifdef ABT_DEBUG_PRINT
    #include "SEGGER_RTT.h"
    #define DEBUG_PRINTF SEGGER_RTT_printf
#else
    #define DEBUG_PRINTF(...)
#endif

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "BLE-Cube"                               /**< Name of device. Will be included in the advertising data. */
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
#define SAMPLES_IN_BUFFER               3       
#define ADC_COMPARE_RATE                200     //Compare ADC event every N milisec

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
static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(1);

static nrf_ppi_channel_t       m_ppi_channel;
//////////////////////////

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

static nrf_saadc_value_t                m_buffer_pool[2][SAMPLES_IN_BUFFER];
static uint8_t                          cube_shaking = 0;

/* Eddystone stasj */
static uint8_t eddystone_url_data[] =   //< Information advertised by the Eddystone URL frame type. 
{
    //APP_EDDYSTONE_URL_FRAME_TYPE,   // Eddystone URL frame type.
    //APP_EDDYSTONE_RSSI,             // RSSI value at 0 m. 
    APP_EDDYSTONE_URL_SCHEME,       // Scheme or prefix for URL ("http", "http://www", etc.)
    APP_EDDYSTONE_URL_URL           // URL with a maximum length of 17 bytes. Last byte is suffix (".com", ".org", etc.)
};

static uint8_t movement_counter;
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
        2 = Rett løsning
        3 = Initiate Lightshow
*/


//Define holders for times a button is pressed
uint8_t btn1_counter = 0;
uint8_t btn2_counter = 0;
uint8_t btn3_counter = 0;
uint8_t btn4_counter = 0;
uint8_t btn5_counter = 0;
uint8_t btn6_counter = 0; 

//Declare the available colors
uint8_t ledMatriseA[] = {whiteA, redA, greenA, redA+greenA, blueA+greenA, blueA+redA, blueA};
uint8_t ledMatriseB[] = {whiteB, redB, greenB, redB+greenB, blueB+greenB, blueB+redB, blueB}; 

bool knappSjekk = true;
bool knappSjekk1 = true;
bool knappSjekk2 = true;
bool knappSjekk3 = true;
bool knappSjekk4 = true;
bool knappSjekk5 = true;
bool knappSjekk6 = true;

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
static void  nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    
    for (uint32_t i = 0; i < 8; i++)
    {
        dataReceived[i] = p_data[i];
        
        //Netsiden har gitt en ny oppgave, svaret var rett, og knapper resettes
		if (dataReceived[7] == 0x02) { //0x02 betyr at løsning er sjekket, og løsningen var rett. 
			btn1_counter = btn2_counter = btn3_counter = btn4_counter = btn5_counter = btn6_counter = 0;
            btn1_counter = solution[0] = dataReceived[0];
            solution[7] = 3;
            solution[6] = 0;
            cube_shaking = 0;
		}
        else if (dataReceived[7] == 0x01){//0x01 betyr at løsning er sjekket, og løsningen var feil. 
            nrf_gpio_pin_set(RUMBLEPIN);
            nrf_delay_ms(100);
            nrf_gpio_pin_clear(RUMBLEPIN);
            solution[7] = 0;
            solution[6] = 0;
            cube_shaking = 0;
        }
        else if (dataReceived[7] == 0x03){//0x03 Betyr at vi vil se et lysshow!.
            solution[7] = 3;
            cube_shaking = 0;         
        }
        //Man har motatt data for nytt mønster fra webside, 
		if (dataReceived[6] == 0x01) {
            btn1_counter = btn2_counter = btn3_counter = btn4_counter = btn5_counter = btn6_counter = 0;
			btn1_counter = solution[0] = dataReceived[0];
            solution[7] = 0;
            solution[6] = 0;
		}
    }
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
//This function is not used. 
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
    SEGGER_RTT_printf(0, "  Setting clck\r\n");
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    SEGGER_RTT_printf(0, "  clck set\r\n");
    // Initialize SoftDevice.
    SEGGER_RTT_printf(0, "  SOFTDEVICE_HANDLER_INIT\r\n");
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    SEGGER_RTT_printf(0, "  INITIALIZED\r\n");
    
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
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

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
//Need to make the program fully eventbased before adding this. 
/*static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}*/


/////////////////////////////////////////////////////////////////////////
/**             Funktions for SAADC and timer                          */
/////////////////////////////////////////////////////////////////////////
void timer_light_show_handler(nrf_timer_event_t event_type, void* p_context){
    switch(event_type)
    {
        case NRF_TIMER_EVENT_COMPARE1:
            NRF_TIMER1->EVENTS_COMPARE[1] = 0;
            NRF_TIMER1->TASKS_CLEAR = 1;
            static int i;
            i++;   
            SEGGER_RTT_printf(0, "Time Event! nr: %d\r\n",i);
            if(solution[7] == 0x03){
                if(i > 44){
                    i = 0;
                }
                else if(i > 34){
                    if(btn1_counter == 3){
                        btn1_counter = btn2_counter = btn3_counter = btn4_counter = btn5_counter = btn6_counter = 1;
                    }
                    else{
                        btn1_counter = btn2_counter = btn3_counter = btn4_counter = btn5_counter = btn6_counter = 3;
                    }
                    shiftRegWrite(ledMatriseA[btn1_counter]+ledMatriseB[btn2_counter], ledMatriseA[btn3_counter]+ledMatriseB[btn4_counter], ledMatriseA[btn5_counter]+ledMatriseB[btn6_counter]);
                }
                else if(i > 24){
                    if(btn1_counter == 0){
                        btn1_counter = btn2_counter = btn3_counter = btn4_counter = btn5_counter = btn6_counter = 4;
                    }
                    else{
                        btn1_counter = btn2_counter = btn3_counter = btn4_counter = btn5_counter = btn6_counter = 0;
                    }
                    shiftRegWrite(ledMatriseA[btn1_counter]+ledMatriseB[btn2_counter], ledMatriseA[btn3_counter]+ledMatriseB[btn4_counter], ledMatriseA[btn5_counter]+ledMatriseB[btn6_counter]);
                }
                else if(i & 1){
                    if(btn1_counter == btnCounter(btn1, btn6_counter)){
                        btn1_counter = btnCounter(btn1, btn1_counter);
                    }
                    else{
                        btn1_counter = btnCounter(btn1, btn6_counter);
                    }
                    btn2_counter = btnCounter(btn1, btn1_counter);
                    btn3_counter = btnCounter(btn1, btn2_counter);
                    btn4_counter = btnCounter(btn1, btn3_counter);
                    btn5_counter = btnCounter(btn1, btn4_counter);
                    btn6_counter = btnCounter(btn1, btn5_counter);
                    shiftRegWrite(ledMatriseA[btn1_counter]+ledMatriseB[btn2_counter], ledMatriseA[btn3_counter]+ledMatriseB[btn4_counter], ledMatriseA[btn5_counter]+ledMatriseB[btn6_counter]);
                }
            }       
            break;
        
        default:
            //Do nothing.
            break;
    } 
}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;  
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
 
    err_code = nrf_drv_timer_init(&m_timer, NULL, timer_light_show_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 100ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, ADC_COMPARE_RATE);
    
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL1, ticks, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, true);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL1);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get(); //NRF_SAADC_TASK_SAMPLE som argument?

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);
}
/**@brief Will Enable ADC sampling event
*/
void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}
/**@brief Will fetch ADC result via ADC interrupt
*/
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event){
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE){
        static nrf_saadc_value_t  adc_result_last;
        int adc_xyz_result = 0;

        uint32_t err_code;
        int i;
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            adc_xyz_result += p_event->data.done.p_buffer[i];            
        }
        if(adc_result_last == 0){
            adc_result_last = adc_xyz_result;
        }
        if((adc_xyz_result > adc_result_last+30)||(adc_xyz_result < adc_result_last-30)){
            movement_counter+=1;
            if(movement_counter>3){
                cube_shaking = 1;
                SEGGER_RTT_printf(0, "    Ristes \r\n");
                movement_counter = 0;
            }
        }
        else{
            if(movement_counter != 0){
                movement_counter--;
            }
        }
        SEGGER_RTT_printf(0, "Time Event!%d\r\n",adc_xyz_result);
        adc_result_last = adc_xyz_result;
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);


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
    
    //Setup SAADC channel(s)
    nrf_saadc_channel_config_t channel_x_config =
            NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);
    channel_x_config.acq_time = NRF_SAADC_ACQTIME_10US;
    channel_x_config.gain = NRF_SAADC_GAIN1_4;
    channel_x_config.reference = NRF_SAADC_REFERENCE_VDD4;
    
    nrf_saadc_channel_config_t channel_y_config =
            NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
    channel_y_config.acq_time = NRF_SAADC_ACQTIME_10US;
    channel_y_config.gain = NRF_SAADC_GAIN1_4;
    channel_y_config.reference = NRF_SAADC_REFERENCE_VDD4;
    
    nrf_saadc_channel_config_t channel_z_config =
            NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);
    channel_z_config.acq_time = NRF_SAADC_ACQTIME_10US;
    channel_z_config.gain = NRF_SAADC_GAIN1_4;
    channel_z_config.reference = NRF_SAADC_REFERENCE_VDD4;
    
    err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    //Initialize SAADC channel(s)
    err_code = nrf_drv_saadc_channel_init(0, &channel_x_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(1, &channel_y_config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(2, &channel_z_config);
    APP_ERROR_CHECK(err_code);


    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}



/////////////////////////////////////////////////////////////////////////
/*                                                                     */
/////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////
//                      Timeslot - Eddystone-URL                    //
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
//////////////////////////////////////////////////////////////////////
//                                                                  //
//////////////////////////////////////////////////////////////////////

/**@brief Application main function.
 */
int main(void)
{
    
       
    uint32_t err_code;
    nrf_gpio_cfg_output(RUMBLEPIN);
    nrf_gpio_pin_clear(RUMBLEPIN);
    SEGGER_RTT_printf(0, "Rumble set\r\n");
    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    
    SEGGER_RTT_printf(0, "No Eddy\r\n");
    bool erase_bonds;
     
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    //SAADC - Analogue accellerometer
    saadc_sampling_event_init();
    saadc_init();
    saadc_sampling_event_enable();    
    //uint8_t  start_string[] = START_STRING;
    //SEGGER_RTT_printf("%s",start_string);

    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    timeslot_init();
    shiftRegInit();
    shiftRegWrite(whiteA + whiteB,whiteA + whiteB,whiteA + whiteB);

    // Enter main loop.
    SEGGER_RTT_printf(0, "We're entering tha looop! AMMAGAD!\r\n");
    for (;;)
    {
        //Checking if the cube has been shaken, then sends solution to the webpage. 
        if(cube_shaking) {
            SEGGER_RTT_printf(0, "Cube has bin shoooocked !\r\n");
            solution[6] = 2;
            ble_nus_string_send(&m_nus, solution, sizeof(solution));
        }
        if(nrf_gpio_pin_read(btn1) && knappSjekk1) {
            btn1_counter = btnCounter(btn1, btn1_counter);
            solution[0] = btn1_counter;
            knappSjekk1 = false;
        }
        else if(!(nrf_gpio_pin_read(btn1))) {
            knappSjekk1 = true;
        }
        if(nrf_gpio_pin_read(btn2) && knappSjekk2) {
            btn2_counter = btnCounter(btn2, btn2_counter);
            solution[1] = btn2_counter;
            knappSjekk2 = false;
        }
        else if(!(nrf_gpio_pin_read(btn2))) {
            knappSjekk2 = true;
        }
        if(nrf_gpio_pin_read(btn3) && knappSjekk3) {
            btn3_counter = btnCounter(btn3, btn3_counter);
            solution[2] = btn3_counter;
            knappSjekk3 = false;
        }
        else if(!(nrf_gpio_pin_read(btn3))) {
            knappSjekk3 = true;
        }
        if(nrf_gpio_pin_read(btn4) && knappSjekk4) {
            btn4_counter = btnCounter(btn4, btn4_counter);
            solution[3] = btn4_counter;
            knappSjekk4 = false;
        }
        else if(!(nrf_gpio_pin_read(btn4))) {
            knappSjekk4 = true;
        }
        if(nrf_gpio_pin_read(btn5) && knappSjekk5) {
            btn5_counter = btnCounter(btn5, btn5_counter);
            solution[4] = btn5_counter;
            knappSjekk5 = false;
        }
        else if(!(nrf_gpio_pin_read(btn5))) {
            knappSjekk5 = true;
        }
        if(nrf_gpio_pin_read(btn6) && knappSjekk6) {
            btn6_counter = btnCounter(btn6, btn6_counter);
            solution[5] = btn6_counter;
            knappSjekk6 = false;
        }
        else if(!(nrf_gpio_pin_read(btn6))) {
            knappSjekk6 = true;
        }
        shiftRegWrite(ledMatriseA[btn1_counter]+ledMatriseB[btn2_counter], ledMatriseA[btn3_counter]+ledMatriseB[btn4_counter], ledMatriseA[btn5_counter]+ledMatriseB[btn6_counter]);

    

        //power_manage();
    }
}


/** 
 * @}
 */
