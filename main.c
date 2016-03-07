/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.nrf_drv_config.h
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_ppi.h"
#include "nrf_soc.h"
#include "nrf_drv_config.h"
#include "app_error.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "ble_sensor_data_custom.h"
#include "nrf_drv_gpiote.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "LettPark"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                800                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 500 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      20                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_sdc_t                        m_sdc;                                      /**< Structure to identify the Send Data Custom service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
static nrf_saadc_value_t                adc_buf;                                    /**< Data buffer saadc. */
static const nrf_drv_timer_t            m_timer = NRF_DRV_TIMER_INSTANCE(1);        /**< Timer Instance to Timer 1. */
static const nrf_drv_timer_t            m_timer_time = NRF_DRV_TIMER_INSTANCE(2);
static nrf_ppi_channel_t                m_ppi_channel;                              /**< Structure to identify the ppi channel setup. */
static bool                             connected = false;                          /**< Bool to indicate if device is connected or not*/

/* Forward decleration of enable/disable saadc trough ppi functions. */
void saadc_sampling_event_enable(void);                                     
void saadc_sampling_event_disable(void);


/**@brief Function for assert macro callback.
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

/* Data handler dummy for initializing the Send Data Custom service in services_init() */
static void sdc_data_handler(ble_sdc_t * p_sdc, uint8_t * p_data, uint16_t length)
{
  
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_sdc_init_t sdc_init;
    
    memset(&sdc_init, 0, sizeof(sdc_init));

    sdc_init.data_handler = sdc_data_handler;
    
    err_code = ble_sdc_init(&m_sdc, &sdc_init);
    APP_ERROR_CHECK(err_code);
}


/* Handler for Send Data Custom Service on BLE events. */
void ble_sdc_on_ble_evt(ble_sdc_t * p_sdc, ble_evt_t * p_ble_evt)
{
    if ((p_sdc == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sdc, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sdc, p_ble_evt);
            saadc_sampling_event_disable();
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_sdc, p_ble_evt);
            saadc_sampling_event_enable();
            break;

        default:
            // No implementation needed.
            break;
    }
}

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
    uint32_t err_code;
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
        case BLE_ADV_EVT_SLOW:  // Advertising event slow.
            NRF_GPIO->OUT = (0<<LED_3);
            
            break;
        case BLE_ADV_EVT_IDLE: // When advertising times out.
            //sleep_mode_enter();
            NRF_GPIO->OUT = (1<<LED_3);
            if (connected != true)
            {
                err_code = ble_advertising_start(BLE_ADV_MODE_SLOW); // Restart advertising.
                APP_ERROR_CHECK(err_code);
            }
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
            connected = true;
            //err_code = ble_advertising_start(BLE_ADV_MODE_IDLE);
            APP_ERROR_CHECK(err_code);
            nrf_gpio_pin_toggle(LED_1);
            //NRF_GPIO->OUT = (0<<3);               // Pin for enabling power to sensor.
            
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            connected = false;
            //err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);
            APP_ERROR_CHECK(err_code);
            nrf_gpio_pin_toggle(LED_1);
            //NRF_GPIO->OUT = (1<<3);               // Pin for disabling power to sensor.
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
    on_ble_evt(p_ble_evt);
    ble_sdc_on_ble_evt(&m_sdc, p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);
    
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
}




/* Dummy for UART events. */ 
void uart_event_handle(app_uart_evt_t * p_event)
{
}
/* Initialize UART. */
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
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
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
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_DISABLED;  // FAST ADVERTISING disabled.
    options.ble_adv_slow_enabled  = BLE_ADV_SLOW_ENABLED;   // ENABLE SLOW ADVERTISING
    options.ble_adv_slow_interval = APP_ADV_INTERVAL;
    options.ble_adv_slow_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}



/* Dummy for handling Timer events. */
void timer_handler(nrf_timer_event_t event_type, void* p_context)
{
        /*uint32_t captured_value = nrf_drv_timer_capture(&m_timer, NRF_TIMER_CC_CHANNEL0);
        nrf_drv_timer_clear(&m_timer);
        uint8_t calculated_value = captured_value/1000;
        if (calculated_value <= 200) 
            {
                uint8_t data_to_send[1] = {calculated_value};
                uint32_t err_code = ble_sdc_data_send(&m_sdc, data_to_send, 1);
                APP_ERROR_CHECK(err_code);
            }
    
        nrf_drv_timer_resume(&m_timer);*/
}
/* Timer and ppi initializing function. */
void saadc_sampling_event_init(void)
{
    
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_timer_config_t config_timer = { 
        .frequency          = NRF_TIMER_FREQ_1MHz,
        .mode               = NRF_TIMER_MODE_TIMER,
        .bit_width          = NRF_TIMER_BIT_WIDTH_24,
        .interrupt_priority = NRF_APP_PRIORITY_LOW,
        .p_context          = NULL  
    };
    
    nrf_drv_timer_config_t config = { 
        .frequency          = NRF_TIMER_FREQ_1MHz,
        .mode               = NRF_TIMER_MODE_TIMER,
        .bit_width          = NRF_TIMER_BIT_WIDTH_24,
        .interrupt_priority = NRF_APP_PRIORITY_LOW,
        .p_context          = NULL  
    };
    
    err_code = nrf_drv_timer_init(&m_timer_time, &config_timer, timer_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_timer_init(&m_timer, &config, timer_handler);
    APP_ERROR_CHECK(err_code);
    
     /*setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 30);
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrf_drv_timer_enable(&m_timer);
    
    uint32_t ppi_event_addr =  nrf_drv_gpiote_in_event_addr_get(BSP_BUTTON_0);
    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t ppi_task_addr =  	nrf_drv_timer_task_address_get(&m_timer,NRF_TIMER_TASK_CAPTURE1);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_task_address_get(NRF_SAADC_TASK_SAMPLE);

     /*setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);
    
}

/* Enable saadc with ppi. */
void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

/* Disable saadc with ppi. */
void saadc_sampling_event_disable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_disable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

/* Handler for saadc events. */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
        uint32_t err_code;
        uint8_t index = 1;
        uint8_t data_to_send[1];
        
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer,1);
        APP_ERROR_CHECK(err_code);
        uint8_t value = p_event->data.done.p_buffer[0];
        if (value >= 240) {
            nrf_drv_timer_capture(&m_timer_time, NRF_TIMER_CC_CHANNEL1);
            nrf_drv_timer_disable(&m_timer_time);
            uint32_t time =  nrf_drv_timer_capture_get(&m_timer_time, NRF_TIMER_CC_CHANNEL1)/1000;
            nrf_drv_timer_enable(&m_timer_time);
            if (time >= 255) {
                data_to_send[0] = time;
                err_code = ble_sdc_data_send(&m_sdc, data_to_send, index);
                APP_ERROR_CHECK(err_code);
                nrf_gpio_pin_toggle(LED_2);
            }
        }
        printf("%d\r\n", p_event->data.done.p_buffer[0]);
    }
}

/* Configuring function for saadc. */
static void saadc_configure(void)
{   
    nrf_saadc_channel_config_t config = {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,         
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,         
        .gain       = NRF_SAADC_GAIN1_6,                   
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,        
        .acq_time   = NRF_SAADC_ACQTIME_40US,              
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,         
        .pin_p      = NRF_SAADC_INPUT_AIN0,                 // P0.02 Inngang SAADC
        .pin_n      = NRF_SAADC_INPUT_DISABLED
    };
    
    nrf_drv_saadc_config_t config_init = {
        .resolution = NRF_SAADC_RESOLUTION_8BIT,
        .oversample = NRF_SAADC_OVERSAMPLE_DISABLED,
        .interrupt_priority = NRF_APP_PRIORITY_LOW 
    };
    ret_code_t err_code = nrf_drv_saadc_init(&config_init, saadc_event_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_saadc_channel_init(0,&config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf,1);
    APP_ERROR_CHECK(err_code);

}

void pin_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{   /*uint8_t data_to_send[1];
    uint32_t err_code;
    if (action == NRF_GPIOTE_POLARITY_LOTOHI)
    {   
        nrf_drv_timer_capture(&m_timer, NRF_TIMER_CC_CHANNEL1);
        nrf_drv_timer_disable(&m_timer);
        uint32_t value =  nrf_drv_timer_capture_get(&m_timer, NRF_TIMER_CC_CHANNEL1)/1000;
        nrf_drv_timer_enable(&m_timer);
        printf("%d\r\n", value);
        if (30 <= value <= 255) {
            
                data_to_send[0] = value;
                err_code = ble_sdc_data_send(&m_sdc, data_to_send, 1);
                APP_ERROR_CHECK(err_code);
                printf("Verdien er %d\r\n", value);
            
        
        }
    //APP_ERROR_CHECK(err_code);
    }*/
}

static void gpiote_and_timer_init(void)
{
    nrf_drv_gpiote_init();
    nrf_drv_gpiote_in_config_t config = 
    {                                            
        .is_watcher = false,                     
        .hi_accuracy = false,                  
        .pull = NRF_GPIO_PIN_NOPULL,             
        .sense = NRF_GPIOTE_POLARITY_LOTOHI,
    };
    nrf_drv_gpiote_in_init(4, &config, pin_int_handler);
    nrf_drv_gpiote_in_event_enable(4, true);
    
    
    
}


int main(void)
{
    uint32_t err_code;
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false); 
    uart_init();
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    saadc_configure();
    saadc_sampling_event_init();
    //gpiote_and_timer_init();
    
    NRF_GPIO->DIR = (1<<LED_1) | (1<<LED_2) | (1<<LED_3);
    NRF_GPIO->OUT = (1<<LED_1) | (1<<LED_2) | (1<<LED_3);
    
    err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);
    APP_ERROR_CHECK(err_code);
    
    for (;;)
    {
        power_manage();
        
    }
}



