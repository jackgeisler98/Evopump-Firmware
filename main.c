/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

 /** @details
 * This conversion is based on BLE_UART_WIP7 examples try to upgrade to SDK15.1 and addressing 
 * advertising manufacturer information issue.
 * Features: Four channels of SAADC, Temperature sensor, GPIO current control (will implement 3 eventually)
 * BSP control, LED
 * SETUP: Using counter instead of multiple timers. App_timer prescaler set to the counter frequency of 1024Hz. 
 * timed advertising working. battery level set at 8 bit. Led on for implication.
 * Since single button control is a risk of data loss, add button 3 as advertiser, Buzzer circuit
 * Targeted dosing with sleep mode after delivery
 * Simple BLE service to notify receiver, Two levels of current control
 * TODO: pressure sensor, occlusion sensor, Service for control, secure connection, alert, linkloss, etc
 * Simplest logic for time compensation, with 50 ms pulsation control and 40 ms saadc time scale.
 * Try to implement BLE control with ble-blinky example.
 * Modified based on SDK15.1 template_WIP8
 * Try to add multiplexer with GPIO controls
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "boards.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "bme280_twi.h"
#include "nrfx_saadc.h"
#include "nrf_temp.h"
#include "nrf_gpio.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_drv_clock.h"
#include "delivery_service.h"
#include "sensor_service.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "delivery_control.h"


#define DEVICE_NAME                     "P1"                                    /**< Name of device. Will be included in the advertising data. */
//added from HTS
#define MANUFACTURER_NAME               "CM"                                    /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                       "A1"                                    /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 0x88                                    /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   0x66                                    /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                3200                                    /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION                0                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define ADV_ON                          APP_TIMER_TICKS(30000)                  //define advertising on timer timeout event, 30 s here. Using counter instead of two timers to turn it on and off.

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/** SAADC Macros */
#define SAADC_SAMPLES_IN_BUFFER 4                                               //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1 per channel. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_DISABLED                          //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE 0                                                      //Set to 1 to enable BURST mode, otherwise set to 0.
#define SAADC_INTERVAL		APP_TIMER_TICKS(40)                             /**< SAADC sample rate in ticks. 500 ms to ticks*/    //problem: cannot go down too fast. Will disrupt data saving and other timers
#define SENSOR_INTERVAL         APP_TIMER_TICKS(5000)                           //Update sensor information every 5 s, including basal info
#define SHDN_INTERVAL 50                                                        //define pumping on off frequency, unit ms, here 50 ms on 50 ms off
#define SHDN_PIN 			6         //define GPIO ports for SHDN pin control
#define BUZZER_PIN                      8         //define GPIO port for buzzer control, check every adv_counter time 30 s
#define SIXMA_PIN                       34        //define GPIO port for 6 mA level current P1.02
#define TWOMA_PIN                       35        //define GPIO port for 2 mA level current P1.03
#define MULTIPLEXER_A0                  36        //define GPIO port for ADG708 Multiplexer A0, P1.04
#define MULTIPLEXER_A1                  37        //define GPIO port for ADG708 Multiplexer A1, P1.05
#define MULTIPLEXER_A2                  38        //define GPIO port for ADG708 Multiplexer A2, P1.06
#define START_PUMP                      0         //define the start pump number, from 0 to END_PUMP
#define END_PUMP                     7            //END pump number, total pump number is end-start+1. //current simplified version for same test continuously only

/** Other declaration */
static bsp_indication_t actual_state = BSP_INDICATE_FIRST;                            //Monitor current indication status. Be sure to update to the true status
static int32_t flash_temp;                                                            //temperature sensor
static uint32_t ble_advertising_stop(ble_advertising_t * const p_advertising);        //declaration of function
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);                         //Declaration of fstorage_eve_handler
static void advertising_start(bool erase_bonds);                                      //Declaration of advertising start, maybe not needed
static void saadc_init(void);                                                         //Declaration
static void advertising_init(void);
void application_timers_stop(void);
static volatile bool m_measurement_fetched;
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
static uint16_t bsp_btn0_counter=0;
static uint32_t current_tick;                                                         //Current tick counted, could be overflow and restart
uint32_t current_mem_loc=0x3f000;                                                     //Monitor the storage location
static uint8_t advcounter=0;                                                          //counter to turn on off advertising automatically, for testing purpose only
static uint8_t sensor_data[4];                                                        //sensor data size, 10 for now to account for timestamp (2), current delivery (2), battery level (1-2), temperature (1-2), pressure(1-2)
static        uint32_t adc_value1;                                                    //For voltage sensing and storage - current measurement, needs static keyword to avoid random changes
static        uint32_t counter_adc1;                                                  //time stamped data, each counter at 1024 Hz, first two bytes time, last two bytes adc
static        uint16_t battery_volt;                                                  //battery_voltage measured.
static uint32_t sensor_counter=0;                                                     //counter to calculate timestamp of sensor info
static uint16_t accumulated_volume=0;                                                 //sum of overall counter for volume assessment
bool start_buzzer=false;                                                              //Flag for buzzer
static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];             //Save of ADC result in a 2D array
static bool                    m_saadc_initialized = false;                           //For initialize and uninitialize saadc

//****************************************variables for mock data set****************************************************//

static uint32_t time_stamp = 0;
static uint32_t local_mem_lock = 0x3f000;
static uint8_t volumes_pumped[] = {5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
static uint8_t volumes_pumped_mol_based[] = {5, 5, 5, 5, 5, 5, 5, 5, 5, 5}; 

NRF_BLE_GATT_DEF(m_gatt);                                                             /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                               /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                   /**< Advertising module instance. */
APP_TIMER_DEF(saadc_timer);                                                           /**< SAADC timer id) */
APP_TIMER_DEF(shdn_timer);                                                            //SHDN counter, pair with timeout event to determine if set pin high or low
APP_TIMER_DEF(adv_timer);                                                             //Advertising timer manual control. will not be used in real case, test only
APP_TIMER_DEF(sensor_timer);                                                          //sensor timer for sensor info update.
APP_TIMER_DEF(delivery_timer);                                                        //delivery timer for quick info update //so far choose the simplest, future might separate basal and bolus modes
NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,
    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x3f000,
    .end_addr   = 0x57000,
};

/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}


static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

/* YOUR_JOB: Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz); //This is with macro definition, but could directly use structure as below.
 */
 // Step1
static ble_delivery_t m_delivery_service;
static ble_sensor_t m_sensor_service;

// YOUR_JOB: Use UUIDs for service(s) used in your application. STEP5
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_SENSOR_SERVICE_UUID,BLE_UUID_TYPE_VENDOR_BEGIN},
    {BLE_UUID_DELIVERY_SERVICE_UUID,BLE_UUID_TYPE_BLE}
};


//Function details

//Built-in temperature measurement
static int32_t read_temperature()
{
    uint32_t err_code=sd_temp_get(&temp);  //Start the temperature measurement
    APP_ERROR_CHECK(err_code);
    temp=(nrf_temp_read()/4);
    return temp;
}

// Wait for flash, might not use due to non-functioning
void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        nrf_pwr_mgmt_run();
    }
}

//Initialize flash for data saving
void flash_save_init(void)
{
    ret_code_t err_code;
    nrf_fstorage_api_t * p_fs_api;
    /* Initialize an fstorage instance using the nrf_fstorage_sd backend.
     * nrf_fstorage_sd uses the SoftDevice to write to flash. This implementation can safely be
     * used whenever there is a SoftDevice, regardless of its status (enabled/disabled). */
    p_fs_api = &nrf_fstorage_sd;
    err_code=nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_fstorage_erase(&fstorage, 0x3f000, 24, NULL);  //erase where size enough pages for data storage. Requires 98 pages for 400KB.
//  Calculate exact size needed based on 4 Hz data operation and every minute rest data. Other sensors every 5s update once, real time.
//    estimate 2 mL delivery in 3 days. running at 0.5 ul/s, idle time data update 1/60 each, only 36.2 K points, about 145KB.
    wait_for_flash_ready(&fstorage);
    APP_ERROR_CHECK(err_code);
}

//Event handler for data storage
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
//        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
//            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
//                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
//            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
  //                       p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

// timeout handler for initiating saadc sample events, timing needs revise to implement macro
static void saadc_timer_timeout_handler(void *p_context)
{
    uint32_t err_code;
    UNUSED_PARAMETER(p_context);
//    if ((counter<200)||(counter%1280==1))              //pump stage, need to run SAADC, only first 10 s and every minute)
    if (counter<200)
    {
        if(!m_saadc_initialized)
        {
            saadc_init();                                              //Initialize the SAADC. In the case when SAADC_SAMPLES_IN_BUFFER > 1 then we only need to initialize the SAADC when the the buffer is empty.
        }
        m_saadc_initialized = true;                                    //Set SAADC as initialized
        {
        nrfx_saadc_sample();   
                          }//Trigger the SAADC SAMPLE task		
        LEDS_INVERT(BSP_LED_1_MASK);                                   //Toggle LED2 to indicate SAADC sampling start
    }		
}

static void pump_gpio_control_set(uint8_t * p_pump_gpio_control) //set GPIO control based on array
{
  *p_pump_gpio_control==1? nrf_gpio_pin_set(MULTIPLEXER_A0):nrf_gpio_pin_clear(MULTIPLEXER_A0) ;
  *(p_pump_gpio_control+1)==1? nrf_gpio_pin_set(MULTIPLEXER_A1):nrf_gpio_pin_clear(MULTIPLEXER_A1) ;
  *(p_pump_gpio_control+2)==1? nrf_gpio_pin_set(MULTIPLEXER_A2):nrf_gpio_pin_clear(MULTIPLEXER_A2) ;
}

static uint16_t basal_time_interval_ct_update(uint16_t basal_rate_ct)
{
    return (MAX_BASAL_INTERVAL/SHDN_INTERVAL*1000/basal_rate_ct);
}

static void adv_timer_timeout_handler(void *p_context)  //every 30s timeout
{
    ret_code_t err_code;
    UNUSED_PARAMETER(p_context);

    if (advcounter==1&&actual_state==BSP_INDICATE_ADVERTISING)  //use 1 to account for potential bsp event. So will advertising at least 30s at most 60s.
    {
        err_code=ble_advertising_stop(&m_advertising);
        APP_ERROR_CHECK(err_code);
        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);
        advcounter++;
    }
    else if (advcounter==59&&actual_state==BSP_INDICATE_IDLE) //reset timer counter, 59 for 30 min, 3 for 1 min.
    {
        advertising_init();
        advertising_start(false);
        advcounter=0;
    }
    else
    advcounter++;
}

static void sensor_timer_timeout_handler(void *p_context) //every 5s
{
    sensor_counter++;
    uint8_t sensor_value[8];  
    static uint8_t previous_sensor_value[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    sensor_value[0]=(uint8_t) sensor_counter>>8;      //time unit every 5s count 1, high part
    sensor_value[1]=(uint8_t) sensor_counter;
    sensor_value[2]=(uint8_t) 250-accumulated_volume;   //left volume, needs to get from delivery service, may need 2 bytes if using 0.5 uL unit, say targeted volume is 250 uL
    sensor_value[3]=read_temperature();
    sensor_value[4]=(uint8_t) battery_volt/256;   //In the end should show battery percentage, will have to implement a conversion function
    //sensor_value[5]=read_pressure(); 
    sensor_value[6]=basal_rate;
    sensor_value[7]=bolus_rate;
    //Select when to update
    if (previous_sensor_value[2]!=sensor_value[2]||previous_sensor_value[3]!=sensor_value[3]||previous_sensor_value[4]!=sensor_value[4])
    {
       // sensor_characteristic_update(&m_sensor_service,sensor_value);
    }
    //save current sensor info
    for (int i=0;i<8;i++)
    {
    previous_sensor_value[i]=sensor_value[i];
    }
    // Buzzer control
    if ((!start_buzzer&&sensor_value[3]>=0x1C)||(start_buzzer&&sensor_value[3]<0x1C))
        {
            nrf_gpio_pin_toggle(BUZZER_PIN);
            start_buzzer=!start_buzzer;
        }
}

static void delivery_timer_timeout_handler(void *p_context) //every min, could save a lot of flash space if real time compensation not required, only 8K needed.
{
        static uint32_t time_volume;
        long double delivery_value[4];  //may use flash data
        if (volumes_pumped[i]!=0)  //only update when there is delivery
        {
            accumulated_volume=250;//+=volumes_pumped_mol_based[i];
            time_volume=(time_stamp);
            if (current_mem_loc<0x57000)
                {
                     //NRF_LOG_INFO("Clock: \"%x\" ms.", time_stamp);
                     ret_code_t err_code = nrf_fstorage_write(&fstorage, current_mem_loc,&time_volume, sizeof(time_volume), NULL); //Store timestamped data.
                     APP_ERROR_CHECK(err_code);
                     wait_for_flash_ready(&fstorage); //kills the application
                     APP_ERROR_CHECK(err_code);
                     current_mem_loc+=4;
                     //NRF_LOG_INFO("State: \"%x\" ", actual_state);
                     if(actual_state == BSP_INDICATE_CONNECTED)
                     {
                         //NRF_LOG_INFO("ble coinnected");
                         uint8_t *p_counter_adc1 = &counter_adc1;
                         ret_code_t err_code = nrf_fstorage_read(&fstorage, local_mem_lock, &counter_adc1, sizeof(counter_adc1));
                         APP_ERROR_CHECK(err_code);
                         sensor_characteristic_update(&m_sensor_service,p_counter_adc1);
                         local_mem_lock += 4;
                     }
                }
            delivery_counter=0;   //reset delivery counter after timestamping and data storage
        }
        time_stamp+=5;
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            advcounter=0;
            break;

        default:
            break;
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       ret_code_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
    err_code = app_timer_create(&saadc_timer,
                                APP_TIMER_MODE_REPEATED,
                                saadc_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&shdn_timer,
                                APP_TIMER_MODE_REPEATED,
                                shdn_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&adv_timer,
                                APP_TIMER_MODE_REPEATED,
                                adv_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&sensor_timer,
                                APP_TIMER_MODE_REPEATED,
                                sensor_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&delivery_timer,
                                APP_TIMER_MODE_REPEATED,
                                delivery_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Add code to initialize the services used by the application.
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;

       // Initialize XXX Service.
       memset(&xxs_init, 0, sizeof(xxs_init));

       xxs_init.evt_handler                = NULL;
       xxs_init.is_xxx_notify_supported    = true;
       xxs_init.ble_xx_initial_value.level = 100;

       err_code = ble_bas_init(&m_xxs, &xxs_init);
       APP_ERROR_CHECK(err_code);

       // Initialize YYY Service.
       memset(&yys_init, 0, sizeof(yys_init));
       yys_init.evt_handler                  = on_yys_evt;
       yys_init.ble_yy_initial_value.counter = 0;

       err_code = ble_yy_service_init(&yys_init, &yy_init);
       APP_ERROR_CHECK(err_code); //step2 could use a function call instead a below
     */
     delivery_service_init(&m_delivery_service);
     sensor_service_init(&m_sensor_service);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
        actual_state=BSP_INDICATE_ADVERTISING;
        advcounter=0;
    }
}


/**@brief Function for handling a Connection Parameters error.
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
    ret_code_t             err_code;
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


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */
    ret_code_t err_code;

    // Start application timers. Possible add delays to offset them?
    err_code = app_timer_start(saadc_timer, SAADC_INTERVAL, NULL);    //every 250 ms timeout
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(shdn_timer, SHDN_INTERVAL, NULL);  //Every 1 s flip SHDN
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(adv_timer, ADV_ON, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(sensor_timer, SENSOR_INTERVAL, NULL);    //every 5 s timeout
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(delivery_timer, SENSOR_INTERVAL, NULL);    //every min timeout
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for stop timers.
 */
void application_timers_stop(void)
{
    ret_code_t err_code;

    // stop
    err_code = app_timer_stop(saadc_timer);  
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_stop(shdn_timer);  //Every 1 s flip SHDN
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_stop(adv_timer);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_stop(sensor_timer);    //every 5 s timeout
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_stop(delivery_timer);    //every min timeout
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)  //not used for now
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

//    // Prepare wakeup buttons.
//    err_code = bsp_btn_ble_sleep_mode_prepare();
//    APP_ERROR_CHECK(err_code);
//
//    // Go to system-off mode (this function will not return; wakeup will cause a reset).
//    err_code = sd_power_system_off();
//    APP_ERROR_CHECK(err_code);
    
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:  //fast advertising
//            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING); 
            APP_ERROR_CHECK(err_code);
            actual_state=BSP_INDICATE_ADVERTISING;
            break;

        case BLE_ADV_EVT_IDLE:  //idle
//            sleep_mode_enter();
//            break;
            err_code=ble_advertising_stop(&m_advertising);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)  //registered with observor in ble_stack_init(), handler of NRF_SDH_BLE_OBSERVER() function
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
//            NRF_LOG_INFO("Disconnected.");
            actual_state=BSP_INDICATE_ADVERTISING;
            advcounter=0;
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
//            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            actual_state=BSP_INDICATE_CONNECTED;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
//            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
//            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            actual_state=BSP_INDICATE_ADVERTISING;
            advcounter=0;
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
//            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            actual_state=BSP_INDICATE_ADVERTISING;
            advcounter=0;
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    // STEP 3C register a handler for responding to connect and disconnect events
    NRF_SDH_BLE_OBSERVER(m_sensor_service_observer, APP_BLE_OBSERVER_PRIO,ble_sensor_service_on_ble_evt,(void*)&m_sensor_service);
    NRF_SDH_BLE_OBSERVER(m_delivery_service_observer, APP_BLE_OBSERVER_PRIO,ble_delivery_service_on_ble_evt,(void*)&m_delivery_service);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

//    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_KEY_0:   //pumping mode selection in order of 2, 5 and 1, default is 2, should be disabled if already started pumping. Will do it later.
                command_bolus_delivery = true;
                command_delivery_rate_ct = 10;//bolus_rate; for 5uL (10s pumping)
                new_command_received=true;
                new_delivery=true;
                break;

        case BSP_EVENT_KEY_1:   //GPIO manual current control        
              break;

        case BSP_EVENT_KEY_2:   //Start test and data collection control


            break;

        case BSP_EVENT_KEY_3:
            if (!start_test)  //try to start test
            {
                flash_save_init();
                application_timers_start();//try to start timer here to start test.
                //advertising_start();  //does not need, starts at the next if-else
                start_test=true;
            }
            if (actual_state==BSP_INDICATE_CONNECTED)   //connected states, goes to idle
            {
                err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                actual_state=BSP_INDICATE_IDLE;
                advcounter=1;
            }
            else if (actual_state==BSP_INDICATE_ADVERTISING)
            {
                err_code = ble_advertising_stop(&m_advertising);  //actual state already built in
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            else
            {
                //update manuf_data and start
                advertising_init();
                advertising_start(false);  //actual state already labeled actual_state inside
                advcounter=0;
            }
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            actual_state=BSP_INDICATE_ADVERTISING;
            advcounter=0;
            break;

        case BSP_EVENT_WHITELIST_OFF:   //by default is key_1 (2nd button)
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;  //BSP_EVENT_KEY_0

        default:
            break;
    }
    err_code = bsp_indication_set(actual_state);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;  // Struct containing advertising parameters
    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&init, 0, sizeof(init));

    ble_advdata_manuf_data_t                  manuf_data; //Variable to hold manufacturer specific data
//    uint8_t *data                            = sensor_data; //Our data to advertise
    manuf_data.company_identifier             =  0x0059; //Nordics company ID
    manuf_data.data.p_data                    = sensor_data; //data;
    manuf_data.data.size                      = sizeof(sensor_data);
    init.advdata.p_manuf_specific_data = &manuf_data;

    init.advdata.name_type = BLE_ADVDATA_SHORT_NAME; // Use a shortened name
    init.advdata.short_name_len = 6; // Advertise only first 6 letters of name
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
 //   init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
 //   init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    //Step 6
    	init.srdata.uuids_complete.uuid_cnt=sizeof(m_adv_uuids)/sizeof(m_adv_uuids[0]);
        init.srdata.uuids_complete.p_uuids=m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
    actual_state=BSP_INDICATE_ADVERTISING;
}

//customized function to stop advertising
uint32_t ble_advertising_stop(ble_advertising_t * const p_advertising)
{
    ret_code_t ret;
    ret= sd_ble_gap_adv_stop(p_advertising->adv_handle);
    if (ret != NRF_SUCCESS)
    {
        p_advertising->error_handler(ret);
    }
    actual_state=BSP_INDICATE_IDLE;
    return NRF_SUCCESS;
}


/* saadc_callbacks */
void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
//        LEDS_INVERT(BSP_LED_2_MASK);  //Toggle LED3 to indicate SAADC buffer full
//        static        uint32_t adc_value1;  //For voltage sensing and storage - current measurement, needs static keyword to avoid random changes
//        static        uint32_t counter_adc1; //time stamped data, each counter at 1024 Hz, first two bytes time, last two bytes adc
//        static        uint32_t battery_volt;  //battery_voltage measured.
        // set buffers so the SAADC can write to it again. This is either buffer[1] or buffer[2]
        err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
//  Advertised info, example only, may not need, or can greatly simplify
            adc_value1 =  p_event->data.done.p_buffer[0];
//            sensor_data[4]=p_event->data.done.p_buffer[1]/16; //battery voltage convert to 8 bit, 256 levels are enough.
            if (adc_value1<0x1000)    //get rid of junk reading
            {
                current_tick=app_timer_cnt_get();
                battery_volt=p_event->data.done.p_buffer[1];
                sensor_data[2]=adc_value1>>8;
                sensor_data[3]=adc_value1;
    //            NRF_LOG_INFO("%d\r\n", adc_value1);
                sensor_data[0]=current_tick>>8;
                sensor_data[1]=current_tick;
//                sensor_data[4]=battery_volt>>8;
//                sensor_data[5]=battery_volt;
//                sensor_data[6]=read_temperature();
            }

        nrfx_saadc_uninit();                                                                   //Unintialize SAADC to disable EasyDMA and save power
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               //Disable the SAADC interrupt
        NVIC_ClearPendingIRQ(SAADC_IRQn);                                                         //Clear the SAADC interrupt if set  //removed this line saves power by disabling nrf_nvic.c reduced 20 uA
        m_saadc_initialized = false;                                                              //Set SAADC as uninitialized
   }
}

//SAADC initialization
void saadc_init(void)
{
    ret_code_t err_code;
    nrfx_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config0;
    nrf_saadc_channel_config_t channel_config1;
    nrf_saadc_channel_config_t channel_config2;
    nrf_saadc_channel_config_t channel_config3;
	
    //Configure SAADC
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = SAADC_OVERSAMPLE;                                           //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
	
    //Initialize SAADC
    err_code = nrfx_saadc_init(&saadc_config, saadc_callback);                         //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
    APP_ERROR_CHECK(err_code);
		
    //Configure SAADC channel
    channel_config0.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config0.gain = NRF_SAADC_GAIN1_3;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config0.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config0.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config0.pin_p = NRF_SAADC_INPUT_AIN0;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config0.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config0.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config0.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin

    channel_config1.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config1.gain = NRF_SAADC_GAIN1_5;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config1.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config1.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config1.pin_p = NRF_SAADC_INPUT_AIN1;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config1.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config1.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config1.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin
    
    channel_config2.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config2.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config2.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config2.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config2.pin_p = NRF_SAADC_INPUT_AIN2;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config2.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config2.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config2.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin
    
    channel_config3.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config3.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config3.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config3.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config3.pin_p = NRF_SAADC_INPUT_AIN3;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config3.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config3.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config3.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin
	
    //Initialize SAADC channel
    err_code = nrfx_saadc_channel_init(0, &channel_config0);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_saadc_channel_init(1, &channel_config1);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_saadc_channel_init(2, &channel_config2);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_saadc_channel_init(3, &channel_config3);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
		
    if(SAADC_BURST_MODE)
    {
        NRF_SAADC->CH[0].CONFIG |= 0x01000000;                                            //Configure burst mode for channel 0. Burst is useful together with oversampling. When triggering the SAMPLE task in burst mode, the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer. If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.		
    }

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    APP_ERROR_CHECK(err_code);
    
    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
    APP_ERROR_CHECK(err_code);
}

 static void lfclk_config(void)  //Will it conflict with BLE?
{
    ret_code_t err_code = nrf_drv_clock_init();                        //Initialize the clock source specified in the nrf_drv_config.h file, i.e. the CLOCK_CONFIG_LF_SRC constant
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}


void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
	// Forward the TWI event to the driver
	bme280_twi_evt_handler(p_event, p_context);
}

void bme280_handler(bme280_twi_evt_t const * p_event, void * p_context)
{
	switch (p_event->type) {
		case BME280_TWI_MEASUREMENT_FETCHED:
			m_measurement_fetched = true;
			break;
		default:
			break;	
	}
}

void twi_init(void)
{
	const nrf_drv_twi_config_t twi_config = {
		.scl                = ARDUINO_SCL_PIN,
		.sda                = ARDUINO_SDA_PIN,
		.frequency          = NRF_TWI_FREQ_100K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init     = false
	};

	ret_code_t err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&m_twi);
}

void bme280_init(void) {
	const bme280_twi_config_t bme280_twi_config = {
		.addr = BME280_TWI_ADDR_1,
		.standby = BME280_TWI_STANDBY_250_MS,   //inactive mode see Register 0xF5 in official datasheet 011=0x03U
		.filter = BME280_TWI_FILTER_OFF,
		.temp_oversampling = BME280_TWI_OVERSAMPLING_X4,
                .p_oversampling=BME280_TWI_OVERSAMPLING_X4,
                .h_oversampling=BME280_TWI_OVERSAMPLING_X4
	};

	bme280_twi_init(&m_twi, &bme280_twi_config, bme280_handler, NULL);
	bme280_twi_enable();
}

static void log_temp(void)  //will revise
{
	bme280_twi_data_t data;
	bme280_twi_measurement_get(&data);
        temp=data.temp/100.00;
	NRF_LOG_INFO("Temperature: " NRF_LOG_FLOAT_MARKER " degrees Celsius.\r\n",
			NRF_LOG_FLOAT(((float)temp)));
	NRF_LOG_INFO("Pressure: " NRF_LOG_FLOAT_MARKER " Pa.\r\n",
			NRF_LOG_FLOAT(((float)data.press)));
	NRF_LOG_INFO("Humidity " NRF_LOG_FLOAT_MARKER " \%RH.\r\n",
			NRF_LOG_FLOAT(((float)data.hum)/1024));
	NRF_LOG_FLUSH();
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;
    nrf_gpio_cfg_output(SHDN_PIN);
    nrf_gpio_pin_clear(SHDN_PIN);
    nrf_gpio_cfg_output(BUZZER_PIN);
    nrf_gpio_pin_clear(BUZZER_PIN);
    //prepare for current level selection, initial 2mA on 6 mA off
    nrf_gpio_cfg_output(SIXMA_PIN);
    nrf_gpio_pin_clear(SIXMA_PIN);
    nrf_gpio_cfg_output(TWOMA_PIN);
    nrf_gpio_pin_set(TWOMA_PIN);
    //prepare for multiple pump selection
    nrf_gpio_cfg_output(MULTIPLEXER_A0);
    nrf_gpio_cfg_output(MULTIPLEXER_A1);
    nrf_gpio_cfg_output(MULTIPLEXER_A2);
    pump_gpio_control[START_PUMP][0]==1? nrf_gpio_pin_set(MULTIPLEXER_A0):nrf_gpio_pin_clear(MULTIPLEXER_A0) ;
    pump_gpio_control[START_PUMP][1]==1? nrf_gpio_pin_set(MULTIPLEXER_A1):nrf_gpio_pin_clear(MULTIPLEXER_A1) ;
    pump_gpio_control[START_PUMP][2]==1? nrf_gpio_pin_set(MULTIPLEXER_A2):nrf_gpio_pin_clear(MULTIPLEXER_A2) ;
    // Initialize.
    log_init();
    lfclk_config();
    timers_init();
    twi_init();
    bme280_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    NRF_POWER->DCDCEN = 1;                           //Enabling the DCDC converter for lower current consumption
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    conn_params_init();
    peer_manager_init();

    flash_save_init();
    // Start execution.
    NRF_LOG_INFO("Template example started.");
    // Enter main loop.

//This is loop structure getting readings, should implement into sensor reading interrupt
//everytime reset the fetch flag to be false and run data fetch. 
//Get data, once done deal with data.
	while (true) {
		nrf_delay_ms(500);

		m_measurement_fetched = false;
		bme280_twi_measurement_fetch();

		do {
			__WFE();
		} while (m_measurement_fetched == false);

		log_temp();
	}

    for (;;)
    {
        idle_state_handle();
    }
}

