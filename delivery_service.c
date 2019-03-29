
#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "delivery_service.h"
#include "ble_srv_common.h"
#include "app_error.h"

// ALREADY_DONE_FOR_YOU: Declaration of a function that will take care of some housekeeping of ble connections related to our service and characteristic
void ble_delivery_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    // OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
		ble_delivery_t * p_delivery_service =(ble_delivery_t *) p_context;
		switch (p_ble_evt->header.evt_id)
                {
                    case BLE_GAP_EVT_CONNECTED:
                        p_delivery_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                        break;
                    case BLE_GAP_EVT_DISCONNECTED:
                        p_delivery_service->conn_handle = BLE_CONN_HANDLE_INVALID;
                        break;
                    default:
                        // No implementation needed.
                        break;
                }
		
}

/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_delivery_service        Delivery Service structure.
 *
 */
static uint32_t delivery_char_add(ble_delivery_t * p_delivery_service)
{
    // OUR_JOB: Step 2.A, Add a custom characteristic UUID //stepB1
		uint32_t            err_code;
		ble_uuid_t          char_uuid;
		ble_uuid128_t       base_uuid = BLE_UUID_OUR_BASE_UUID;
		char_uuid.uuid      = BLE_UUID_DELIVERY_CHARACTERISTIC_UUID;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
		APP_ERROR_CHECK(err_code);  
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic //StepB5
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 1;

    
    // OUR_JOB: Step 3.A, Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vloc                = BLE_GATTS_VLOC_STACK; //BLE_GATTS_VLOC_USER; //   
		char_md.p_cccd_md           = &cccd_md;
		char_md.char_props.notify   = 1;

   
    
    // OUR_JOB: Step 2.B, Configure the attribute metadata StepB2
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));  
		attr_md.vloc        = BLE_GATTS_VLOC_STACK; //BLE_GATTS_VLOC_USER;
    
    
    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic //stepB7
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute  //stepB3 store uuid and metadata from B1 and B2
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
		attr_char_value.p_uuid      = &char_uuid;
		attr_char_value.p_attr_md   = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes //stepB8, try to use 32 bits to record timer counter and counter of delivery, better with 20 bit time counter and 12 bit timecounte For flash storage
		attr_char_value.max_len     = 6;
		attr_char_value.init_len    = 6;
		uint8_t value[6]            = {0x00,0x00,0x00,0x00,0x00,0x00};  //time+delivered volume each time slot (flash), accumulated volume (not necessary for flash)
//                		attr_char_value.max_len     = 4;
//		attr_char_value.init_len    = 4;
//		uint8_t value[4]            = {0x00,0x00,0x00,0x00};  //time+delivered volume each time slot (flash), accumulated volume (not necessary for flash)
		attr_char_value.p_value     = value;


    // OUR_JOB: Step 2.E, Add our new characteristic to the service //StepB6
		err_code = sd_ble_gatts_characteristic_add(p_delivery_service->service_handle,
                                   &char_md,
                                   &attr_char_value,
                                   &p_delivery_service->char_handles);
		APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}


/**@brief Function for initiating our new service.
 *
 * @param[in]   p_our_service        Our Service structure.
 *
 */
void delivery_service_init(ble_delivery_t * p_delivery_service)
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions

    // FROM_SERVICE_TUTORIAL: Declare 16-bit service and 128-bit base UUIDs and add them to the BLE stack step3
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_OUR_BASE_UUID;
    service_uuid.uuid = BLE_UUID_DELIVERY_SERVICE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    
    
    // OUR_JOB: Step 3.B, Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
		p_delivery_service->conn_handle = BLE_CONN_HANDLE_INVALID;

    // FROM_SERVICE_TUTORIAL: Add our service step4
		err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_delivery_service->service_handle);
    
    APP_ERROR_CHECK(err_code);
    
    // OUR_JOB: Call the function to add our new characteristic to the service. StepA0
    delivery_char_add(p_delivery_service);
}

// ALREADY_DONE_FOR_YOU: Function to be called when updating characteristic value
void delivery_characteristic_update(ble_delivery_t *p_delivery_service, uint8_t *delivery_value)
{
    // OUR_JOB: Step 3.E, Update characteristic value
		if (p_delivery_service->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
				uint16_t               len = 6;
				ble_gatts_hvx_params_t hvx_params;
				memset(&hvx_params, 0, sizeof(hvx_params));

				hvx_params.handle = p_delivery_service->char_handles.value_handle;
				hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset = 0;
				hvx_params.p_len  = &len;
				hvx_params.p_data = /*(uint8_t*)*/delivery_value;  

				sd_ble_gatts_hvx(p_delivery_service->conn_handle, &hvx_params);
		}

}
