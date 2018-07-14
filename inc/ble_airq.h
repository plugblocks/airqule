#ifndef BLE_AIRQ_H_
#define BLE_AIRQ_H_

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define APP_BEACON_INFO_LENGTH          0x11

uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH];

unsigned int main_get_param_val(module_parameter_item_e item);
void on_ble_evt(ble_evt_t * p_ble_evt);
void ble_evt_dispatch(ble_evt_t * p_ble_evt);
void sys_evt_dispatch(uint32_t event);
void ble_stack_init(void);
void gap_params_init(const char *payload, int payload_size);
void advertising_init(void);
void Ble_set_payload(const char *name);
void Init_ble(void);

#endif