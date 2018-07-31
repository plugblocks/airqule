#ifndef AIRQULE_BLE_H_
#define AIRQULE_BLE_H_

#define DEVICE_NAME                     "AirQule"

//unsigned int main_get_param_val(module_parameter_item_e item);
void on_ble_evt(ble_evt_t * p_ble_evt);
void ble_evt_dispatch(ble_evt_t * p_ble_evt);
void sys_evt_dispatch(uint32_t event);
void ble_stack_init(void);
void gap_params_init(const char *payload, int payload_size);
void advertising_init(void);
void Ble_set_payload(const char *name);
void Init_ble(const char *name);

#endif