/* Copyright (c) 2017 WISOL Corp. All Rights Reserved.
 *
 * The information contained herein is property of WISOL Corp.
 * Terms and conditions of usage are described in detail in WISOL STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * This heading must NOT be removed fromthe file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "cfg_board_def.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_flash.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "peer_manager.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "cfg_app_main.h"
#include "cfg_sigfox_module.h"
#include "cfg_bma250_module.h"
#include "cfg_tmp102_module.h"
#include "cfg_gps_module.h"
#include "cfg_dbg_log.h"
#include "cfg_wifi_module.h"
#include "cfg_board.h"
#include "nrf_drv_gpiote.h"
#include "fds.h"
#include "nrf_drv_twi.h"
#include "nfc_t2t_lib.h"
#include "nfc_uri_msg.h"
#include "nfc_launchapp_msg.h"
#include "hardfault.h"
#include "nrf_drv_clock.h"
#include "cfg_external_sense_gpio.h"


#include "airqule_ble.h"
#include "bme680.h"


#define MAIN_INTERVAL_MS                120000
#define MEASURE_INTERVAL                120
#define BLE_ENABLED                     1
#define WIFI_ENABLED                    1
#define GPS_ENABLED                     1
#define SIGFOX_ENABLED                  0
#define CCS811_ENABLED                  0
#define BME280_ENABLED                  0

APP_TIMER_DEF(main_wakeup_timer_id);
APP_TIMER_DEF(main_sec_tick_timer_id);

module_parameter_t m_module_parameter;  //setting values
nus_service_parameter_t m_nus_service_parameter;
module_peripheral_data_t m_module_peripheral_data;
module_peripheral_ID_t m_module_peripheral_ID;
bool m_module_parameter_update_req;
uint8_t   avg_report_volts;

bool main_wakeup_evt_expired = false;
int interrupt_cnt = 0;
uint8_t   m_main_sec_tick;

bool accel_interrupt = false;
volatile bool main_wakeup_interrupt; //from cfg_board

unsigned int main_get_param_val(module_parameter_item_e item)
{
    unsigned int ret = 0;
    switch(item)
    {
        case module_parameter_item_snek_testmode_enable:
            ret = m_module_parameter.sigfox_snek_testmode_enable;
            break;

        case module_parameter_item_gps_tracking_time_sec:
            ret = m_module_parameter.gps_acquire_tracking_time_sec;
            break;

        default:
            break;
    }
    return ret;
}

bool module_parameter_erase_and_reset(void)
{
    return false;
}

void module_parameter_check_update(void)
{
    if(m_module_parameter_update_req)
    {
        m_module_parameter_update_req = false;
    }
    return;
}

void main_set_param_val(module_parameter_item_e item, unsigned int val)
{
    return;
}


static void main_sec_tick_timer_handler(void * p_context)
{
    m_main_sec_tick++;
}

static void main_sec_tick_timer_init(void)
{
    uint32_t err_code;

    err_code = app_timer_create(&main_sec_tick_timer_id, APP_TIMER_MODE_REPEATED, main_sec_tick_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(main_sec_tick_timer_id, APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);

}

static void main_wakeup_timer_timeout_handler(void * p_context)
{
    cPrintLog(CDBG_MAIN_LOG, "Wakeup timer expired\n");
    main_wakeup_evt_expired = true;
}

static void main_wakeup_timer_init(void)
{
    uint32_t err_code;
    err_code = app_timer_create(&main_wakeup_timer_id, APP_TIMER_MODE_REPEATED, main_wakeup_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);

}
static void main_wakeup_timer_start(uint32_t msec)
{
    uint32_t err_code;
    uint32_t timeout_ticks; 

    timeout_ticks = APP_TIMER_TICKS(msec, APP_TIMER_PRESCALER);
    cPrintLog(CDBG_MAIN_LOG, "Wakeup start ticks:%d\n", timeout_ticks);
    err_code = app_timer_start(main_wakeup_timer_id, timeout_ticks, NULL);  //timer to wake up every 600s
    APP_ERROR_CHECK(err_code);

}

static void main_resource_init(void)
{
    //timer Initialize
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    ble_stack_init();
    main_wakeup_timer_init();
    main_sec_tick_timer_init();

    //Handling low power
    nrf_gpio_cfg_output(PIN_DEF_2ND_POW_EN);
    nrf_gpio_pin_write(PIN_DEF_2ND_POW_EN, 0);
}

void main_wakeup_interrupt_set(void)
{
    main_wakeup_interrupt = true;
}

/**
 * @brief Callback function for handling iterrupt from accelerometer.
 */
void accelerometer_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    main_wakeup_interrupt_set();
    //accel_interrupt=true; 
    cPrintLog(CDBG_MAIN_LOG,"%s", "Interrupt flag raised !\n");
    interrupt_cnt++;
    ble_advertising_start(BLE_ADV_MODE_FAST);
}

/**
 * @brief  function for setting interrupt from accelerometer.
 */
void accelerometer_interrupt_init(void)
{
    uint32_t err_code;
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);

    if (!nrf_drv_gpiote_is_init())
    {
        // Initialize GPIO Interrupt Driver
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }
    // Initialize PIN_DEF_ACC_INT1 as GPIOTE Input Pin
    err_code = nrf_drv_gpiote_in_init(PIN_DEF_ACC_INT1, &in_config, accelerometer_int_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_DEF_ACC_INT1, true);
}

static void Init_accel(void)
{
    cfg_i2c_master_init();
    cfg_bma250_timer_create();
    bma250_set_state(NONE_ACC);
    if(bma250_get_state() == NONE_ACC)
    {
        cPrintLog(CDBG_MAIN_LOG, "%s %d ACC MODULE started\n",  __func__, __LINE__);
        bma250_set_state(SET_S);
        cfg_bma250_timers_start();
    }  
    accelerometer_interrupt_init();

    while(1)
    {
        nrf_delay_ms(200);
        if(bma250_get_state() == NONE_ACC || bma250_get_state() == EXIT_ACC)
        {
            cfg_bma250_timers_stop();
            break;
        }
    }
}

static void Init_sigfox(void)
{
//    m_module_parameter.sigfox_snek_testmode_enable = true;  //snek test mode
    cfg_sigfox_prepare_start();
    sigfox_set_rcz(RCZ_1);
    cfg_sigfox_set_powerlevel(15);
}

static void Init_wifi(void)
{
    wifi_drv_init();
    set_scan_interval(10);
}


static void Init_gps(void)
{
    gps_init();
    set_cn0_current_savetime_enable(module_parameter_item_gps_cn0_current_savetime_enable, CGPS_CNO_CHECK_DISABLE);  
    gps_tracking_set_interval(module_parameter_item_gps_tracking_time_sec, 30);
}

static void get_device_ids(/*uint8_t *ble_MAC; uint8_t *wifi_MAC, uint8_t *sigfox_ID, uint8_t *sigfox_PAC, uint8_t *chip_UUID*/) {
    uint8_t     ble_MAC[6];
    uint8_t     wifi_MAC[6];
    uint8_t     sigfox_ID[4];
    uint8_t     sigfox_PAC[8];
    uint8_t     chip_UUID[16];
    ble_gap_addr_t      addr;
    uint32_t err_code = sd_ble_gap_addr_get(&addr);
    APP_ERROR_CHECK(err_code);

    for (uint8_t i = 0; i < 6; i++) {
        ble_MAC[i] = addr.addr[5 - i];
    }

    //memcpy(ble_MAC, m_module_peripheral_ID.ble_MAC, sizeof(m_module_peripheral_ID.ble_MAC));
    memcpy(wifi_MAC, m_module_peripheral_ID.wifi_MAC_STA, sizeof(m_module_peripheral_ID.wifi_MAC_STA));
    memcpy(sigfox_ID, m_module_peripheral_ID.sigfox_device_ID, sizeof(m_module_peripheral_ID.sigfox_device_ID));
    memcpy(sigfox_PAC, m_module_peripheral_ID.sigfox_pac_code, sizeof(m_module_peripheral_ID.sigfox_pac_code));
    memcpy(chip_UUID, m_module_peripheral_ID.UUID, sizeof(m_module_peripheral_ID.UUID));
    
    cPrintLog(CDBG_IOT_INFO, "BLE MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", ble_MAC[0], ble_MAC[1], ble_MAC[2], ble_MAC[3], ble_MAC[4], ble_MAC[5]);
    cPrintLog(CDBG_IOT_INFO, "Wifi MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", wifi_MAC[0], wifi_MAC[1], wifi_MAC[2], wifi_MAC[3], wifi_MAC[4], wifi_MAC[5]);
    cPrintLog(CDBG_IOT_INFO, "Sigfox ID:  [%02x%02x%02x%02x]\t", sigfox_ID[0], sigfox_ID[1], sigfox_ID[2], sigfox_ID[3]);
    cPrintLog(CDBG_IOT_INFO, "Sigfox PAC:  [%02x%02x%02x%02x%02x%02x%02x%02x]\n", sigfox_PAC[0], sigfox_PAC[1], sigfox_PAC[2], sigfox_PAC[3], sigfox_PAC[4], sigfox_PAC[5], sigfox_PAC[6], sigfox_PAC[7]);
    cPrintLog(CDBG_IOT_INFO, "Board ID: %02d\n", chip_UUID);
}

static void init_module(void)
{   
    main_resource_init();
    Init_accel();
    Init_sigfox();
    Init_wifi();
    Init_ble(DEVICE_NAME);
    Init_gps();
}

static void Wifi_get_scanned_BSSID(unsigned char *bssid_buf)
{
    uint32_t cnt;
    uint8_t *ssid;
    int32_t *rssi;
    uint8_t *bssid;
    
    memset(bssid_buf, 12, 0);
    if(start_AP_scan() == CWIFI_Result_OK)
    {
        get_AP_scanResult(&cnt, &ssid, &rssi, &bssid);
        cPrintLog(CDBG_WIFI_INFO, "Scanned AP:\n");
        
        for (int i = 0; i < cnt; i++) {
            cPrintLog(CDBG_WIFI_INFO, "AP [%"PRId32"]\t [%02x:%02x:%02x:%02x:%02x:%02x], ssid:%s\n", 
                rssi[i], bssid[0+(i*6)], bssid[1+(i*6)], bssid[2+(i*6)], bssid[3+(i*6)], bssid[4+(i*6)], bssid[5+(i*6)], ssid);
        }

        memcpy(bssid_buf, bssid, 12);
    }
}

static void Sigfox_send_payload(unsigned char *payload)
{
    sigfox_send_payload(payload, NULL);
}

static bool gps_acquire(unsigned char *position)
{
    uint8_t *pGpsInfo;
    bool ret = false;
    
    memset(position, 12, 0);
    if(start_gps_tracking() == CGPS_Result_OK)
    {
        while(!(cGPS_waiting_tracking_end_check()));
            
        if(cGps_nmea_get_bufPtr(&pGpsInfo) == CGPS_Result_OK)
        {
            memcpy(position, pGpsInfo, 12);
            ret = true;
        }
    }

    while(cGps_bus_busy_check());  //wait for release gps spi 
    return ret;
}

bool module_parameter_get_bootmode(int *bootmode)
{
    return false;
}

void main_examples_prepare(void)
{
    return;
}
/*
void bme280_init(void)
{
    BME280_Ret BME280RetVal = bme280_init();
    if (BME280_RET_OK == BME280RetVal) {
        NRF_LOG_INFO("BME280 init Done\r\n");
    }
    else {
        NRF_LOG_ERROR("BME280 init Failed: Error Code: %d\r\n", (int32_t)BME280RetVal); 
    }

    //setup BME280 if present
    uint8_t conf = bme280_read_reg(BME280REG_CTRL_MEAS);
    NRF_LOG_INFO("CONFIG: %x\r\n", conf);

    bme280_set_oversampling_hum(BME280_OVERSAMPLING_1);
    bme280_set_oversampling_temp(BME280_OVERSAMPLING_1);
    bme280_set_oversampling_press(BME280_OVERSAMPLING_1);

    conf = bme280_read_reg(BME280REG_CTRL_MEAS);
    //NRF_LOG_INFO("CONFIG: %x\r\n", conf);
    //Start sensor read for next pass
    bme280_set_mode(BME280_MODE_FORCED);
    NRF_LOG_INFO("BME280 configuration done\r\n");

    static int32_t raw_t  = 0;
    static uint32_t raw_p = 0;
    static uint32_t raw_h = 0;

    // Get raw environmental data
    raw_t = bme280_get_temperature();
    raw_p = bme280_get_pressure() * 0.003906; // (/25600*100);
    raw_h = bme280_get_humidity() * 0.097656;// (/1024*100)

    bme280_set_mode(BME280_MODE_SLEEP);
}*/

int main(void)
{
    unsigned char bssid[12];
    unsigned char position[12];

    init_module();

    cPrintLog(CDBG_IOT_INFO, "+--------------------------------------------------------+\n");
    cPrintLog(CDBG_IOT_INFO, "|                    AirQule Starting                    |\n");
    get_device_ids();
    cPrintLog(CDBG_IOT_INFO, "+--------------------------------------------------------+\n");
    
    uint8_t * chip_id;
    //bme680_i2c_bus_read(BME680_CHIP_ID_ADDRESS, chip_id, 1);
    cPrintLog(CDBG_IOT_INFO, "chip_id: [%02x]", chip_id);

    /*uint8_t * chip_id2;
    get_bme680_chipid(chip_id2);
    cPrintLog(CDBG_IOT_INFO, "chip_id2: [%02x]", chip_id2);*/
    
    main_wakeup_timer_start(MAIN_INTERVAL_MS);  // Here you shoud set up a timer to wake up every 618s

    while(1)
    {
        if (WIFI_ENABLED) {
            Wifi_get_scanned_BSSID(bssid);
            if (SIGFOX_ENABLED) Sigfox_send_payload(bssid);
        }
        if (GPS_ENABLED && gps_acquire(position) == true) { 
            if (SIGFOX_ENABLED) Sigfox_send_payload(position);
        }

        // Here you wait for the timer event.
        while(1)
        {
            if(main_wakeup_evt_expired && interrupt_cnt >= 3)
            {
                main_wakeup_evt_expired=false;
                interrupt_cnt = 0;
                break;
            }
            sd_app_evt_wait();
        }
    }
}
