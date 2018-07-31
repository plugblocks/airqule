#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_log.h"
#include "nrf_drv_twi.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "cfg_dbg_log.h"
#include "bme680.h"
#include "cfg_board_def.h"
#include "cfg_board.h"
#include "nrf_delay.h"

extern volatile bool spi_xfer_done;  // Flag used to indicate that SPI instance completed the transfer.

extern const nrf_drv_twi_t m_twi;

/*static uint8_t       m_tx_buf[20];
static uint8_t       m_rx_buf[20]; */

bme680_params_t BME_params;
bme680_coefficients_t BME_coefs;

int32_t t_fine;

uint8_t chip_id;

#define I2C_BUFFER_LEN 8
#define SPI_BUFFER_LEN 5
#define BME680_BUS_READ_WRITE_ARRAY_INDEX   1
#define MPU_TWI_TIMEOUT                     100000

#define COMMAND_SIZE 9


//##########################################################################
//BASIC FUNCTIONS
//##########################################################################
void bme680_i2c_bus_write(uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, BME680_DEFAULT_ADDRESS, &reg_addr, 1, false));
    /*nrf_drv_twi_t const * p_instance,
                          uint8_t               address,
                          uint8_t const *       p_data,
                          uint8_t               length,
                          bool                  no_stop*/
}

void bme680_i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    uint32_t timeout = MPU_TWI_TIMEOUT;
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, BME680_DEFAULT_ADDRESS, &reg_addr, 1, false));
    while((!spi_xfer_done) && --timeout);

    spi_xfer_done = false;
    timeout = MPU_TWI_TIMEOUT;
    APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, BME680_DEFAULT_ADDRESS, reg_data, cnt));
}

void get_bme680_chipid(uint8_t *ret_data)
{
    bme680_i2c_bus_read(BME680_CHIP_ID_ADDRESS, ret_data, 1);
}
/*void writeByte(byte reg, byte value)
{ 
  
  Wire.beginTransmission(parameter.I2CAddress);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(); 
}
//##########################################################################
uint8_t readByte(byte reg)
{
  uint8_t value;  
  
  Wire.beginTransmission(parameter.I2CAddress);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(parameter.I2CAddress,1);   
  value = Wire.read();    
  return value; 
}


void bma250_i2c_bus_write(u8 reg_addr, u8 *reg_data, u8 cnt)
{

    u8 array[I2C_BUFFER_LEN];
    u8 stringpos = BMA2x2_INIT_VALUE;

    array[BMA2x2_INIT_VALUE] = reg_addr;
        for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) 
            array[stringpos + BME680_BUS_READ_WRITE_ARRAY_INDEX] =
            *(reg_data + stringpos);
 
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, BMA2x2_I2C_ADDR1,array, cnt+1, false));
}


void bma250_i2c_bus_read(u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 stringpos = BMA2x2_INIT_VALUE;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, BMA2x2_I2C_ADDR1,&reg_addr, 1, false));
    while((!spi_xfer_done) && --timeout);

    spi_xfer_done = false;
    timeout = MPU_TWI_TIMEOUT;
    APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi,BMA2x2_I2C_ADDR1, reg_data, cnt));
    while((!spi_xfer_done) && --timeout);
    for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++)
        *(reg_data + stringpos) = reg_data[stringpos];

}

*/