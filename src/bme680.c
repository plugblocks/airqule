#include <stdint.h>
#include <string.h>
#include <inttypes.h>
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


static float const_array1[16] = {1, 1, 1, 1, 1, 0.99, 1, 0.992, 1, 1, 0.998, 0.995, 1, 0.99, 1, 1};
static double const_array2[16] = {8000000.0, 4000000.0, 2000000.0, 1000000.0, 499500.4995, 248262.1648, 125000.0, 
63004.03226, 31281.28128, 15625.0, 7812.5, 3906.25, 1953.125, 976.5625, 488.28125, 244.140625};

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
    //APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, BME680_DEFAULT_ADDRESS, &reg_addr, 1, false));
    /*nrf_drv_twi_t const * p_instance,
                          uint8_t               address,
                          uint8_t const *       p_data,
                          uint8_t               length,
                          bool                  no_stop*/
    uint8_t array[I2C_BUFFER_LEN];
    uint8_t stringpos = 0;

    array[0] = reg_addr;
        for (stringpos = 0; stringpos < cnt; stringpos++) 
            array[stringpos + 1] =
            *(reg_data + stringpos);

    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, BME680_ADDRESS, array, cnt+1, false));
}

void bme680_i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    /*uint32_t timeout = MPU_TWI_TIMEOUT;
    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, BME680_DEFAULT_ADDRESS, &reg_addr, 1, false));
    while((!spi_xfer_done) && --timeout);

    spi_xfer_done = false;
    timeout = MPU_TWI_TIMEOUT;
    APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, BME680_DEFAULT_ADDRESS, reg_data, cnt));*/

    uint8_t stringpos = 0;
    uint32_t timeout = MPU_TWI_TIMEOUT;

    APP_ERROR_CHECK(nrf_drv_twi_tx(&m_twi, BME680_ADDRESS, &reg_addr, 1, false));
    while((!spi_xfer_done) && --timeout);

    spi_xfer_done = false;
    timeout = MPU_TWI_TIMEOUT;
    APP_ERROR_CHECK(nrf_drv_twi_rx(&m_twi, BME680_ADDRESS, reg_data, cnt));
    while((!spi_xfer_done) && --timeout);
    for (stringpos = 0; stringpos < cnt; stringpos++)
        *(reg_data + stringpos) = reg_data[stringpos];
}

void bme680_i2c_bus_write_one(uint8_t reg_addr, uint8_t reg_data)
{
    bme680_i2c_bus_write(reg_addr, reg_data, 1);
}

uint8_t bme680_i2c_bus_read_one(uint8_t reg_addr)
{
    uint8_t *reg_data;
    bme680_i2c_bus_read(reg_addr, reg_data, 1);
    return reg_data;
}

void get_bme680_chipid(uint8_t *ret_data)
{
    bme680_i2c_bus_read(BME680_CHIP_ID_ADDRESS, ret_data, 1);
}

void bme680_set_default_parameters(void)
{
    BME_params.I2CAddress = BME680_ADDRESS;

  //Now the device will be set to forced mode, this is the default setup
  //Please note that unlike the BME280, BME680 does not have a normal mode
  //0b01:     In forced mode a single measured is performed and the device returns automatically to sleep mode
  
    BME_params.sensorMode = 0b01;                   //Default sensor mode
                       
  //The IIR (Infinite Impulse Response) filter suppresses high frequency fluctuations
  //In short, a high factor value means less noise, but measurements are also less responsive
  //0b000:      factor 0 (filter off)
  //0b001:      factor 1,           0b010: factor 3,               0b011: factor 7
  //0b100:      factor 15 (default value)
  //0b101:      factor 31,          0b110: factor 63
  //0b111:      factor 127 (maximum value)

    BME_params.IIRfilter = 0b100;               //Setting IIR Filter coefficient to 15 (default)

  //Dfine the oversampling factor for measurements
  //Again, higher values mean less noise, but slower responses
  //If you don't want to measure values, set the oversampling to zero

  //0b000:      factor 0 (Disable hum, temp or pres measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)

    BME_params.humidOversampling = 0b101;         //Setting Humidity Oversampling to factor 16 (default) 

    BME_params.tempOversampling = 0b101;          //Setting Temperature Oversampling factor to 16 (default)
    
    BME_params.pressOversampling = 0b101;         //Setting Pressure Oversampling to factor 16 (default) 

    BME_params.pressureSeaLevel = 1013.25;        //default value of 1013.25 hPa

    BME_params.tempOutsideCelsius = 15;           //default value of 15°C

    BME_params.target_temp = 320;
}


bool bme680_init (void)
{
    bme680_set_default_parameters();

    bme680_i2c_bus_write(BME680_CTRL_GAS_1, 0, 1);
    writeIIRFilter();                           //set IIR Filter coefficient
    writeCTRLMeas();
    readCoefficients();                         //read coefficients from calibration registers
    
    //The following three functions enable gas sensor
    calculateHotPlateTime();                    //set heat up duration to 100 ms            
    calculateHotPlateRes();                     //convert target_temp to heater resistance (set on Arduino Sketch)
    setHotPlateProfile();                       //select heater profile to be used and set run_gas_I to 1 to enable gas measurement
    
    uint8_t *chipid;
    get_bme680_chipid(chipid);

    if (chipid = BME680_CHIP_ID) return true;

    return false;
}

//##########################################################################
//Here we read the coefficients from the calibration registers
//These functions were taken directly from the file "bme680.c" from Bosch (Revision 2.2.0 $ from 5 May 2017)

void readCoefficients(void)
{
    BME_coefs.dig_T1 = ((uint16_t)(bme680_i2c_bus_read_one(BME680_DIG_T1_MSB) << 8) + bme680_i2c_bus_read_one(BME680_DIG_T1_LSB));
    BME_coefs.dig_T2 = ((int16_t)(bme680_i2c_bus_read_one(BME680_DIG_T2_MSB) << 8) + bme680_i2c_bus_read_one(BME680_DIG_T2_LSB));
    BME_coefs.dig_T3 = ((int8_t)(bme680_i2c_bus_read_one(BME680_DIG_T3)));
    
    BME_coefs.dig_P1 = ((uint16_t)(bme680_i2c_bus_read_one(BME680_DIG_P1_MSB) << 8) + bme680_i2c_bus_read_one(BME680_DIG_P1_LSB));
    BME_coefs.dig_P2 = ((int16_t)(bme680_i2c_bus_read_one(BME680_DIG_P2_MSB) << 8) + bme680_i2c_bus_read_one(BME680_DIG_P2_LSB));
    BME_coefs.dig_P3 = ((int8_t)(bme680_i2c_bus_read_one(BME680_DIG_P3)));
    BME_coefs.dig_P4 = ((int16_t)(bme680_i2c_bus_read_one(BME680_DIG_P4_MSB) << 8) + bme680_i2c_bus_read_one(BME680_DIG_P4_LSB));
    BME_coefs.dig_P5 = ((int16_t)(bme680_i2c_bus_read_one(BME680_DIG_P5_MSB) << 8) + bme680_i2c_bus_read_one(BME680_DIG_P5_LSB));
    BME_coefs.dig_P6 = ((int8_t)(bme680_i2c_bus_read_one(BME680_DIG_P6)));
    BME_coefs.dig_P7 = ((int8_t)(bme680_i2c_bus_read_one(BME680_DIG_P7)));
    BME_coefs.dig_P8 = ((int16_t)(bme680_i2c_bus_read_one(BME680_DIG_P8_MSB) << 8) + bme680_i2c_bus_read_one(BME680_DIG_P8_LSB));
    BME_coefs.dig_P9 = ((int16_t)(bme680_i2c_bus_read_one(BME680_DIG_P9_MSB) << 8) + bme680_i2c_bus_read_one(BME680_DIG_P9_LSB));
    BME_coefs.dig_P10 = ((uint8_t)(bme680_i2c_bus_read_one(BME680_DIG_P10)));
    
    BME_coefs.dig_H1 = (uint16_t)(((uint16_t)(bme680_i2c_bus_read_one(BME680_DIG_H1_MSB)) << 4) + ((uint8_t)(bme680_i2c_bus_read_one(BME680_DIG_H1_LSB)) & BME680_BIT_MASK_H1_DATA));
    BME_coefs.dig_H2 = (uint16_t)(((uint16_t)(bme680_i2c_bus_read_one(BME680_DIG_H2_MSB)) << 4) + (((uint8_t)(bme680_i2c_bus_read_one(BME680_DIG_H2_LSB))) >> 4));
    BME_coefs.dig_H3 = (int8_t)(bme680_i2c_bus_read_one(BME680_DIG_H3));
    BME_coefs.dig_H4 = (int8_t)(bme680_i2c_bus_read_one(BME680_DIG_H4));
    BME_coefs.dig_H5 = (int8_t)(bme680_i2c_bus_read_one(BME680_DIG_H5));
    BME_coefs.dig_H6 = (uint8_t)(bme680_i2c_bus_read_one(BME680_DIG_H6));
    BME_coefs.dig_H7 = (int8_t)(bme680_i2c_bus_read_one(BME680_DIG_H7));

//The following calibration parameters are used to convert the target temperature of the hot plate to a register value
//This register value is written to the registers res_heat_x<7:0> (see Datasheet, "Gas sensor heating and measurement", pages 18 and 19) 
    
    BME_coefs.par_g1 = (int8_t)(bme680_i2c_bus_read_one(BME680_DIG_G1)); 
    BME_coefs.par_g2 = (int16_t)(bme680_i2c_bus_read_one(BME680_DIG_G2_MSB) << 8) + bme680_i2c_bus_read_one(BME680_DIG_G2_LSB); 
    BME_coefs.par_g3 = (int8_t)(bme680_i2c_bus_read_one(BME680_DIG_G3));
    
    BME_coefs.res_heat_range = (int8_t)(bme680_i2c_bus_read_one(BME680_RES_HEAT_RG));
    BME_coefs.res_heat_val   = (int8_t)(bme680_i2c_bus_read_one(BME680_RES_HEAT_VL));    
}

//##########################################################################
//We set up the IIR Filter through bits 4, 3 and 2 from Config Register (0x75)]
//The other bits from this register won't be used in this program and remain 0
//Please refer to the BME680 Datasheet for more information (page 28)


void writeIIRFilter(void)
{   
    uint8_t value;
    value = (BME_params.IIRfilter << 2) & 0b00011100;
    bme680_i2c_bus_write_one(BME680_CONFIG, value);
}
//##########################################################################
//Here we set the oversampling for humidity measurements by writting to the register BME680_CTRL_HUM 
//We also set the oversampling factors for measuring temperature and pressure by writting to the register BME680_CTRL_MEAS
//For that, refer to the Datasheet, "Temperature, pressure and relative humidity control registers", pages 27 and 28)
//Finally, here we define the sensor mode by writting to the register BME680_CTRL_MEAS (bits 1 and 0)
//Please note that setting the sensor mode to "01" (Forced Mode) starts a measurement
//After a single measurement the sensor will return to sensor mode "00" (Sleep Mode)
//Therefore we must set the sensor mode to Forced Mode on the Loop function 


void writeCTRLMeas(void)
{
    uint8_t value;
    value = BME_params.humidOversampling & 0b00000111;
    bme680_i2c_bus_write_one(BME680_CTRL_HUM, value);
    
    value = (BME_params.tempOversampling << 5) & 0b11100000;
    value |= (BME_params.pressOversampling << 2) & 0b00011100;
    value |= BME_params.sensorMode & 0b00000011;
    bme680_i2c_bus_write_one(BME680_CTRL_MEAS, value);     
}
//##########################################################################
//DATA READOUT FUNCTIONS
//##########################################################################
//Reading the temperature value from the BME680 is identical to reading the temperature from the BME280
//If the oversampling factor is set to 0, the temperature measurement is disabled
//This function returns the temperature in Celsius


float readTempC(void)
{   
    if (BME_params.tempOversampling == 0b000)                    //disabling the temperature measurement function
    {
        return 0;
    }
    
    else
    {
        int32_t adc_T;
        adc_T = (uint32_t)bme680_i2c_bus_read_one(BME680_TEMPERATURE_MSB) << 12;
        adc_T |= (uint32_t)bme680_i2c_bus_read_one(BME680_TEMPERATURE_LSB) << 4;
        adc_T |= (bme680_i2c_bus_read_one(BME680_TEMPERATURE_XLSB) >> 4) & 0b00001111;
        
        int64_t var1, var2;
        
        var1 = ((((adc_T>>3) - ((int32_t)BME_coefs.dig_T1<<1))) * ((int32_t)BME_coefs.dig_T2)) >> 11;
        var2 = (((((adc_T>>4) - ((int32_t)BME_coefs.dig_T1)) * ((adc_T>>4) - ((int32_t)BME_coefs.dig_T1))) >> 12) *
        ((int32_t)BME_coefs.dig_T3)) >> 14;
        t_fine = var1 + var2;
        float T = (t_fine * 5 + 128) >> 8;
        T = T / 100;
        return T;
    }
}

//##########################################################################
//This function also reads the temperature from BME680 and converts the resulting value from Celsius to Fahrenheit


float readTempF(void)
{   
    if (BME_params.tempOversampling == 0b000)                    //disabling the temperature measurement function
    {
        return 0;
    }
    
    else
    {
        int32_t adc_T;
        adc_T = (uint32_t)bme680_i2c_bus_read_one(BME680_TEMPERATURE_MSB) << 12;
        adc_T |= (uint32_t)bme680_i2c_bus_read_one(BME680_TEMPERATURE_LSB) << 4;
        adc_T |= (bme680_i2c_bus_read_one(BME680_TEMPERATURE_XLSB) >> 4) & 0b00001111;
        
        int64_t var1, var2;
        
        var1 = ((((adc_T>>3) - ((int32_t)BME_coefs.dig_T1<<1))) * ((int32_t)BME_coefs.dig_T2)) >> 11;
        var2 = (((((adc_T>>4) - ((int32_t)BME_coefs.dig_T1)) * ((adc_T>>4) - ((int32_t)BME_coefs.dig_T1))) >> 12) *
        ((int32_t)BME_coefs.dig_T3)) >> 14;
        t_fine = var1 + var2;
        float T = (t_fine * 5 + 128) >> 8;
        T = T / 100;
        T = (T * 1.8) + 32;
        return T;
    }
}
//##########################################################################
//Temperature in Kelvin is needed for the conversion of pressure to altitude    
//Both parameters "tempOutsideCelsius" and "tempOutsideFahrenheit" are set to 999 as default (see .h file)
//If the user chooses to read temperature in Celsius, "tempOutsideFahrenheit" remains 999
//If the user chooses to read temperature in Fahrenheit instead, "tempOutsideCelsius" remains 999
//If both values are used, then the temperature in Celsius will be used for the conversion
//If none of them are used, then the default value of 288.15 will be used (i.e. 273.15 + 15)


float convertTempKelvin(void)
{
    float tempOutsideKelvin;    
    
    if (BME_params.tempOutsideCelsius != 999 & BME_params.tempOutsideFahrenheit == 999 )   
    {
        tempOutsideKelvin = BME_params.tempOutsideCelsius;
        tempOutsideKelvin = tempOutsideKelvin + 273.15;
        return tempOutsideKelvin;       
    }
    
    if (BME_params.tempOutsideCelsius != 999 & BME_params.tempOutsideFahrenheit != 999 )   
    {
        tempOutsideKelvin = BME_params.tempOutsideCelsius;
        tempOutsideKelvin = tempOutsideKelvin + 273.15;
        return tempOutsideKelvin;       
    }
    
    if (BME_params.tempOutsideFahrenheit != 999 & BME_params.tempOutsideCelsius == 999)
    {
        
        tempOutsideKelvin = (BME_params.tempOutsideFahrenheit - 32);
        tempOutsideKelvin = tempOutsideKelvin * 5;
        tempOutsideKelvin = tempOutsideKelvin / 9;
        tempOutsideKelvin = tempOutsideKelvin + 273.15;
        return tempOutsideKelvin;   
    }
    
    if (BME_params.tempOutsideFahrenheit == 999 & BME_params.tempOutsideCelsius == 999)
    {
        tempOutsideKelvin = 273.15 + 15;
        return tempOutsideKelvin; 
    }
    
    tempOutsideKelvin = 273.15 + 15;
    return tempOutsideKelvin;

}
//##########################################################################
//Reading the pressure from the BME680 is NOT the same as reading the pressure from the BME280 sensor
//The following functions were taken from the file "bme680_calculations.c" from Bosch Sensortec (Revision 2.2.0 $ from 5 May 2017)
//Note that I chose to let the variable "var4" out of the calculation (see commentary below)


float readPressure(void)
{
    if (BME_params.pressOversampling == 0b000)                       //disabling the pressure measurement function
    {
        return 0;
    }
    
    else
    {       
        readTempC();        
        
        uint32_t adc_P;
        adc_P = (uint32_t)bme680_i2c_bus_read_one(BME680_PRESSURE_MSB) << 12;
        adc_P |= (uint32_t)bme680_i2c_bus_read_one(BME680_PRESSURE_LSB) << 4;
        adc_P |= (bme680_i2c_bus_read_one(BME680_PRESSURE_XLSB) >> 4) & 0b00001111;        
        
        int32_t var1 = 0;
        int32_t var2 = 0;
        int32_t var3 = 0;
        //int32_t var4 = 0;
        int32_t P = 0;
        
        var1 = (((int32_t)t_fine) >> 1) - 64000;
        
        var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)BME_coefs.dig_P6) >> 2;
        var2 = var2 + ((var1 * (int32_t)BME_coefs.dig_P5) << 1);
        var2 = (var2 >> 2) + ((int32_t)BME_coefs.dig_P4 << 16);
        
        var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)BME_coefs.dig_P3 << 5)) >> 3) + (((int32_t)BME_coefs.dig_P2 * var1) >> 1);
        var1 = var1 >> 18;
        var1 = ((32768 + var1) * (int32_t)BME_coefs.dig_P1) >> 15;
        
        P = 1048576 - adc_P;
        P = (int32_t)((P - (var2 >> 12)) * ((uint32_t)3125));       
    
//This is the original calculation taken from the file "bme680_calculations.c"
//I chose to change part of the calculation, because the values I was getting were way off
    
        /*
        var4 = (1 << 31);
        if (P >= var4)
            P = ((P / (uint32_t)var1) << 1);                
        else
            P = ((P << 1) / (uint32_t)var1);
        */
        
//See what I did here? This corresponds to the "else" situation

        P = ((P << 1) / (uint32_t)var1);        
        
        var1 = ((int32_t)BME_coefs.dig_P9 * (int32_t)(((P >> 3) * (P >> 3)) >> 13)) >> 12;        
        var2 = ((int32_t)(P >> 2) * (int32_t)BME_coefs.dig_P8) >> 13;     
        var3 = ((int32_t)(P >> 8) * (int32_t)(P >> 8) * (int32_t)(P >> 8) * (int32_t)BME_coefs.dig_P10) >> 17;        
        
        P = (int32_t)(P) + ((var1 + var2 + var3 + ((int32_t)BME_coefs.dig_P7 << 7)) >> 4);
        
        return (float)P/100;        
    }
}
//##########################################################################
//Reading the humidity values from the BME680 is DIFFERENT from reading the humidity value from the BME280 sensor
//The following functions were taken from the file "bme680_calculations.c" from Bosch Sensortec (Revision 2.2.0 $ from 5 May 2017)


float readHumidity(void)
{
    if (BME_params.humidOversampling == 0b000)                   //disabling the humitidy measurement function
    {
        return 0;
    }
    
    else
    {
        int32_t adc_H;
        adc_H = (uint32_t)bme680_i2c_bus_read_one(BME680_HUMIDITY_MSB) << 8;
        adc_H |= (uint32_t)bme680_i2c_bus_read_one(BME680_HUMIDITY_LSB);       
        
        int32_t temp_scaled = 0;
        int32_t var1 = 0;
        int32_t var2 = 0;
        int32_t var3 = 0;
        int32_t var4 = 0;
        int32_t var5 = 0;
        int32_t var6 = 0;
        int32_t H    = 0;
        
        temp_scaled = ((((int32_t)t_fine) * 5) + 128) >> 8;     
    
        var1 = (int32_t)adc_H - ((int32_t)((int32_t)BME_coefs.dig_H1 << 4)) - 
               ((temp_scaled * (int32_t)BME_coefs.dig_H3/(int32_t)100) >> 1);
               
        
        var2 = ((int32_t)BME_coefs.dig_H2 * (((temp_scaled * (int32_t)BME_coefs.dig_H4)/(int32_t)100) + 
                (((temp_scaled * ((temp_scaled * (int32_t)BME_coefs.dig_H5)/((int32_t)100))) >> 6)/((int32_t)100)) + 
                (int32_t)(1 << 14))) >> 10;
                
        var3 = var1 * var2;
        
        var4 = ((((int32_t)BME_coefs.dig_H6) << 7) + ((temp_scaled * (int32_t)BME_coefs.dig_H7)/((int32_t)100))) >> 4;
        
        var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
        var6 = (var4 * var5) >> 1;
        
        H = (var3 + var6) >> 12;
        
//This part avoids returning humidity values above 100% and below 0%, which wouldn't make sense     
        if (H > 102400)
            H = 102400;
        else if (H < 0)
            H = 0;
        
        return H/1024.;     
    }
}
//##########################################################################
//To calculate the altitude (in meters) from the pressure (in hPa) I've used the international Barometric Formula
//I've used the formula from the German Wikipedia Page https://de.wikipedia.org/wiki/Barometrische_H%C3%B6henformel
//On the website the formula sets the outside temperature at 15°C and the pressure at sea level at 1013.25hPa as constant values
//For more precise measurements, change this constants to actual values from your location


float readAltitudeMeter(void)
{
    float heightOutput = 0;
    float tempOutsideKelvin = convertTempKelvin();
    
    heightOutput = readPressure();
    heightOutput = (heightOutput/BME_params.pressureSeaLevel);
    heightOutput = pow(heightOutput, 0.190284);
    heightOutput = 1 - heightOutput;    
    heightOutput = heightOutput * tempOutsideKelvin;
    heightOutput = heightOutput / 0.0065;
    return heightOutput;        
}
//##########################################################################
//This is the same formula from before, but it converts the altitude from meters to feet before returning the results


float readAltitudeFeet(void)
{   
    float heightOutput = 0;
    float tempOutsideKelvin = convertTempKelvin();
    
    heightOutput = readPressure();
    heightOutput = (heightOutput/BME_params.pressureSeaLevel);
    heightOutput = pow(heightOutput, 0.190284);
    heightOutput = 1 - heightOutput;
    heightOutput = heightOutput * tempOutsideKelvin;
    heightOutput = heightOutput / 0.0065;
    heightOutput = heightOutput / 0.3048;
    return heightOutput;    
}


//##########################################################################
//DATA READOUT FUNCTIONS - GAS SENSOR
//##########################################################################
//1st step
//First we write the target temperature for the hot plate as defined in BME_params.target_temp
//This function uses the target_temp value to calculate the resistance of the hot plate
//The BME680 will use this resistance to heat up the gas sensor hot plate
//According to the datasheet the sensor needs a target temperature between 200°C and 400°C to work properly
void calculateHotPlateRes(void)
{
    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    
    
    var1 = ((double)BME_coefs.par_g1 / 16.0) + 49.0;
    var2 = (((double)BME_coefs.par_g2 / 32768.0) * 0.0005) + 0.00235;
    var3 = (double)BME_coefs.par_g3 / 1024.0;
    var4 = var1 * (1.0 + (var2 * (double)BME_params.target_temp));
    var5 = var4 + (var3 * (double)readTempC());
    
    BME_coefs.res_heat_0 = (uint8_t)(3.4 * ((var5 * (4.0 / (4.0 + (double)BME_coefs.res_heat_range)) *
                                        (1.0 / (1.0 + ((double)BME_coefs.res_heat_val * 0.002)))) - 25));
    
    bme680_i2c_bus_write_one(BME680_RES_HEAT_0, (uint8_t)BME_coefs.res_heat_0); 
    
}

//##########################################################################
//2nd step
//Defining how long the hot plate should keep the target temperature
//I've arbitrary chosen a fixed duration of 148 ms for all measurements
//If you wish to change the duration, please refer to the datasheet (page 30, chapter 5.3.3.3)
void calculateHotPlateTime(void)
{
    int8_t heat_time_0 = 0b01100101;                    //148 ms heating duration
    bme680_i2c_bus_write_one(BME680_GAS_WAIT_0, heat_time_0); 
}

//##########################################################################
//3rd step
//Sending the heating duration and the target resistance (calculated from the target temperature) to the sensor
//This is done by writing to the register BME680_CTRL_GAS_1 (register 071h)
//Finally by writing the bit 4 of this register to logic 1 we enable the gas sensor measurement
void setHotPlateProfile(void)
{
    uint8_t value;
    value = (((uint8_t)BME_params.hotplate_profile) & 0b00001111) | 0b00010000;
    bme680_i2c_bus_write_one(BME680_CTRL_GAS_1, value);
}

bool readStatus(void)
{
    uint8_t newDataBit = ((bme680_i2c_bus_read_one(0x1D) & 0b10000000) >> 7);      
    
  if (newDataBit == 1) 
  {                                                  
    uint8_t gasValidBit = ((bme680_i2c_bus_read_one(0x2B) & 0b00100000) >> 5);
    uint8_t heaterStabilityBit = ((bme680_i2c_bus_read_one(0x2B) & 0b00010000) >> 4);

    if ((gasValidBit == 1) & (heaterStabilityBit == 1))
    {
        
        return 1;
    }

    else
    {
      return 0;
    }
    
  }

  else
  {
    return 0;
  }

}

float readGas(void)
{
    
    for (int8_t i = 0; i = 10; i++)
    {
        if (readStatus() == 1)
        {
            int16_t gas_r;  
            gas_r = (uint16_t)bme680_i2c_bus_read_one(BME680_GAS_MSB) << 2;
            gas_r |= (uint16_t)((uint16_t)bme680_i2c_bus_read_one(BME680_GAS_LSB) >> 6);   
    
            int8_t gas_range;
            gas_range = (uint8_t)bme680_i2c_bus_read_one(BME680_GAS_LSB) & 0b00001111;
    
            double var1;
            float gas_res;
    
            var1 = (1340.0 + 5.0 * BME_coefs.range_switching_error) * const_array1[gas_range];
            gas_res = var1 * const_array2[gas_range] / (gas_r - 512.0 + var1);  

            return gas_res;
        }
        
        nrf_delay_ms(20);
    }
    
    return 0;   
    
}

int16_t convertoFloatToInt16(float value, long min, long max) {
  float conversionFactor = (float) (INT16_MAX) / (float)(max - min);
  return (int16_t)(value * conversionFactor);
}

uint16_t convertoFloatToUInt16(float value, long min, long max) {
  float conversionFactor = (float) (UINT16_MAX) / (float)(max - min);
  return (uint16_t)(value * conversionFactor);
}

void bme680_measure_sigfox(uint8_t *res)
{
    int16_t *tmp;
    tmp[0] = convertoFloatToInt16(readTempC(), -30, 50);
    tmp[1] = convertoFloatToUInt16(readHumidity(), 0, 100);
    tmp[2] = convertoFloatToUInt16((readPressure()/100)-900, 900, 1100);
    tmp[3] = convertoFloatToUInt16(readGas(), 0, 200);

    res[0] = tmp[0] >> 8;
    res[1] = tmp[0];
    res[2] = tmp[1] >> 8;
    res[3] = tmp[1];
    res[4] = tmp[2] >> 8;
    res[5] = tmp[2];
    res[6] = tmp[3] >> 8;
    res[7] = tmp[3];
}
