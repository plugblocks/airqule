#ifndef BME680_H_
#define BME680_H_

#define BME680_DEFAULT_ADDRESS                (0x77)

#define BME680_CHIP_ID_ADDRESS          0xD0
#define BME680_CTRL_HUM       0x72
#define BME680_CTRL_MEAS      0x74    
#define BME680_CONFIG       0x75
#define BME680_TEMPERATURE_MSB    0x22
#define BME680_TEMPERATURE_LSB    0x23
#define BME680_TEMPERATURE_XLSB   0x24
#define BME680_PRESSURE_MSB     0x1F
#define BME680_PRESSURE_LSB     0x20
#define BME680_PRESSURE_XLSB    0x21
#define BME680_HUMIDITY_MSB     0x25
#define BME680_HUMIDITY_LSB     0x26
#define BME680_GAS_MSB        0x2A
#define BME680_GAS_LSB        0x2B

#define BME680_BIT_MASK_H1_DATA   0x0F


typedef enum
{
  BME680_DIG_T1_LSB =   0xE9,
  BME680_DIG_T1_MSB =   0xEA,
  BME680_DIG_T2_LSB =   0x8A,
  BME680_DIG_T2_MSB =   0x8B,
  BME680_DIG_T3   =   0x8C,
  
  BME680_DIG_P1_LSB =   0x8E,
  BME680_DIG_P1_MSB =   0x8F,
  BME680_DIG_P2_LSB =   0x90,
  BME680_DIG_P2_MSB =   0x91,
  BME680_DIG_P3   =   0x92,
  BME680_DIG_P4_LSB =   0x94,
  BME680_DIG_P4_MSB =   0x95,
  BME680_DIG_P5_LSB =   0x96,
  BME680_DIG_P5_MSB =   0x97, 
  BME680_DIG_P6   =   0x99,
  BME680_DIG_P7   =   0x98,
  BME680_DIG_P8_LSB =   0x9C,
  BME680_DIG_P8_MSB =   0x9D,
  BME680_DIG_P9_LSB =   0x9E,
  BME680_DIG_P9_MSB =   0x9F,
  BME680_DIG_P10    =   0xA0,
  
  BME680_DIG_H1_LSB   =   0xE2,
  BME680_DIG_H1_MSB =   0xE3,
  BME680_DIG_H2_LSB =   0xE2,
  BME680_DIG_H2_MSB =   0xE1,
  BME680_DIG_H3   =   0xE4,
  BME680_DIG_H4   =   0xE5,
  BME680_DIG_H5   =   0xE6,
  BME680_DIG_H6   =   0xE7,
  BME680_DIG_H7   =   0xE8,
  
  BME680_DIG_G1   =   0xED,
  BME680_DIG_G2_LSB =   0xEB,
  BME680_DIG_G2_MSB =     0xEC,
  BME680_DIG_G3   =   0xEE,
  
  BME680_RES_HEAT_RG  =   0x02,
  BME680_RES_HEAT_VL  =   0x00,
  
  BME680_RES_HEAT_0 =   0x5A,
  BME680_GAS_WAIT_0   =     0x64,
  
  BME680_CTRL_GAS_1 =   0x71,
} bme680_coefficients_vals_t;


typedef struct
{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int8_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int8_t   dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int8_t   dig_P6;
    int8_t   dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_P10;

    uint16_t dig_H1;
    uint16_t dig_H2;
    int8_t   dig_H3;
    int8_t   dig_H4;
    int8_t   dig_H5;
    uint8_t  dig_H6;
    int8_t   dig_H7;

    int8_t   par_g1;
    int16_t  par_g2;
    int8_t   par_g3;

    int8_t   res_heat_range;
    int8_t   res_heat_val;

    uint8_t  res_heat_0;  
    uint8_t  gas_wait_0;

    int8_t   gas_range;
    int8_t   range_switching_error;
} bme680_coefficients_t;
  
typedef struct
{
    uint8_t communication;
    uint8_t I2CAddress;
    uint8_t sensorMode;
    uint8_t IIRfilter;
    uint8_t tempOversampling;
    uint8_t pressOversampling;
    uint8_t humidOversampling;
    uint16_t pressureSeaLevel;
    int16_t tempOutsideCelsius;
    int16_t tempOutsideFahrenheit;
    int16_t target_temp;
    int16_t amb_temp;
    uint8_t hotplate_profile;
} bme680_params_t;

void bme680_i2c_bus_write(uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
void bme680_i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
void get_bme680_chipid(uint8_t *ret_data);

#endif