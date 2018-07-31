#ifndef BME680_H_
#define BME680_H_

#define BME680_ADDRESS                BME680_I2C_ADDR_SECONDARY

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


/** BME680 General config */
#define BME680_POLL_PERIOD_MS       UINT8_C(10)

/** BME680 I2C addresses */
#define BME680_I2C_ADDR_PRIMARY     UINT8_C(0x76)
#define BME680_I2C_ADDR_SECONDARY   UINT8_C(0x77)

/** BME680 unique chip identifier */
#define BME680_CHIP_ID  UINT8_C(0x61)

/** BME680 coefficients related defines */
#define BME680_COEFF_SIZE       UINT8_C(41)
#define BME680_COEFF_ADDR1_LEN      UINT8_C(25)
#define BME680_COEFF_ADDR2_LEN      UINT8_C(16)

/** BME680 field_x related defines */
#define BME680_FIELD_LENGTH     UINT8_C(15)
#define BME680_FIELD_ADDR_OFFSET    UINT8_C(17)

/** Soft reset command */
#define BME680_SOFT_RESET_CMD   UINT8_C(0xb6)

/** Error code definitions */
#define BME680_OK       INT8_C(0)
/* Errors */
#define BME680_E_NULL_PTR           INT8_C(-1)
#define BME680_E_COM_FAIL           INT8_C(-2)
#define BME680_E_DEV_NOT_FOUND      INT8_C(-3)
#define BME680_E_INVALID_LENGTH     INT8_C(-4)

/* Warnings */
#define BME680_W_DEFINE_PWR_MODE    INT8_C(1)
#define BME680_W_NO_NEW_DATA        INT8_C(2)

/* Info's */
#define BME680_I_MIN_CORRECTION     UINT8_C(1)
#define BME680_I_MAX_CORRECTION     UINT8_C(2)

/** Register map */
/** Other coefficient's address */
#define BME680_ADDR_RES_HEAT_VAL_ADDR   UINT8_C(0x00)
#define BME680_ADDR_RES_HEAT_RANGE_ADDR UINT8_C(0x02)
#define BME680_ADDR_RANGE_SW_ERR_ADDR   UINT8_C(0x04)
#define BME680_ADDR_SENS_CONF_START UINT8_C(0x5A)
#define BME680_ADDR_GAS_CONF_START  UINT8_C(0x64)

/** Field settings */
#define BME680_FIELD0_ADDR      UINT8_C(0x1d)

/** Heater settings */
#define BME680_RES_HEAT0_ADDR       UINT8_C(0x5a)
#define BME680_GAS_WAIT0_ADDR       UINT8_C(0x64)

/** Sensor configuration registers */
#define BME680_CONF_HEAT_CTRL_ADDR      UINT8_C(0x70)
#define BME680_CONF_ODR_RUN_GAS_NBC_ADDR    UINT8_C(0x71)
#define BME680_CONF_OS_H_ADDR           UINT8_C(0x72)
#define BME680_MEM_PAGE_ADDR            UINT8_C(0xf3)
#define BME680_CONF_T_P_MODE_ADDR       UINT8_C(0x74)
#define BME680_CONF_ODR_FILT_ADDR       UINT8_C(0x75)

/** Coefficient's address */
#define BME680_COEFF_ADDR1  UINT8_C(0x89)
#define BME680_COEFF_ADDR2  UINT8_C(0xe1)

/** Chip identifier */
#define BME680_CHIP_ID_ADDR UINT8_C(0xd0)

/** Soft reset register */
#define BME680_SOFT_RESET_ADDR      UINT8_C(0xe0)

/** Heater control settings */
#define BME680_ENABLE_HEATER        UINT8_C(0x00)
#define BME680_DISABLE_HEATER       UINT8_C(0x08)

/** Gas measurement settings */
#define BME680_DISABLE_GAS_MEAS     UINT8_C(0x00)
#define BME680_ENABLE_GAS_MEAS      UINT8_C(0x01)

/** Over-sampling settings */
#define BME680_OS_NONE      UINT8_C(0)
#define BME680_OS_1X        UINT8_C(1)
#define BME680_OS_2X        UINT8_C(2)
#define BME680_OS_4X        UINT8_C(3)
#define BME680_OS_8X        UINT8_C(4)
#define BME680_OS_16X       UINT8_C(5)

/** IIR filter settings */
#define BME680_FILTER_SIZE_0    UINT8_C(0)
#define BME680_FILTER_SIZE_1    UINT8_C(1)
#define BME680_FILTER_SIZE_3    UINT8_C(2)
#define BME680_FILTER_SIZE_7    UINT8_C(3)
#define BME680_FILTER_SIZE_15   UINT8_C(4)
#define BME680_FILTER_SIZE_31   UINT8_C(5)
#define BME680_FILTER_SIZE_63   UINT8_C(6)
#define BME680_FILTER_SIZE_127  UINT8_C(7)

/** Power mode settings */
#define BME680_SLEEP_MODE   UINT8_C(0)
#define BME680_FORCED_MODE  UINT8_C(1)

/** Delay related macro declaration */
#define BME680_RESET_PERIOD UINT32_C(10)

/** SPI memory page settings */
#define BME680_MEM_PAGE0    UINT8_C(0x10)
#define BME680_MEM_PAGE1    UINT8_C(0x00)

/** Ambient humidity shift value for compensation */
#define BME680_HUM_REG_SHIFT_VAL    UINT8_C(4)

/** Run gas enable and disable settings */
#define BME680_RUN_GAS_DISABLE  UINT8_C(0)
#define BME680_RUN_GAS_ENABLE   UINT8_C(1)

/** Buffer length macro declaration */
#define BME680_TMP_BUFFER_LENGTH    UINT8_C(40)
#define BME680_REG_BUFFER_LENGTH    UINT8_C(6)
#define BME680_FIELD_DATA_LENGTH    UINT8_C(3)
#define BME680_GAS_REG_BUF_LENGTH   UINT8_C(20)

/** Settings selector */
#define BME680_OST_SEL          UINT16_C(1)
#define BME680_OSP_SEL          UINT16_C(2)
#define BME680_OSH_SEL          UINT16_C(4)
#define BME680_GAS_MEAS_SEL     UINT16_C(8)
#define BME680_FILTER_SEL       UINT16_C(16)
#define BME680_HCNTRL_SEL       UINT16_C(32)
#define BME680_RUN_GAS_SEL      UINT16_C(64)
#define BME680_NBCONV_SEL       UINT16_C(128)
#define BME680_GAS_SENSOR_SEL       (BME680_GAS_MEAS_SEL | BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)

/** Number of conversion settings*/
#define BME680_NBCONV_MIN       UINT8_C(0)
#define BME680_NBCONV_MAX       UINT8_C(10)

/** Mask definitions */
#define BME680_GAS_MEAS_MSK UINT8_C(0x30)
#define BME680_NBCONV_MSK   UINT8_C(0X0F)
#define BME680_FILTER_MSK   UINT8_C(0X1C)
#define BME680_OST_MSK      UINT8_C(0XE0)
#define BME680_OSP_MSK      UINT8_C(0X1C)
#define BME680_OSH_MSK      UINT8_C(0X07)
#define BME680_HCTRL_MSK    UINT8_C(0x08)
#define BME680_RUN_GAS_MSK  UINT8_C(0x10)
#define BME680_MODE_MSK     UINT8_C(0x03)
#define BME680_RHRANGE_MSK  UINT8_C(0x30)
#define BME680_RSERROR_MSK  UINT8_C(0xf0)
#define BME680_NEW_DATA_MSK UINT8_C(0x80)
#define BME680_GAS_INDEX_MSK    UINT8_C(0x0f)
#define BME680_GAS_RANGE_MSK    UINT8_C(0x0f)
#define BME680_GASM_VALID_MSK   UINT8_C(0x20)
#define BME680_HEAT_STAB_MSK    UINT8_C(0x10)
#define BME680_MEM_PAGE_MSK UINT8_C(0x10)
#define BME680_SPI_RD_MSK   UINT8_C(0x80)
#define BME680_SPI_WR_MSK   UINT8_C(0x7f)
#define BME680_BIT_H1_DATA_MSK  UINT8_C(0x0F)

/** Bit position definitions for sensor settings */
#define BME680_GAS_MEAS_POS UINT8_C(4)
#define BME680_FILTER_POS   UINT8_C(2)
#define BME680_OST_POS      UINT8_C(5)
#define BME680_OSP_POS      UINT8_C(2)
#define BME680_RUN_GAS_POS  UINT8_C(4)

/** Array Index to Field data mapping for Calibration Data*/
#define BME680_T2_LSB_REG   (1)
#define BME680_T2_MSB_REG   (2)
#define BME680_T3_REG       (3)
#define BME680_P1_LSB_REG   (5)
#define BME680_P1_MSB_REG   (6)
#define BME680_P2_LSB_REG   (7)
#define BME680_P2_MSB_REG   (8)
#define BME680_P3_REG       (9)
#define BME680_P4_LSB_REG   (11)
#define BME680_P4_MSB_REG   (12)
#define BME680_P5_LSB_REG   (13)
#define BME680_P5_MSB_REG   (14)
#define BME680_P7_REG       (15)
#define BME680_P6_REG       (16)
#define BME680_P8_LSB_REG   (19)
#define BME680_P8_MSB_REG   (20)
#define BME680_P9_LSB_REG   (21)
#define BME680_P9_MSB_REG   (22)
#define BME680_P10_REG      (23)
#define BME680_H2_MSB_REG   (25)
#define BME680_H2_LSB_REG   (26)
#define BME680_H1_LSB_REG   (26)
#define BME680_H1_MSB_REG   (27)
#define BME680_H3_REG       (28)
#define BME680_H4_REG       (29)
#define BME680_H5_REG       (30)
#define BME680_H6_REG       (31)
#define BME680_H7_REG       (32)
#define BME680_T1_LSB_REG   (33)
#define BME680_T1_MSB_REG   (34)
#define BME680_GH2_LSB_REG  (35)
#define BME680_GH2_MSB_REG  (36)
#define BME680_GH1_REG      (37)
#define BME680_GH3_REG      (38)

/** BME680 register buffer index settings*/
#define BME680_REG_FILTER_INDEX     UINT8_C(5)
#define BME680_REG_TEMP_INDEX       UINT8_C(4)
#define BME680_REG_PRES_INDEX       UINT8_C(4)
#define BME680_REG_HUM_INDEX        UINT8_C(2)
#define BME680_REG_NBCONV_INDEX     UINT8_C(1)
#define BME680_REG_RUN_GAS_INDEX    UINT8_C(1)
#define BME680_REG_HCTRL_INDEX      UINT8_C(0)


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
void bme680_i2c_bus_write_one(uint8_t reg_addr, uint8_t reg_data);
uint8_t bme680_i2c_bus_read_one(uint8_t reg_addr);
void get_bme680_chipid(uint8_t *ret_data);
void bme680_set_default_parameters(void);
bool bme680_init(void);
void readCoefficients(void);
void writeIIRFilter(void);
void writeCTRLMeas(void);
float readTempC(void);
float readTempF(void);
float convertTempKelvin(void);
float readPressure(void);
float readHumidity(void);
float readAltitudeMeter(void);
float readAltitudeFeet(void);
void calculateHotPlateRes(void);
void calculateHotPlateTime(void);
void setHotPlateProfile(void);
bool readStatus(void);
float readGas(void);


#endif