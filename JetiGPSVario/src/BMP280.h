/***************************************************************************
  This is a library for the BMP280 pressure sensor

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __BMP280_H__
#define __BMP280_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

//#include <Adafruit_Sensor.h>
#define BMP280_SPI
#ifdef __AVR_ATtiny85__
 #include "TinyWireM.h"
 #define Wire TinyWireM
#else
	#ifndef BMP280_SPI
		#include <Wire.h>
	#endif
#endif
/* Alternate pin for SPI SCK (pin 13 is LED!)
 *
 */
#define BMP_SCK 14
/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define BMP280_ADDRESS                (0x77)
/*=========================================================================*/
#define BMP280_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define BMP280_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/************************************************/
/**\name	POWER MODE DEFINITION       */
/***********************************************/
/* Sensor Specific constants */
#define BMP280_SLEEP_MODE                    (0x00)
#define BMP280_FORCED_MODE                   (0x01)
#define BMP280_NORMAL_MODE                   (0x03)
#define BMP280_SOFT_RESET_CODE               (0xB6)
/************************************************/
/**\name	STANDBY TIME DEFINITION       */
/***********************************************/
#define BMP280_STANDBY_TIME_1_MS              (0x00)
#define BMP280_STANDBY_TIME_63_MS             (0x01)
#define BMP280_STANDBY_TIME_125_MS            (0x02)
#define BMP280_STANDBY_TIME_250_MS            (0x03)
#define BMP280_STANDBY_TIME_500_MS            (0x04)
#define BMP280_STANDBY_TIME_1000_MS           (0x05)
#define BMP280_STANDBY_TIME_2000_MS           (0x06)
#define BMP280_STANDBY_TIME_4000_MS           (0x07)
/************************************************/
/**\name	OVERSAMPLING DEFINITION       */
/***********************************************/
#define BMP280_OVERSAMP_SKIPPED          (0x00)
#define BMP280_OVERSAMP_1X               (0x01)
#define BMP280_OVERSAMP_2X               (0x02)
#define BMP280_OVERSAMP_4X               (0x03)
#define BMP280_OVERSAMP_8X               (0x04)
#define BMP280_OVERSAMP_16X              (0x05)
/************************************************/
/**\name	WORKING MODE DEFINITION       */
/***********************************************/
#define BMP280_ULTRA_LOW_POWER_MODE          (0x00)
#define BMP280_LOW_POWER_MODE	             (0x01)
#define BMP280_STANDARD_RESOLUTION_MODE      (0x02)
#define BMP280_HIGH_RESOLUTION_MODE          (0x03)
#define BMP280_ULTRA_HIGH_RESOLUTION_MODE    (0x04)

#define BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE          BMP280_OVERSAMP_1X
#define BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE       BMP280_OVERSAMP_1X

#define BMP280_LOWPOWER_OVERSAMP_PRESSURE	         BMP280_OVERSAMP_2X
#define BMP280_LOWPOWER_OVERSAMP_TEMPERATURE	         BMP280_OVERSAMP_1X

#define BMP280_STANDARDRESOLUTION_OVERSAMP_PRESSURE     BMP280_OVERSAMP_4X
#define BMP280_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE  BMP280_OVERSAMP_1X

#define BMP280_HIGHRESOLUTION_OVERSAMP_PRESSURE         BMP280_OVERSAMP_8X
#define BMP280_HIGHRESOLUTION_OVERSAMP_TEMPERATURE      BMP280_OVERSAMP_1X

#define BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE       BMP280_OVERSAMP_16X
#define BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE    BMP280_OVERSAMP_2X
/************************************************/
/**\name	FILTER DEFINITION       */
/***********************************************/
#define BMP280_FILTER_COEFF_OFF               (0x00)
#define BMP280_FILTER_COEFF_2                 (0x01)
#define BMP280_FILTER_COEFF_4                 (0x02)
#define BMP280_FILTER_COEFF_8                 (0x03)
#define BMP280_FILTER_COEFF_16                (0x04)
/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    enum
    {
      BMP280_REGISTER_DIG_T1              = 0x88,
      BMP280_REGISTER_DIG_T2              = 0x8A,
      BMP280_REGISTER_DIG_T3              = 0x8C,

      BMP280_REGISTER_DIG_P1              = 0x8E,
      BMP280_REGISTER_DIG_P2              = 0x90,
      BMP280_REGISTER_DIG_P3              = 0x92,
      BMP280_REGISTER_DIG_P4              = 0x94,
      BMP280_REGISTER_DIG_P5              = 0x96,
      BMP280_REGISTER_DIG_P6              = 0x98,
      BMP280_REGISTER_DIG_P7              = 0x9A,
      BMP280_REGISTER_DIG_P8              = 0x9C,
      BMP280_REGISTER_DIG_P9              = 0x9E,

      BMP280_REGISTER_CHIPID             = 0xD0,
      BMP280_REGISTER_VERSION            = 0xD1,
      BMP280_REGISTER_SOFTRESET          = 0xE0,

      BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0
	  BMP280_REGISTER_STATUS			 = 0xF3,
      BMP280_REGISTER_CONTROL            = 0xF4,
      BMP280_REGISTER_CONFIG             = 0xF5,
      BMP280_REGISTER_PRESSUREDATA       = 0xF7,
      BMP280_REGISTER_TEMPDATA           = 0xFA,
    };

/*=========================================================================*/
  /**\name	BIT LENGTH,POSITION AND MASK DEFINITION      */
  /***********************************************/
  /* Status Register */
  #define BMP280_STATUS_REG_MEASURING__POS           (3)
  #define BMP280_STATUS_REG_MEASURING__MSK           (0x08)
  #define BMP280_STATUS_REG_MEASURING__LEN           (1)
  #define BMP280_STATUS_REG_MEASURING__REG           (BMP280_REGISTER_STATUS)

  #define BMP280_STATUS_REG_IM_UPDATE__POS            (0)
  #define BMP280_STATUS_REG_IM_UPDATE__MSK            (0x01)
  #define BMP280_STATUS_REG_IM_UPDATE__LEN            (1)
  #define BMP280_STATUS_REG_IM_UPDATE__REG           (BMP280_REGISTER_STATUS)
  /************************************************/
  /**\name	BIT LENGTH,POSITION AND MASK DEFINITION
  FOR TEMPERATURE OVERSAMPLING */
  /***********************************************/
  /* Control Measurement Register */
  #define BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS             (5)
  #define BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK             (0xE0)
  #define BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__LEN             (3)
  #define BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG             \
  (BMP280_REGISTER_CONTROL)
  /************************************************/
  /**\name	BIT LENGTH,POSITION AND MASK DEFINITION
  FOR PRESSURE OVERSAMPLING */
  /***********************************************/
  #define BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS             (2)
  #define BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK             (0x1C)
  #define BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__LEN             (3)
  #define BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG             \
  (BMP280_REGISTER_CONTROL)
  /************************************************/
  /**\name	BIT LENGTH,POSITION AND MASK DEFINITION
  FOR POWER MODE */
  /***********************************************/
  #define BMP280_CTRL_MEAS_REG_POWER_MODE__POS              (0)
  #define BMP280_CTRL_MEAS_REG_POWER_MODE__MSK              (0x03)
  #define BMP280_CTRL_MEAS_REG_POWER_MODE__LEN              (2)
  #define BMP280_CTRL_MEAS_REG_POWER_MODE__REG             (BMP280_REGISTER_CONTROL)
  /************************************************/
  /**\name	BIT LENGTH,POSITION AND MASK DEFINITION
  FOR STANDBY DURATION */
  /***********************************************/
  /* Configuration Register */
  #define BMP280_CONFIG_REG_STANDBY_DURN__POS                 (5)
  #define BMP280_CONFIG_REG_STANDBY_DURN__MSK                 (0xE0)
  #define BMP280_CONFIG_REG_STANDBY_DURN__LEN                 (3)
  #define BMP280_CONFIG_REG_STANDBY_DURN__REG                 (BMP280_REGISTER_CONFIG)
  /************************************************/
  /**\name	BIT LENGTH,POSITION AND MASK DEFINITION
  FOR IIR FILTER */
  /***********************************************/
  #define BMP280_CONFIG_REG_FILTER__POS              (2)
  #define BMP280_CONFIG_REG_FILTER__MSK              (0x1C)
  #define BMP280_CONFIG_REG_FILTER__LEN              (3)
  #define BMP280_CONFIG_REG_FILTER__REG              (BMP280_REGISTER_CONFIG)
  /************************************************/
  /**\name	BIT LENGTH,POSITION AND MASK DEFINITION
  FOR SPI ENABLE*/
  /***********************************************/
  #define BMP280_CONFIG_REG_SPI3_ENABLE__POS             (0)
  #define BMP280_CONFIG_REG_SPI3_ENABLE__MSK             (0x01)
  #define BMP280_CONFIG_REG_SPI3_ENABLE__LEN             (1)
  #define BMP280_CONFIG_REG_SPI3_ENABLE__REG             (BMP280_REGISTER_CONFIG)

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct
    {
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;

      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;
    } bmp280_calib_data;
/*=========================================================================*/

/*
class Adafruit_BMP280_Unified : public Adafruit_Sensor
{
  public:
    Adafruit_BMP280_Unified(int32_t sensorID = -1);

    bool  begin(uint8_t addr = BMP280_ADDRESS);
    void  getTemperature(float *temp);
    void  getPressure(float *pressure);
    float pressureToAltitude(float seaLevel, float atmospheric, float temp);
    float seaLevelForAltitude(float altitude, float atmospheric, float temp);
    void  getEvent(sensors_event_t*);
    void  getSensor(sensor_t*);

  private:
    uint8_t   _i2c_addr;
    int32_t   _sensorID;
};

*/

class BMP280
{
  public:
    BMP280(void);
    BMP280(int8_t cspin);
    BMP280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);

    bool  begin(uint8_t addr = BMP280_ADDRESS);
    float readTemperature(void);
    float readPressure(void);
    float readAltitude(float seaLevelhPa = 1013.25);
    uint8_t readPressureTemperature(float* press, float* temp);
    void  get_oversamp_temperature(uint8_t *oversamp_temp);
    void  set_oversamp_temperature(uint8_t v_value_u8);
    void  get_oversamp_pressure(uint8_t *oversamp_press);
    void  set_oversamp_pressure(uint8_t v_value_u8);
    void  get_power_mode(uint8_t *power_mode);
    void  set_power_mode(uint8_t v_power_mode_u8);
    void  set_soft_rst(void);
    void  get_spi3(uint8_t *v_enable_disable_u8);
    void  set_spi3(uint8_t v_enable_disable_u8);
    void  get_filter(uint8_t *v_value_u8);
    void  set_filter(uint8_t v_value_u8);
    void  get_standby_durn(uint8_t *v_standby_durn_u8);
    void  set_standby_durn(uint8_t v_standby_durn_u8);
    void  set_work_mode(uint8_t v_work_mode_u8);


  private:

    void readCoefficients(void);
    uint8_t spixfer(uint8_t x);

    void      write8(byte reg, byte value);
    uint8_t   read8(byte reg);
    uint16_t  read16(byte reg);
    uint32_t  read24(byte reg);
    int16_t   readS16(byte reg);
    uint16_t  read16_LE(byte reg); // little endian
    int16_t   readS16_LE(byte reg); // little endian

    uint8_t   _i2caddr;
    int32_t   _sensorID;
    int32_t t_fine;
    uint8_t	 _oversamp_temperature;/**< temperature over sampling*/
	uint8_t   _oversamp_pressure;/**< pressure over sampling*/

    int8_t _cs, _mosi, _miso, _sck;

    bmp280_calib_data _bmp280_calib;

};

#endif
