/***************************************************************************
  This is a library for the BMP280 pressure sensor

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include "BMP280.h"

//#include "Arduino.h"
#include <i2c_t3.h>
#include <SPI.h>


/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/


BMP280::BMP280()							//called with no argument: I2C
  : _cs(-1), _mosi(-1), _miso(-1), _sck(-1)
{ }

BMP280::BMP280(int8_t cspin)				//called with pin number for CS: SPI
  : _cs(cspin), _mosi(-1), _miso(-1), _sck(-1) //default Pin for CS is 10
{ }

BMP280::BMP280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin)
  : _cs(cspin), _mosi(mosipin), _miso(misopin), _sck(sckpin)
{ }


bool BMP280::begin(uint8_t a) {
  _i2caddr = a;

  if (_cs == -1) {
    // i2c
    Wire.begin();
  } else {
    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);

    if (_sck == -1) {
      // hardware SPI

     SPI.setSCK(BMP_SCK); //redefine SCK pin
     SPI.begin();
    } else {
      // software SPI
      pinMode(_sck, OUTPUT);
      pinMode(_mosi, OUTPUT);
      pinMode(_miso, INPUT);
    }
  }
  read8(BMP280_REGISTER_CHIPID); //needed to clear / wakeup BMP280
  if (read8(BMP280_REGISTER_CHIPID) != 0x58)

	 return false;

  readCoefficients();
  write8(BMP280_REGISTER_CONTROL, 0x3F);
  return true;
}

uint8_t BMP280::spixfer(uint8_t x) {
  if (_sck == -1)
    return SPI.transfer(x);

  // software spi
  //Serial.println("Software SPI");
  uint8_t reply = 0;
  for (int i=7; i>=0; i--) {
    reply <<= 1;
    digitalWrite(_sck, LOW);
    digitalWrite(_mosi, x & (1<<i));
    digitalWrite(_sck, HIGH);
    if (digitalRead(_miso))
      reply |= 1;
  }
  return reply;
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C/SPI
*/
/**************************************************************************/
void BMP280::write8(byte reg, byte value)
{
  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg & ~0x80); // write, bit 7 low
    spixfer(value);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t BMP280::read8(byte reg)
{
  uint8_t value;

  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, (byte)1);
    value = Wire.read();

  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }
  return value;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C
*/
/**************************************************************************/
uint16_t BMP280::read16(byte reg)
{
  uint16_t value;

  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, (byte)2);
    value = (Wire.read() << 8) | Wire.read();

  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = (spixfer(0) << 8) | spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }

  return value;
}

uint16_t BMP280::read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);

}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C
*/
/**************************************************************************/
int16_t BMP280::readS16(byte reg)
{
  return (int16_t)read16(reg);

}

int16_t BMP280::readS16_LE(byte reg)
{
  return (int16_t)read16_LE(reg);

}


/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C
*/
/**************************************************************************/

uint32_t BMP280::read24(byte reg)
{
  uint32_t value;

  if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, (byte)3);
    
    value = Wire.read();
    value <<= 8;
    value |= Wire.read();
    value <<= 8;
    value |= Wire.read();

  } else {
    if (_sck == -1)
      SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    
    value = spixfer(0);
    value <<= 8;
    value |= spixfer(0);
    value <<= 8;
    value |= spixfer(0);

    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      SPI.endTransaction();              // release the SPI bus
  }

  return value;
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void BMP280::readCoefficients(void)
{
    _bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
    _bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
    _bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

    _bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
    _bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
    _bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
    _bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
    _bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
    _bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
    _bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
    _bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
    _bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
float BMP280::readTemperature(void)
{
  int32_t var1, var2;

  int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
  adc_T >>= 4;

  var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
	   ((int32_t)_bmp280_calib.dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
	     ((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
	   ((int32_t)_bmp280_calib.dig_T3)) >> 14;

  t_fine = var1 + var2;

  float T  = (t_fine * 5 + 128) >> 8;
  return T/100;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
float BMP280::readPressure(void) {
  int64_t var1, var2, p;
  // Must be done first to get the t_fine variable set up
  readTemperature();

  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
    ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
  return (float)p/256;
}
uint8_t BMP280::readPressureTemperature(float* press, float* temp) {

	int64_t var1, var2, p;


	  // Must be done first to get the t_fine variable set up
	  *temp = readTemperature() + 273.15;

	  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
	  adc_P >>= 4;

	  var1 = ((int64_t)t_fine) - 128000;
	  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
	  var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
	  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
	  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
	    ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
	  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;

	  if (var1 == 0) {
	    return 0;  // avoid exception caused by division by zero
	  }
	  p = 1048576 - adc_P;
	  p = (((p<<31) - var2)*3125) / var1;
	  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
	  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

	  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
	  *press = p/256.0;
	  return 1;
}

float BMP280::readAltitude(float seaLevelhPa) {
  float altitude;

  float pressure = readPressure(); // in Si units for Pascal
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return altitude;
}
/*!
 *	@brief This API is used to get
 *	the temperature oversampling setting in the register 0xF4
 *	bits from 5 to 7
 *
 *        value             | Temperature oversampling
 *  ------------------------|------------------------------
 *       0x00               |  BMP280_OVERSAMP_SKIPPED
 *       0x01               |  BMP280_OVERSAMP_1X
 *       0x02               |  BMP280_OVERSAMP_2X
 *       0x03               |  BMP280_OVERSAMP_4X
 *       0x04               |  BMP280_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP280_OVERSAMP_16X
 *
 *
 *
 *
 *
 *
 *  @return oversamp_temperature value
 *
 *
 *
 */
void BMP280::get_oversamp_temperature(uint8_t *oversamp_temp)
{

	uint8_t v_data_u8 = 0;
	/* read temperature over sampling*/
	v_data_u8 = read8(BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG);
	*oversamp_temp = BMP280_GET_BITSLICE(v_data_u8,
				BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE);
	_oversamp_temperature = *oversamp_temp;
}
/*!
 *	@brief This API is used to set
 *	the temperature oversampling setting in the register 0xF4
 *	bits from 5 to 7
 *
 *        value             | Temperature oversampling
 *  ------------------------|------------------------------
 *       0x00               |  BMP280_OVERSAMP_SKIPPED
 *       0x01               |  BMP280_OVERSAMP_1X
 *       0x02               |  BMP280_OVERSAMP_2X
 *       0x03               |  BMP280_OVERSAMP_4X
 *       0x04               |  BMP280_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP280_OVERSAMP_16X
 *
 *
 *  @param v_value_u8 :The value of temperature over sampling
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMP280::set_oversamp_temperature(uint8_t v_value_u8)
{
	uint8_t v_data_u8 = 0;
	//read current register data
	v_data_u8 = read8(BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG);
	/* write over sampling*/
	v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
	BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE,v_value_u8);
	write8(BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,v_data_u8);
}
/*!
 *	@brief This API is used to get
 *	the pressure oversampling setting in the register 0xF4
 *	bits from 2 to 4
 *
 *        value             | Pressure oversampling
 *  ------------------------|------------------------------
 *       0x00               |  BMP280_OVERSAMP_SKIPPED
 *       0x01               |  BMP280_OVERSAMP_1X
 *       0x02               |  BMP280_OVERSAMP_2X
 *       0x03               |  BMP280_OVERSAMP_4X
 *       0x04               |  BMP280_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP280_OVERSAMP_16X
 *
 *
 *  @param  v_value_u8 : The value of pressure over sampling
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMP280::get_oversamp_pressure(uint8_t *oversamp_press)
{
	uint8_t v_data_u8 = 0;
	/* read pressure over sampling*/
	v_data_u8 = read8(BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG);
	*oversamp_press = BMP280_GET_BITSLICE(v_data_u8,
			BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE);
	_oversamp_pressure = *oversamp_press;
}
/*!
 *	@brief This API is used to set
 *	the pressure oversampling setting in the register 0xF4
 *	bits from 2 to 4
 *
 *        value             | Pressure oversampling
 *  ------------------------|------------------------------
 *       0x00               |  BMP280_OVERSAMP_SKIPPED
 *       0x01               |  BMP280_OVERSAMP_1X
 *       0x02               |  BMP280_OVERSAMP_2X
 *       0x03               |  BMP280_OVERSAMP_4X
 *       0x04               |  BMP280_OVERSAMP_8X
 *       0x05,0x06 and 0x07 |  BMP280_OVERSAMP_16X
 *
 *
 *  @param  v_value_u8 : The value of pressure over sampling
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMP280::set_oversamp_pressure(uint8_t v_value_u8)
{

	uint8_t v_data_u8 = 0;
	//read current register data
	v_data_u8 = read8(BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG);
	/* write over sampling*/
	v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
	BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE,v_value_u8);
	write8(BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,v_data_u8);
}
/*!
 *	@brief This API used to get the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	@param v_power_mode_u8 : The value of power mode value
 *  value            |   Power mode
 * ------------------|------------------
 *	0x00             | BMP280_SLEEP_MODE
 *	0x01 and 0x02    | BMP280_FORCED_MODE
 *	0x03             | BMP280_NORMAL_MODE
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMP280::get_power_mode(uint8_t *power_mode)
{
	uint8_t v_data_u8 = 0;
	/* read power mode */
	v_data_u8 = read8(BMP280_CTRL_MEAS_REG_POWER_MODE__REG);
	*power_mode = BMP280_GET_BITSLICE(v_data_u8,
				BMP280_CTRL_MEAS_REG_POWER_MODE);
}
/*!
 *	@brief This API used to set the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	@param v_power_mode_u8 : The value of power mode value
 *  value            |   Power mode
 * ------------------|------------------
 *	0x00             | BMP280_SLEEP_MODE
 *	0x01 and 0x02    | BMP280_FORCED_MODE
 *	0x03             | BMP280_NORMAL_MODE
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMP280::set_power_mode(uint8_t v_power_mode_u8)
{

	uint8_t v_mode_u8 = 0;

		if (v_power_mode_u8 <= BMP280_NORMAL_MODE) {
			/* write the power mode*/
			v_mode_u8 = (_oversamp_temperature
					<< 5)
					+ (_oversamp_pressure
					<< 2)
					+ v_power_mode_u8;
			write8(BMP280_CTRL_MEAS_REG_POWER_MODE__REG,
					v_mode_u8);
		}
}
/*!
 * @brief Used to reset the sensor
 * The value 0xB6 is written to the 0xE0 register
 * the device is reset using the
 * complete power-on-reset procedure.
 * Soft reset can be easily set using bmp280_set_softreset().
 *
 * @note Usage Hint : bmp280_set_softreset()
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMP280::set_soft_rst(void)
{
	uint8_t v_data_u8 = BMP280_SOFT_RESET_CODE;

		/* write soft reset */
	write8(BMP280_REGISTER_SOFTRESET, v_data_u8);
}
/*!
 *	@brief This API used to get the sensor
 *	SPI mode(communication type) in the register 0xF5 bit 0
 *
 *
 *
 *	@param v_enable_disable_u8 : The spi3 enable or disable state
 *    value    | Description
 *  -----------|---------------
 *     0       | Disable
 *     1       | Enable
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMP280::get_spi3(uint8_t *v_enable_disable_u8)
{

	uint8_t v_data_u8 = 0;
	v_data_u8 = read8(BMP280_CONFIG_REG_SPI3_ENABLE__REG);
	*v_enable_disable_u8 = BMP280_GET_BITSLICE(v_data_u8,
				BMP280_CONFIG_REG_SPI3_ENABLE);

}
/*!
 *	@brief This API used to set the sensor
 *	SPI mode(communication type) in the register 0xF5 bit 0
 *
 *
 *
 *	@param v_enable_disable_u8 : The spi3 enable or disable state
 *    value    | Description
 *  -----------|---------------
 *     0       | Disable
 *     1       | Enable
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMP280::set_spi3(uint8_t v_enable_disable_u8)
{

	uint8_t v_data_u8 = 0;
	v_data_u8 = read8(BMP280_CONFIG_REG_SPI3_ENABLE__REG);
	v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
					BMP280_CONFIG_REG_SPI3_ENABLE,
					v_enable_disable_u8);
	write8(BMP280_CONFIG_REG_SPI3_ENABLE__REG,v_data_u8);

}
/*!
 *	@brief This API is used to reads filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	@param v_value_u8 : The value of filter coefficient
 *	value	    |	Filter coefficient
 * -------------|-------------------------
 *	0x00        | BMP280_FILTER_COEFF_OFF
 *	0x01        | BMP280_FILTER_COEFF_2
 *	0x02        | BMP280_FILTER_COEFF_4
 *	0x03        | BMP280_FILTER_COEFF_8
 *	0x04        | BMP280_FILTER_COEFF_16
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMP280::get_filter(uint8_t *v_value_u8)
{
	uint8_t v_data_u8 = 0;
	/* read filter*/
	v_data_u8 = read8(BMP280_CONFIG_REG_FILTER__REG);
	*v_value_u8 = BMP280_GET_BITSLICE(v_data_u8,
				BMP280_CONFIG_REG_FILTER);

}
/*!
 *	@brief This API is used to write filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	@param v_value_u8 : The value of filter coefficient
 *	value	    |	Filter coefficient
 * -------------|-------------------------
 *	0x00        | BMP280_FILTER_COEFF_OFF
 *	0x01        | BMP280_FILTER_COEFF_2
 *	0x02        | BMP280_FILTER_COEFF_4
 *	0x03        | BMP280_FILTER_COEFF_8
 *	0x04        | BMP280_FILTER_COEFF_16
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMP280::set_filter(uint8_t v_value_u8)
{
	uint8_t v_data_u8 = 0;
	/* write filter*/
	v_data_u8 = read8(BMP280_CONFIG_REG_FILTER__REG);
	v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
					BMP280_CONFIG_REG_FILTER, v_value_u8);
	write8(BMP280_CONFIG_REG_FILTER__REG,v_data_u8);
}
/*!
 *	@brief This API used to Read the
 *	standby duration time from the sensor in the register 0xF5 bit 5 to 7
 *
 *	@param v_standby_durn_u8 : The standby duration time value.
 *  value     |  standby duration
 * -----------|--------------------
 *    0x00    | BMP280_STANDBYTIME_1_MS
 *    0x01    | BMP280_STANDBYTIME_63_MS
 *    0x02    | BMP280_STANDBYTIME_125_MS
 *    0x03    | BMP280_STANDBYTIME_250_MS
 *    0x04    | BMP280_STANDBYTIME_500_MS
 *    0x05    | BMP280_STANDBYTIME_1000_MS
 *    0x06    | BMP280_STANDBYTIME_2000_MS
 *    0x07    | BMP280_STANDBYTIME_4000_MS
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMP280::get_standby_durn(uint8_t *v_standby_durn_u8)
{
	uint8_t v_data_u8 = 0;
	/* read the standby duration*/
	v_data_u8 = read8(BMP280_CONFIG_REG_STANDBY_DURN__REG);
	*v_standby_durn_u8 = BMP280_GET_BITSLICE(v_data_u8,
				BMP280_CONFIG_REG_STANDBY_DURN);
}
/*!
 *	@brief This API used to Read the
 *	standby duration time from the sensor
 *	in the register 0xF5 bit 5 to 7
 *	@note Normal mode comprises an
 *	automated perpetual cycling between an (active)
 *	Measurement period and an (inactive) standby period.
 *	@note The standby time is determined
 *	by the contents of the register t_sb.
 *	Standby time can be set using BMP280_STANDBYTIME_125_MS.
 *
 *	@note bmp280_set_standby_durn(BMP280_STANDBYTIME_125_MS)
 *
 *
 *
 *	@param v_standby_durn_u8 : The standby duration time value.
 *  value     |  standby duration
 * -----------|--------------------
 *    0x00    | BMP280_STANDBYTIME_1_MS
 *    0x01    | BMP280_STANDBYTIME_63_MS
 *    0x02    | BMP280_STANDBYTIME_125_MS
 *    0x03    | BMP280_STANDBYTIME_250_MS
 *    0x04    | BMP280_STANDBYTIME_500_MS
 *    0x05    | BMP280_STANDBYTIME_1000_MS
 *    0x06    | BMP280_STANDBYTIME_2000_MS
 *    0x07    | BMP280_STANDBYTIME_4000_MS
 *
 *
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMP280::set_standby_durn(uint8_t v_standby_durn_u8)
{

	uint8_t v_data_u8 = 0;
	/* write the standby duration*/
	v_data_u8 = read8(BMP280_CONFIG_REG_STANDBY_DURN__REG);
	v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
					BMP280_CONFIG_REG_STANDBY_DURN,
					v_standby_durn_u8);
	write8(BMP280_CONFIG_REG_STANDBY_DURN__REG,v_data_u8);
}
/*!
 *	@brief This API is used to write
 *	 the working mode of the sensor
 *
 *
 *  @param v_work_mode_u8 : The value of work mode
 *   value      |  mode
 * -------------|-------------
 *    0         | BMP280_ULTRA_LOW_POWER_MODE
 *    1         | BMP280_LOW_POWER_MODE
 *    2         | BMP280_STANDARD_RESOLUTION_MODE
 *    3         | BMP280_HIGH_RESOLUTION_MODE
 *    4         | BMP280_ULTRA_HIGH_RESOLUTION_MODE
 *
 *  @return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMP280::set_work_mode(uint8_t v_work_mode_u8)
{
	uint8_t v_data_u8 = 0;

	if (v_work_mode_u8 <= BMP280_ULTRA_HIGH_RESOLUTION_MODE) {
		v_data_u8 = read8(BMP280_REGISTER_CONTROL);
		switch (v_work_mode_u8) {
			/* write work mode*/
			case BMP280_ULTRA_LOW_POWER_MODE:
				_oversamp_temperature =
				BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE;
				_oversamp_pressure =
					BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE;
				break;
			case BMP280_LOW_POWER_MODE:
				_oversamp_temperature =
					BMP280_LOWPOWER_OVERSAMP_TEMPERATURE;
				_oversamp_pressure =
					BMP280_LOWPOWER_OVERSAMP_PRESSURE;
				break;
			case BMP280_STANDARD_RESOLUTION_MODE:
				_oversamp_temperature =
				BMP280_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE;
				_oversamp_pressure =
				BMP280_STANDARDRESOLUTION_OVERSAMP_PRESSURE;
				break;
			case BMP280_HIGH_RESOLUTION_MODE:
				_oversamp_temperature =
				BMP280_HIGHRESOLUTION_OVERSAMP_TEMPERATURE;
				_oversamp_pressure =
				BMP280_HIGHRESOLUTION_OVERSAMP_PRESSURE;
				break;
			case BMP280_ULTRA_HIGH_RESOLUTION_MODE:
				_oversamp_temperature =
				BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE;
				_oversamp_pressure =
				BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE;
				break;
		}
		v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
			BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE,
				_oversamp_temperature);
		v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
			BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE,
				_oversamp_pressure);
		write8(BMP280_REGISTER_CONTROL,v_data_u8);
	}
}
