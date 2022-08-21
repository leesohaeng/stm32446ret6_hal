/*
 * htu21d_i2c.h
 *
 *  Created on: 2017. 10. 4.
 *      Author: sohaenglee
 */

#ifndef HTU21D_I2C_H_
#define HTU21D_I2C_H_


#define HTDU21D_ADDRESS 				0x80	/* Shifted 8-bit I2C address for the sensor */

#define WRITE_USER_REG  				0xE6	/* Write user register*/
#define READ_USER_REG  					0xE7	/* Read user register*/
#define SOFT_RESET  					0xFE	/* Soft Reset (takes 15ms). Switch sensor OFF & ON again. All registers set to default */

#define TEMP_COEFFICIENT 				-0.15 	/* Temperature coefficient (from 0deg.C to 80deg.C) */
#define CRC8_POLYNOMINAL 				0x13100 /* CRC8 polynomial for 16bit CRC8 x^8 + x^5 + x^4 + 1 */


/* Enumarations */
typedef enum{
	HTU21_SensorResolution_RH12_TEMP14	= 0x00,	/* RH: 12Bit, measuring time 16ms. Temperature: 14Bit, measuring time 50ms (dafault on Power ON) */
	HTU21_SensorResolution_RH8_TEMP12	= 0x01,	/* RH: 8Bit, measuring time 8ms. Temperature: 12Bit, measuring time 25ms */
	HTU21_SensorResolution_RH10_TEMP13	= 0x80,	/* RH: 10Bit, measuring time 5ms. Temperature: 13Bit, measuring time 13ms. */
	HTU21_SensorResolution_RH11_TEMP11	= 0x81	/* RH: 11Bit, measuring time 3ms. Temperature: 11Bit, measuring time 7ms. */
}HTU21SensorResolution_TypeDef;

typedef enum{
	HTU21_HeaterSwitch_On	= 0x04,
	HTU21_HeaterSwitch_Off	= 0xFB
}HTU21HeaterSwitch_TypeDef;

typedef enum{
	HTU21_HumidityMeasurementMode_Hold		= 0xE5,		/* Trigger Humidity Measurement. Hold master (SCK line is blocked) */
	HTU21_HumidityMeasurementMode_NoHold	= 0xF5		/* Trigger Humidity Measurement. No Hold master (allows other I2C communication on a bus while sensor is measuring) */
}HTU21HumidityMeasurementMode_TypeDef;

typedef enum{
	HTU21_TemperatureMeasurementMode_Hold	= 0xE3,		/* Trigger Temperature Measurement. Hold master (SCK line is blocked) */
	HTU21_TemperatureMeasurementMode_NoHold	= 0xF3		/* Trigger Temperature Measurement. No Hold master (allows other I2C communication on a bus while sensor is measuring) */
}HTU21TemperatureMeasurementMode_TypeDef;

typedef enum{
	HTU21_BatteryStatus_EndOfBattery	= 0x01,			/* VDD < 2.25V */
	HTU21_BatteryStatus_BatteryOk		= 0x00			/* VDD > 2.25V */
}HTU21_BatteryStatus_TypeDef;


#endif /* HTU21D_I2C_H_ */

uint8_t htu21_checkCRC8(uint16_t data);


uint8_t htu21_checkCRC8(uint16_t data){
	uint8_t bit;

	for (bit = 0; bit < 16; bit++)
	{
		if (data & 0x8000)
		{
			data = (data << 1) ^ CRC8_POLYNOMINAL;
		}
		else
		{
			data <<= 1;
		}
	}
	data >>= 8;

	return data;
}
