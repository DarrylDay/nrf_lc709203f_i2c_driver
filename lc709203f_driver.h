/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LC709203F_NRF_I2C_Driver_H
#define LC709203F_NRF_I2C_Driver_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "nrf_drv_twi.h"

/* Common Structures ---------------------------------------------------------*/

#ifndef __SHARED__TYPES
#define __SHARED__TYPES

typedef enum
{
  MEMS_SUCCESS      =   0x01,
  MEMS_ERROR        =   0x00
} status_t;

#endif

/* LC709203F Structures ------------------------------------------------------*/

typedef enum
{
  LC709203F_CURRENT_DIRECTION_AUTO        = 0x0000,
  LC709203F_CURRENT_DIRECTION_CHARGE      = 0x0001,
  LC709203F_CURRENT_DIRECTION_DISCHARGE   = 0xFFFF,
} LC709203F_CURRENT_DIRECTION_t;

typedef enum
{
  LC709203F_CHANGE_OF_PARAM_TYPE_1 = 0x0000,
  LC709203F_CHANGE_OF_PARAM_TYPE_2 = 0x0001,
} LC709203F_CHANGE_OF_PARAM_t;

typedef enum
{
  LC709203F_IC_POWER_MODE_OPERATIONAL = 0x0001,
  LC709203F_IC_POWER_MODE_SLEEP       = 0x0002,
} LC709203F_IC_POWER_MODE_t;

typedef enum
{
  LC709203F_TEMP_MODE_I2C         = 0x0000,
  LC709203F_TEMP_MODE_THERMISTOR  = 0x0001,
} LC709203F_TEMP_MODE_t;

typedef struct
{
	uint16_t thermistorB;
  LC709203F_CURRENT_DIRECTION_t currentDirection;
	uint16_t apa;
	uint16_t apt;
	LC709203F_CHANGE_OF_PARAM_t batteryProfile;
	uint16_t alarmLowRSOC;
	uint16_t alarmLowVoltage;
	LC709203F_IC_POWER_MODE_t powerMode;
	LC709203F_TEMP_MODE_t temperatureMode;

} lc709203f_config_t;

/***************** PARAM VALUES ******************/

#define LC709203F_ADDR_7BIT             0x0B
#define LC709203F_ADDR_8BIT             0x16
#define LC709203F_CRC_POLYNOMIAL        0x07
#define LC709203F_PARAM_INITIAL_RSOC    0xAA55

/***************** PIN VALUES ******************/

#define LC709203F_ALARM_PIN           16

/***************** REGISTER VALUES ******************/

#define LC709203F_REG_BEFORE_RSOC		            0x04
#define LC709203F_REG_THERMISTOR_B		          0x06
#define LC709203F_REG_INITIAL_RSOC		          0x07
#define LC709203F_REG_TEMPERATURE		            0x08
#define LC709203F_REG_VOLTAGE		                0x09
#define LC709203F_REG_CURRENT_DIRECTION		      0x0A
#define LC709203F_REG_ADJUSTMENT_PACK_APPLI	    0x0B
#define LC709203F_REG_ADJUSTMENT_PACK_THERM	    0x0C
#define LC709203F_REG_RSOC			                0x0D
#define LC709203F_REG_INDICATOR_TO_EMPTY	      0x0F
#define LC709203F_REG_IC_VERSION		            0x11
#define LC709203F_REG_CHANGE_OF_THE_PARAM	      0x12
#define LC709203F_REG_ALARM_LOW_CELL_RSOC	      0x13
#define LC709203F_REG_ALARM_LOW_CELL_VOLT	      0x14
#define LC709203F_REG_IC_POWER_MODE		          0x15
#define LC709203F_REG_STATUS_BIT		            0x16
#define LC709203F_REG_NUM_OF_THE_PARAM	        0x1A

/************** General Functions *******************/

nrf_drv_twi_config_t LC709203F_GET_I2C_CONFIG();
lc709203f_config_t LC709203F_GET_DEFAULT_CONFIG();
status_t LC709203F_INIT(void *handle, lc709203f_config_t config, bool runInitialRSOC);

/************** Register Functions *******************/

/*******************************************************************************
* Register      : BEFORE_RSOC
* Address       : 0x04
* Permission    : W
*******************************************************************************/
status_t LC709203F_CALL_BEFORE_RSOC(void *handle);

/*******************************************************************************
* Register      : THERMISTOR_B
* Address       : 0x06
* Permission    : RW
*******************************************************************************/
status_t LC709203F_GET_THERMISTOR_B(void *handle, uint16_t *word);
status_t LC709203F_SET_THERMISTOR_B(void *handle, uint16_t word, bool checkResult);

/*******************************************************************************
* Register      : INITIAL_RSOC
* Address       : 0x07
* Permission    : W
*******************************************************************************/
status_t LC709203F_CALL_INITIAL_RSOC(void *handle);

/*******************************************************************************
* Register      : TEMPERATURE
* Address       : 0x08
* Permission    : RW
*******************************************************************************/
status_t LC709203F_GET_CELL_TEMPERATURE(void *handle, uint16_t *word);
status_t LC709203F_SET_CELL_TEMPERATURE(void *handle, uint16_t word, bool checkResult);

/*******************************************************************************
* Register      : VOLTAGE
* Address       : 0x09
* Permission    : R
*******************************************************************************/
status_t LC709203F_GET_CELL_VOLTAGE(void *handle, uint16_t *word);

/*******************************************************************************
* Register      : CURRENT_DIRECTION
* Address       : 0x0A
* Permission    : RW
*******************************************************************************/
status_t LC709203F_GET_CURRENT_DIRECTION(void *handle, LC709203F_CURRENT_DIRECTION_t *mode);
status_t LC709203F_SET_CURRENT_DIRECTION(void *handle, LC709203F_CURRENT_DIRECTION_t mode, bool checkResult);

/*******************************************************************************
* Register      : ADJUSTMENT_PACK_APPLI
* Address       : 0x0B
* Permission    : RW
*******************************************************************************/
status_t LC709203F_GET_APA(void *handle, uint16_t *word);
status_t LC709203F_SET_APA(void *handle, uint16_t word, bool checkResult);

/*******************************************************************************
* Register      : ADJUSTMENT_PACK_THERM
* Address       : 0x0C
* Permission    : RW
*******************************************************************************/
status_t LC709203F_GET_APT(void *handle, uint16_t *word);
status_t LC709203F_SET_APT(void *handle, uint16_t word, bool checkResult);

/*******************************************************************************
* Register      : RSOC
* Address       : 0x0D
* Permission    : R
*******************************************************************************/
status_t LC709203F_GET_RSOC(void *handle, uint16_t *word);

/*******************************************************************************
* Register      : INDICATOR_TO_EMPTY
* Address       : 0x0F
* Permission    : R
*******************************************************************************/
status_t LC709203F_GET_ITE(void *handle, uint16_t *word);

/*******************************************************************************
* Register      : IC_VERSION
* Address       : 0x11
* Permission    : R
*******************************************************************************/
status_t LC709203F_GET_IC_VERSION(void *handle, uint16_t *word);

/*******************************************************************************
* Register      : CHANGE_OF_THE_PARAM
* Address       : 0x12
* Permission    : RW
*******************************************************************************/
status_t LC709203F_GET_CHANGE_OF_PARAM(void *handle, LC709203F_CHANGE_OF_PARAM_t *mode);
status_t LC709203F_SET_CHANGE_OF_PARAM(void *handle, LC709203F_CHANGE_OF_PARAM_t mode, bool checkResult);

/*******************************************************************************
* Register      : ALARM_LOW_CELL_RSOC
* Address       : 0x13
* Permission    : RW
*******************************************************************************/
status_t LC709203F_GET_ALARM_LOW_RSOC(void *handle, uint16_t *word);
status_t LC709203F_SET_ALARM_LOW_RSOC(void *handle, uint16_t word, bool checkResult);

/*******************************************************************************
* Register      : ALARM_LOW_CELL_VOLT
* Address       : 0x14
* Permission    : RW
*******************************************************************************/
status_t LC709203F_GET_ALARM_LOW_CELL_VOLTAGE(void *handle, uint16_t *word);
status_t LC709203F_SET_ALARM_LOW_CELL_VOLTAGE(void *handle, uint16_t word, bool checkResult);

/*******************************************************************************
* Register      : IC_POWER_MODE
* Address       : 0x15
* Permission    : RW
*******************************************************************************/
status_t LC709203F_GET_IC_POWER_MODE(void *handle, LC709203F_IC_POWER_MODE_t *mode);
status_t LC709203F_SET_IC_POWER_MODE(void *handle, LC709203F_IC_POWER_MODE_t mode, bool checkResult);

/*******************************************************************************
* Register      : STATUS_BIT
* Address       : 0x16
* Permission    : RW
*******************************************************************************/
status_t LC709203F_GET_TEMP_MODE(void *handle, LC709203F_TEMP_MODE_t *mode);
status_t LC709203F_SET_TEMP_MODE(void *handle, LC709203F_TEMP_MODE_t mode, bool checkResult);

/*******************************************************************************
* Register      : NUM_OF_THE_PARAM
* Address       : 0x1A
* Permission    : R
*******************************************************************************/
status_t LC709203F_GET_NUMBER_OF_PARAM(void *handle, uint16_t *word);

#endif