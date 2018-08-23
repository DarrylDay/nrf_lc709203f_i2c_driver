/* Includes ------------------------------------------------------------------*/
#include "lc709203f_driver.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/*****************************************************************************/
/*                                                                           */
/*                           Private Functions                               */
/*                                                                           */
/*****************************************************************************/

/*******************************************************************************
* Function Name  : _get_crc
* Description    : Does a CRC check and returns the checksum
* Input          : *rec_values (Array of Values), len (Length of Array)
* Output         : None
* Return         : crc (Checksum Value)
*******************************************************************************/
static uint8_t _get_crc(uint8_t *rec_values, uint8_t len)
{
    uint8_t crc = 0x00;
    uint8_t current_byte;
    uint8_t bit;

    for (current_byte = 0; current_byte < len; current_byte++) {
        crc ^= (rec_values[current_byte]);
        for (bit = 8; bit > 0; bit--) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ LC709203F_CRC_POLYNOMIAL;
            }
            else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/*******************************************************************************
* Function Name  : LC709203F_READ_WORD
* Description    : Read a Word from a Register
* Input          : *handle (Pointer to I2C Instance), reg (Register Value)
* Output         : *word (Register Word Value)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
static status_t LC709203F_READ_WORD(void *handle, uint8_t reg, uint16_t *word) {

  // Create tx and rx buffer
  uint8_t m_tx_buf[] = {reg};
  uint8_t m_rx_buf[3];
  memset(m_rx_buf, 0, 3);

  NRF_LOG_DEBUG("TX Buffer: ");
  NRF_LOG_HEXDUMP_DEBUG(m_tx_buf, sizeof(m_tx_buf));

  // Write Register to Read From
  ret_code_t err_code = nrf_drv_twi_tx(handle, LC709203F_ADDR_7BIT, m_tx_buf, 1, true);
  APP_ERROR_CHECK(err_code);

  // Read from Register and get Checksum
  err_code = nrf_drv_twi_rx(handle, LC709203F_ADDR_7BIT, m_rx_buf, 3);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEBUG("RX Buffer: ");
  NRF_LOG_HEXDUMP_DEBUG(m_rx_buf, sizeof(m_rx_buf));

  // Compare Checksums
  uint8_t crc_buf[5] = { LC709203F_ADDR_8BIT, reg, LC709203F_ADDR_8BIT+1, m_rx_buf[0], m_rx_buf[1] };
  if (_get_crc(crc_buf, 5) != m_rx_buf[2]) {
      NRF_LOG_ERROR("Read Word CRC Incorrect.");
      return MEMS_ERROR;
  }

  *word = (uint16_t)m_rx_buf[1] << 8 | (uint16_t)m_rx_buf[0];

  NRF_LOG_FLUSH();

  return MEMS_SUCCESS;

}

/*******************************************************************************
* Function Name  : LC709203F_WRITE_WORD
* Description    : Write a Word to a Register
* Input          : *handle (Pointer to I2C Instance), reg (Register Value), 
*                  *word (Word to Write), crc (Checksum Value), checkResult (Check Word Wrote Correctly)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
static status_t LC709203F_WRITE_WORD(void *handle, uint8_t reg, uint16_t word, bool checkResult) {

  // Create Lower and Upper Byte
  uint8_t lowerByte = (uint8_t)(word);
  uint8_t upperByte = (uint8_t)(word >> 8);

  // Generate Checksum and Create tx buffer
  uint8_t crc_buf[4] = { LC709203F_ADDR_8BIT, reg, lowerByte, upperByte };
  uint8_t m_tx_buf[] = {reg, lowerByte, upperByte, _get_crc(crc_buf, 4) };

  NRF_LOG_DEBUG("TX Buffer: ");
  NRF_LOG_HEXDUMP_DEBUG(m_tx_buf, sizeof(m_tx_buf));

  // Write Register to Read From
  ret_code_t err_code = nrf_drv_twi_tx(handle, LC709203F_ADDR_7BIT, m_tx_buf, sizeof(m_tx_buf), false);
  APP_ERROR_CHECK(err_code);

  if(checkResult) {

    uint16_t tempWord = 0x0000;
    if(LC709203F_READ_WORD(handle, reg, &tempWord) == MEMS_SUCCESS) {

      if(tempWord == word) {
        NRF_LOG_DEBUG("Check Write Result Passed.");
      } else {
        NRF_LOG_ERROR("Check Write Result Failed.");
        return MEMS_ERROR;
      }

    } else {
      return MEMS_ERROR;
    }

  }

  NRF_LOG_FLUSH();

  return MEMS_SUCCESS;

}

/*****************************************************************************/
/*                                                                           */
/*                            Public Functions                               */
/*                                                                           */
/*****************************************************************************/

/*******************************************************************************
* Function Name  : LC709203F_GET_I2C_CONFIG
* Description    : Get I2C Config
* Input          : None
* Output         : None
* Return         : nrf_drv_twi_config_t
*******************************************************************************/
nrf_drv_twi_config_t LC709203F_GET_I2C_CONFIG(){

  nrf_drv_twi_config_t twi_config = {
       .scl                = 15,
       .sda                = 14,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

  return twi_config;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_DEFAULT_CONFIG
* Description    : Get Default Config
* Input          : None
* Output         : None
* Return         : lc709203f_config_t
*******************************************************************************/
lc709203f_config_t LC709203F_GET_DEFAULT_CONFIG() {

  lc709203f_config_t config;

  config.thermistorB        = 0x0D34;                                 // B = 3380
  config.currentDirection   = LC709203F_CURRENT_DIRECTION_AUTO;
  config.apa                = 0x0007;
  config.apt                = 0x001E;                                 // Default
  config.batteryProfile     = LC709203F_CHANGE_OF_PARAM_TYPE_2;
  config.alarmLowRSOC       = 0x0014;                                 // 20%
  config.alarmLowVoltage    = 0x0000;                                 // Disabled
  config.powerMode          = LC709203F_IC_POWER_MODE_OPERATIONAL;
  config.temperatureMode    = LC709203F_TEMP_MODE_THERMISTOR;

  return config;

}

/*******************************************************************************
* Function Name  : LC709203F_INIT
* Description    : Initalize IC
* Input          : config (LC709203F Config), runInitialRSOC (Run Initial RSOC Command)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_INIT(void *handle, lc709203f_config_t config, bool runInitialRSOC) {

  NRF_LOG_INFO("Running LC709203F Initialization...");

  if(LC709203F_SET_IC_POWER_MODE(handle, config.powerMode, true) == MEMS_ERROR) {
    NRF_LOG_ERROR("LC709203F Init Aborted!\n");
    return MEMS_ERROR;
  }

  if(LC709203F_SET_APA(handle, config.apa, true) == MEMS_ERROR) {
    NRF_LOG_ERROR("LC709203F Init Aborted!\n");
    return MEMS_ERROR;
  }

  if(LC709203F_SET_APT(handle, config.apt, true) == MEMS_ERROR) {
    NRF_LOG_ERROR("LC709203F Init Aborted!\n");
    return MEMS_ERROR;
  }

  if(LC709203F_SET_CHANGE_OF_PARAM(handle, config.batteryProfile, true) == MEMS_ERROR) {
    NRF_LOG_ERROR("LC709203F Init Aborted!\n");
    return MEMS_ERROR;
  }

  if(LC709203F_SET_CURRENT_DIRECTION(handle, config.currentDirection, true) == MEMS_ERROR) {
    NRF_LOG_ERROR("LC709203F Init Aborted!\n");
    return MEMS_ERROR;
  }

  if(runInitialRSOC) {
    if(LC709203F_CALL_INITIAL_RSOC(handle) == MEMS_ERROR) {
      NRF_LOG_ERROR("LC709203F Init Aborted!\n");
      return MEMS_ERROR;
    }
  }

  if(LC709203F_SET_TEMP_MODE(handle, config.temperatureMode, true) == MEMS_ERROR) {
    NRF_LOG_ERROR("LC709203F Init Aborted!\n");
    return MEMS_ERROR;
  }

  if(LC709203F_SET_THERMISTOR_B(handle, config.thermistorB, true) == MEMS_ERROR) {
    NRF_LOG_ERROR("LC709203F Init Aborted!\n");
    return MEMS_ERROR;
  }

  if(LC709203F_SET_ALARM_LOW_RSOC(handle, config.alarmLowRSOC, true) == MEMS_ERROR) {
    NRF_LOG_ERROR("LC709203F Init Aborted!\n");
    return MEMS_ERROR;
  }

  if(LC709203F_SET_ALARM_LOW_CELL_VOLTAGE(handle, config.alarmLowVoltage, true) == MEMS_ERROR) {
    NRF_LOG_ERROR("LC709203F Init Aborted!\n");
    return MEMS_ERROR;
  }

  NRF_LOG_INFO("LC709203F Initialization Complete.\n");

  return MEMS_SUCCESS;

}

/*******************************************************************************
* Function Name  : LC709203F_CALL_BEFORE_RSOC
* Description    : Call Before RSOC Operation
* Input          : *handle (Pointer to I2C Instance)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_CALL_BEFORE_RSOC(void *handle) {

  if(LC709203F_WRITE_WORD(handle, LC709203F_REG_BEFORE_RSOC, LC709203F_PARAM_INITIAL_RSOC, false) == MEMS_ERROR) {
    NRF_LOG_ERROR("Call Before RSOC Operation Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_THERMISTOR_B
* Description    : Read Thermistor B Value
* Input          : *handle (Pointer to I2C Instance)
* Output         : *word (Thermistor B Value)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_THERMISTOR_B(void *handle, uint16_t *word) {

  if(LC709203F_READ_WORD(handle, LC709203F_REG_THERMISTOR_B, word) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read Thermistor B Value Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;

}

/*******************************************************************************
* Function Name  : LC709203F_SET_THERMISTOR_B
* Description    : Write Thermistor B Value
* Input          : *handle (Pointer to I2C Instance), word (Thermistor B Value),
*                  checkResult (Check Word Wrote Correctly)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_SET_THERMISTOR_B(void *handle, uint16_t word, bool checkResult) {

  if(LC709203F_WRITE_WORD(handle, LC709203F_REG_THERMISTOR_B, word, checkResult) == MEMS_ERROR) {
    NRF_LOG_ERROR("Write Thermistor B Value Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_CALL_INITIAL_RSOC
* Description    : Call Initial RSOC Operation
* Input          : *handle (Pointer to I2C Instance)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_CALL_INITIAL_RSOC(void *handle) {

  if(LC709203F_WRITE_WORD(handle, LC709203F_REG_INITIAL_RSOC, LC709203F_PARAM_INITIAL_RSOC, false) == MEMS_ERROR) {
    NRF_LOG_ERROR("Call Initial RSOC Operation Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_CELL_TEMPERATURE
* Description    : Read Cell Temperature in 0.1 Kelvin (0.1K), 0.0°C = 0x0AAC
* Input          : *handle (Pointer to I2C Instance)
* Output         : *word (Cell Temperature Value)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_CELL_TEMPERATURE(void *handle, uint16_t *word) {
  
  if(LC709203F_READ_WORD(handle, LC709203F_REG_TEMPERATURE, word) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read Cell Temperature Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_SET_CELL_TEMPERATURE
* Description    : Read Cell Temperature in 0.1 Kelvin (0.1K), Range: 0x09E4 to 0x0D04
* Input          : *handle (Pointer to I2C Instance), *word (Cell Temperature Value),
*                  checkResult (Check Word Wrote Correctly)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_SET_CELL_TEMPERATURE(void *handle, uint16_t word, bool checkResult) {

  if(word < 0x09E4 || word > 0x0D04) {
    NRF_LOG_ERROR("Inputted tempature is outside of writable range!\n");
    return MEMS_ERROR;
  }

  if(LC709203F_WRITE_WORD(handle, LC709203F_REG_TEMPERATURE, word, checkResult) == MEMS_ERROR) {
    NRF_LOG_ERROR("Write Cell Temperature Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_CELL_VOLTAGE
* Description    : Read Cell Voltage (mV)
* Input          : *handle (Pointer to I2C Instance)
* Output         : *word (Cell Voltage Value)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_CELL_VOLTAGE(void *handle, uint16_t *word) {
  
  if(LC709203F_READ_WORD(handle, LC709203F_REG_VOLTAGE, word) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read Cell Voltage Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_CURRENT_DIRECTION
* Description    : Read Current Direction Mode
* Input          : *handle (Pointer to I2C Instance)
* Output         : *mode (Current Direction Mode)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_CURRENT_DIRECTION(void *handle, LC709203F_CURRENT_DIRECTION_t *mode) {

  if(LC709203F_READ_WORD(handle, LC709203F_REG_CURRENT_DIRECTION, (uint16_t *)mode) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read Current Direction Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_SET_CURRENT_DIRECTION
* Description    : Write Current Direction Mode
* Input          : *handle (Pointer to I2C Instance), mode (Current Direction Mode),
*                  checkResult (Check Word Wrote Correctly)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_SET_CURRENT_DIRECTION(void *handle, LC709203F_CURRENT_DIRECTION_t mode, bool checkResult) {

  if(LC709203F_WRITE_WORD(handle, LC709203F_REG_CURRENT_DIRECTION, (uint16_t)mode, checkResult) == MEMS_ERROR) {
      NRF_LOG_ERROR("Write Current Direction Failed!\n");
      return MEMS_ERROR;
    }

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_APA
* Description    : Read APA (mOhm), Range: 0x0000 to 0x00FF
* Input          : *handle (Pointer to I2C Instance)
* Output         : *word (APA Value)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_APA(void *handle, uint16_t *word) {
  
  if(LC709203F_READ_WORD(handle, LC709203F_REG_ADJUSTMENT_PACK_APPLI, word) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read APA Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_SET_APA
* Description    : Write APA (mOhm), Range: 0x0000 to 0x00FF
* Input          : *handle (Pointer to I2C Instance), word (APA Value),
*                  checkResult (Check Word Wrote Correctly)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_SET_APA(void *handle, uint16_t word, bool checkResult) {

  if(word > 0x00FF) {
    NRF_LOG_ERROR("Inputted APA is outside of writable range!\n");
    return MEMS_ERROR;
  }

  if(LC709203F_WRITE_WORD(handle, LC709203F_REG_ADJUSTMENT_PACK_APPLI, word, checkResult) == MEMS_ERROR) {
    NRF_LOG_ERROR("Write APA Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_APT
* Description    : Read APT
* Input          : *handle (Pointer to I2C Instance)
* Output         : *word (APT Value)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_APT(void *handle, uint16_t *word) {
  
  if(LC709203F_READ_WORD(handle, LC709203F_REG_ADJUSTMENT_PACK_THERM, word) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read APT Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_SET_APT
* Description    : Write APT
* Input          : *handle (Pointer to I2C Instance), word (APT Value),
*                  checkResult (Check Word Wrote Correctly)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_SET_APT(void *handle, uint16_t word, bool checkResult) {

  if(LC709203F_WRITE_WORD(handle, LC709203F_REG_ADJUSTMENT_PACK_THERM, word, checkResult) == MEMS_ERROR) {
    NRF_LOG_ERROR("Write APA Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_RSOC
* Description    : Read RSOC (1%), Range: 0x0000 to 0x0064
* Input          : *handle (Pointer to I2C Instance)
* Output         : *word (RSOC Value)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_RSOC(void *handle, uint16_t *word) {
  
  if(LC709203F_READ_WORD(handle, LC709203F_REG_RSOC, word) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read RSOC Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_ITE
* Description    : Read RSOC (0.1%), Range: 0x0000 to 0x03E8
* Input          : *handle (Pointer to I2C Instance)
* Output         : *word (RSOC Value)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_ITE(void *handle, uint16_t *word) {
  
  if(LC709203F_READ_WORD(handle, LC709203F_REG_INDICATOR_TO_EMPTY, word) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read ITE Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_IC_VERSION
* Description    : Read ID Version Value
* Input          : *handle (Pointer to I2C Instance)
* Output         : *word (IC Version Value)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_IC_VERSION(void *handle, uint16_t *word) {

  if(LC709203F_READ_WORD(handle, LC709203F_REG_IC_VERSION, word) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read Device ID Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_CHANGE_OF_PARAM
* Description    : Read Change of the Param
* Input          : *handle (Pointer to I2C Instance)
* Output         : *mode (Change of the Param Mode)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_CHANGE_OF_PARAM(void *handle, LC709203F_CHANGE_OF_PARAM_t *mode) {

  if(LC709203F_READ_WORD(handle, LC709203F_REG_CHANGE_OF_THE_PARAM, (uint16_t *)mode) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read Change of the Param Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_SET_CHANGE_OF_PARAM
* Description    : Write Change of the Param
* Input          : *handle (Pointer to I2C Instance), mode (Change of the Param),
*                  checkResult (Check Word Wrote Correctly)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_SET_CHANGE_OF_PARAM(void *handle, LC709203F_CHANGE_OF_PARAM_t mode, bool checkResult) {

  if(LC709203F_WRITE_WORD(handle, LC709203F_REG_CHANGE_OF_THE_PARAM, (uint16_t)mode, checkResult) == MEMS_ERROR) {
      NRF_LOG_ERROR("Write Change of the Param Failed!\n");
      return MEMS_ERROR;
    }

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_ALARM_LOW_RSOC
* Description    : Read Alarm Low RSOC Threshold (1%), Range: 0x0000 to 0x0064
* Input          : *handle (Pointer to I2C Instance)
* Output         : *word (Alarm Low RSOC Threshold Value)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_ALARM_LOW_RSOC(void *handle, uint16_t *word) {
  
  if(LC709203F_READ_WORD(handle, LC709203F_REG_ALARM_LOW_CELL_RSOC, word) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read Alarm Low Cell RSOC Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_SET_ALARM_LOW_RSOC
* Description    : Write Alarm Low RSOC Threshold (1%), Range: 0x0000 to 0x0064
* Input          : *handle (Pointer to I2C Instance), word (Alarm Low RSOC Threshold Value),
*                  checkResult (Check Word Wrote Correctly)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_SET_ALARM_LOW_RSOC(void *handle, uint16_t word, bool checkResult) {

  if(word > 0x0064) {
    NRF_LOG_ERROR("Inputted Alarm Low RSOC is outside of writable range!\n");
    return MEMS_ERROR;
  }

  if(LC709203F_WRITE_WORD(handle, LC709203F_REG_ALARM_LOW_CELL_RSOC, word, checkResult) == MEMS_ERROR) {
    NRF_LOG_ERROR("Write Alarm Low RSOC Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_ALARM_LOW_CELL_VOLTAGE
* Description    : Read Alarm Low Cell Voltage Threshold (mV)
* Input          : *handle (Pointer to I2C Instance)
* Output         : *word (Alarm Low Voltage Threshold Value)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_ALARM_LOW_CELL_VOLTAGE(void *handle, uint16_t *word) {
  
  if(LC709203F_READ_WORD(handle, LC709203F_REG_ALARM_LOW_CELL_VOLT, word) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read Alarm Low Cell Voltage Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_SET_ALARM_LOW_CELL_VOLTAGE
* Description    : Write Alarm Low Cell Voltage Threshold (mV)
* Input          : *handle (Pointer to I2C Instance), word (Alarm Low Voltage Threshold Value),
*                  checkResult (Check Word Wrote Correctly)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_SET_ALARM_LOW_CELL_VOLTAGE(void *handle, uint16_t word, bool checkResult) {

  if(LC709203F_WRITE_WORD(handle, LC709203F_REG_ALARM_LOW_CELL_VOLT, word, checkResult) == MEMS_ERROR) {
    NRF_LOG_ERROR("Write Alarm Low Cell Voltage Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_IC_POWER_MODE
* Description    : Read IC Power Mode
* Input          : *handle (Pointer to I2C Instance)
* Output         : *mode (IC Power Mode)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_IC_POWER_MODE(void *handle, LC709203F_IC_POWER_MODE_t *mode) {

  if(LC709203F_READ_WORD(handle, LC709203F_REG_IC_POWER_MODE, (uint16_t *)mode) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read IC Power Mode Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_SET_IC_POWER_MODE
* Description    : Write IC Power Mode
* Input          : *handle (Pointer to I2C Instance), mode (IC Power Mode),
*                  checkResult (Check Word Wrote Correctly)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_SET_IC_POWER_MODE(void *handle, LC709203F_IC_POWER_MODE_t mode, bool checkResult) {

  if(LC709203F_WRITE_WORD(handle, LC709203F_REG_IC_POWER_MODE, (uint16_t)mode, checkResult) == MEMS_ERROR) {
      NRF_LOG_ERROR("Write IC Power Mode Failed!\n");
      return MEMS_ERROR;
    }

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_TEMP_MODE
* Description    : Read Temperature Mode
* Input          : *handle (Pointer to I2C Instance)
* Output         : *mode (Temperature Mode)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_TEMP_MODE(void *handle, LC709203F_TEMP_MODE_t *mode) {

  if(LC709203F_READ_WORD(handle, LC709203F_REG_STATUS_BIT, (uint16_t *)mode) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read Temperature Mode Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_SET_IC_POWER_MODE
* Description    : Write Temperature Mode
* Input          : *handle (Pointer to I2C Instance), mode (Temperature Mode),
*                  checkResult (Check Word Wrote Correctly)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_SET_TEMP_MODE(void *handle, LC709203F_TEMP_MODE_t mode, bool checkResult) {

  if(LC709203F_WRITE_WORD(handle, LC709203F_REG_STATUS_BIT, (uint16_t)mode, checkResult) == MEMS_ERROR) {
      NRF_LOG_ERROR("Write Temperature Mode Failed!\n");
      return MEMS_ERROR;
    }

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LC709203F_GET_NUMBER_OF_PARAM
* Description    : Read Number of the Param Value, 0x0301 or 0x0504
* Input          : *handle (Pointer to I2C Instance)
* Output         : *word (Number of the Param Value)
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LC709203F_GET_NUMBER_OF_PARAM(void *handle, uint16_t *word) {
  
  if(LC709203F_READ_WORD(handle, LC709203F_REG_NUM_OF_THE_PARAM, word) == MEMS_ERROR) {
    NRF_LOG_ERROR("Read Number of the Param Failed!\n");
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}