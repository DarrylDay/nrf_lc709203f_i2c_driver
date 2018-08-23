#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "lc709203f_driver.h"

// TWI instance
#define TWI_INSTANCE_ID     0
static nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

int main(void)
{
    // Configure Log
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\033[2J\033[;H");
    NRF_LOG_INFO("== LC709203F Test ==\n");
    NRF_LOG_FLUSH();

    // Configure Alarm Pin, pullup on board, next rev can remove extrenal pullup and use the internal one
    nrf_gpio_cfg_input(LC709203F_ALARM_PIN, NRF_GPIO_PIN_NOPULL);

    // Configure I2C
    nrf_drv_twi_config_t twi_config = LC709203F_GET_I2C_CONFIG();
    APP_ERROR_CHECK(nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL));
    nrf_drv_twi_enable(&m_twi);

    // Init BFG
    lc709203f_config_t lc709203f_config = LC709203F_GET_DEFAULT_CONFIG();
    LC709203F_INIT(&m_twi, lc709203f_config, true);
    NRF_LOG_FLUSH();

    nrf_delay_ms(2000);

    uint16_t id = 0x0000;
    NRF_LOG_INFO("Reading IC Version Number...");
    if(LC709203F_GET_IC_VERSION(&m_twi, &id) == MEMS_SUCCESS) {
        NRF_LOG_INFO("IC Number = %X\n", id);
    }
    NRF_LOG_FLUSH();

    nrf_delay_ms(1000);

    uint16_t voltage    = 0x0000;
    uint16_t temp       = 0x0000;
    uint16_t RSOC       = 0x0000;
    uint16_t ITE        = 0x0000;
    float ftemp         = 0;
    float fITE          = 0;

    while(1) {

        LC709203F_GET_CELL_VOLTAGE(&m_twi, &voltage);
        LC709203F_GET_CELL_TEMPERATURE(&m_twi, &temp);
        LC709203F_GET_RSOC(&m_twi, &RSOC);
        LC709203F_GET_ITE(&m_twi, &ITE);

        ftemp = ((float)temp)/10;
        fITE = ((float)ITE)/10;

        NRF_LOG_INFO("Cell Voltage = %d mV", voltage);
        NRF_LOG_INFO("Cell Temperature = " NRF_LOG_FLOAT_MARKER " K", NRF_LOG_FLOAT(ftemp));
        NRF_LOG_INFO("RSOC = %d %c", RSOC, '%');
        NRF_LOG_INFO("ITE = " NRF_LOG_FLOAT_MARKER " %c\n", NRF_LOG_FLOAT(fITE), '%');

        NRF_LOG_FLUSH();
        nrf_delay_ms(5000);
        
    }

}