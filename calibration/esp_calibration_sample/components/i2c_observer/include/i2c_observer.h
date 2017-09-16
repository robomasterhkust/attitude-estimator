/*
 *	20170513, Beck Pang, I2C set up and functions for observer and communication
 */

#ifndef _I2C_OBSERVER_H_
#define _I2C_OBSERVER_H_

#include <stdio.h>
#include "driver/i2c.h"
#include "board_maxon_imu_pinout.h"


#define DATA_LENGTH                  512               /*!<Data buffer length for test buffer */
// Choose which I2C bus to use
#define I2C_SLAVE_NUM                I2C_NUM_0         /*!<I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN         (2 * DATA_LENGTH) /*!<I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN         (2 * DATA_LENGTH) /*!<I2C slave rx buffer size */
#define I2C_MASTER_NUM               I2C_NUM_1
#define I2C_MASTER_NUM_0             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ           100000            /*!< 100kHz I2C bus */
#define I2C_MASTER_TX_BUF_DISABLE    0                 /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE    0                 /*!< I2C master do not need buffer */

#define ESP_SLAVE_ADDR               0x28              /*!< ESP32 slave address, set to any 7bit value */
#define WRITE_BIT                    I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT                     I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN                 0x1               /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                0x0               /*!< I2C master will not check ack from slave */
#define ACK_VAL                      0x0               /*!< I2C ack value */
#define NACK_VAL                     0x1               /*!< I2C nack value */

void i2c_master_init();
void i2c_master_init_0();
void i2c_slave_init();
void disp_buf(uint8_t *buf, int len);
esp_err_t i2c_read_words(i2c_port_t i2c_num, uint8_t devAddr, uint8_t regAddr, uint8_t *data_rd, size_t size);
esp_err_t i2c_write_reg(i2c_port_t i2c_num, uint8_t devAddr, uint8_t regAddr, uint8_t data_wr);


#endif
