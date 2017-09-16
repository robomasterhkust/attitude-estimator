/*
 *	20170513, Beck Pang, basic i2c functions
 */

#include "i2c_observer.h"

/**
 * @brief i2c master default initialization
 */
void i2c_master_init()
{
    i2c_config_t conf;

    int i2c_master_port = I2C_MASTER_NUM;

    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_io_num       = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief i2c master initialization
 */
void i2c_master_init_0()
{
    i2c_config_t conf;

    int i2c_master_port = I2C_MASTER_NUM_0;

    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = I2C_MASTER_SDA_IO_0;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_io_num       = I2C_MASTER_SCL_IO_0;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}


/**
 * @brief i2c slave initialization
 */
void i2c_slave_init()
{
    int          i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave;

    conf_slave.sda_io_num          = I2C_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en       = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num          = I2C_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en       = GPIO_PULLUP_ENABLE;
    conf_slave.mode                = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr    = ESP_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    i2c_driver_install(i2c_slave_port, conf_slave.mode,
                       I2C_SLAVE_RX_BUF_LEN,
                       I2C_SLAVE_TX_BUF_LEN, 0);
}


/**
 * @brief test function to show buffer
 */
void disp_buf(uint8_t *buf, int len)
{
    int i;

    for (i = 0; i < len; i++)
    {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0)
        {
            printf("\n");
        }
    }
    printf("\n");
}


/**
 * @brief code to read from an address continously
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
esp_err_t i2c_read_words(i2c_port_t i2c_num, uint8_t devAddr, uint8_t regAddr, uint8_t *data_rd, size_t size)
{
    esp_err_t ret;

    if (size == 0)
    {
        return ESP_OK;
    }
    // Acknowledge the slave device for sending out the registers
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, devAddr << 1 | READ_BIT, ACK_CHECK_EN);
    if (size > 1)
    {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/**
 * @brief code to write by registers, not continous write
 *        Master device write data to slave,
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
esp_err_t i2c_write_reg(i2c_port_t i2c_num, uint8_t devAddr, uint8_t regAddr, uint8_t data_wr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data_wr, ACK_CHECK_EN);
    // i2c_master_write(cmd, data_wr, size,  ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
