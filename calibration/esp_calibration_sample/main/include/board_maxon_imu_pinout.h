/*	20170509, Beck Pang, for the pin out on the board
 *	with both Maxon motor driver and two IMUs
 */

#ifndef _MAXON_IMU_H
#define _MAXON_IMU_H

/*
 * Board second pinout, 20170508 by Beck Pang,
 * I2C_1 for reading MPU6050 measurement
 * I2C_0 for slave and board to board communication
 * I2C_1 SCL -> GPIO 19, SDA -> GPIO 21
 * I2C_0 SCL -> GPIO 22, SDA -> GPIO 23
 */
#define I2C_MASTER_SCL_IO            19                /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO            21                /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL_IO_0          22                /*!< gpio number for I2C slave clock */
#define I2C_MASTER_SDA_IO_0          23                /*!< gpio number for I2C slave data  */
#define I2C_SLAVE_SCL_IO             22                /*!< gpio number for I2C slave clock */
#define I2C_SLAVE_SDA_IO             23                /*!< gpio number for I2C slave data  */

#define GPIO_EN_IO              (25) /*!< gpio number for enable pin, but using the PIN_SEL */
#define GPIO_PWM_IO             (32)
#define ADC1_CURRENT_CHANNEL    (0)  /*!< gpio number for SENSOR_VP */
#define ADC1_SPEED_CHANNEL      (3)  /*!< gpio number for SENSOR_VN */
#define CAN_RX_IO               (16) /*!< gpio number for CAN bus RX */
#define CAN_TX_IO               (5)  /*!< gpio number for CAN bus TX */
#define UART_1_RX1_IO           (9) /*!< gpio number for UART RX */
#define UART_1_TX1_IO           (10) /*!< gpio number for UART TX */

#define GPIO_EN_PIN_SEL         (1 << GPIO_EN_IO)

#endif
