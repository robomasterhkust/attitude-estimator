#ifndef _ADI_GYRO_H_
#define _ADI_GYRO_H_

//Type of Gyro
#ifndef GYRO
  #define GYRO
  #define GYRO_ADIS
  #define GYRO_INTERFACE_SPI
#else
  #error "Multiple gyro config file found, stop"
#endif
/*==================USER CONFIGURATION==================*/
/* Update frequency of Gyro in Hz */
#define GYRO_UPDATE_FREQ  200U

/* User SPI interface configuration */
#define GYRO_SPI				&SPID3

#define GYRO_MOSI        GPIOC_PIN12
#define GYRO_MISO				 GPIOC_PIN11
#define GYRO_CLK         GPIOC_PIN10

#define GYRO_NSS_pin  GPIOD_PIN2
#define GYRO_NSS_port GPIOD

/* Sensor specific configuration*/
#define GYRO_RESET_PIN  GPIOA_PIN15
#define GYRO_RESET_PORT GPIOA

/*==========END OF USER CONFIGURATION==================*/

#define GYRO_ANG_VEL_TH 		       10
#define GYRO_SCALE 				350.4661806	//		1 / (0.07326 * 3.908ms ) / 10 =349.2838703

#define GYRO_FLASH			0x01
#define GYRO_POWER			0x03
#define GYRO_VEL				0x05
#define GYRO_ADC				0x0B
#define GYRO_TEMP				0x0D
#define GYRO_ANGL				0x0F
#define GYRO_OFF				0x15
#define GYRO_COMD				0x3F
#define GYRO_SENS				0x39
#define GYRO_SMPL				0x37

#define GYRO_UPDATE_PERIOD_US 1000000U/GYRO_UPDATE_FREQ

#define X 0
#define Y 1

typedef uint16_t gyrodata_t;

typedef enum{
  NOT_INITED = 0,
  INITED = 1,
  CALIBRATING = 2
} gyro_state_t;

typedef struct {
  gyro_state_t state;
  uint8_t error_flag;
  SPIDriver* spid;

  uint8_t angle_updated;
  float angle_vel; //angle velocity of Gyro
  lpfilterStruct* lpf; //low pass filter for gyro

  volatile float angle; //Measured Gyro Angle In Rad
} GyroStruct, *PGyroStruct;

extern int32_t gyro_angle;
extern int32_t gyro_comb;
extern int32_t gyro_temp;

PGyroStruct gyro_get(void);
PGyroStruct gyro_init(void);
//init ADIS gyro

int16_t gyro_get_off(PGyroStruct pGyro);				//read offset(result of callibration) for angular velocity
void gyro_set_angle(const float angle);

extern float yaw_pid_output_angle;

void polar_coordinate(int16_t x, int16_t y,int32_t current_angle);

extern float polar_angle ;
extern float polar_distance;

extern float Vx_by_polar;
extern float Vy_by_polar;

#endif
