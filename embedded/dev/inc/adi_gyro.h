#ifndef _ADI_GYRO_H_
#define _ADI_GYRO_H_

extern int32_t get_to_turning_angle;

#define GYRO_UPDATE_FREQ 1000U
//NSS //pc12
#define GYRO_NSS_pin  GPIOD_PIN2
#define GYRO_NSS_port GPIOD

#define GYRO_RESET_PIN  GPIOA_PIN15
#define GYRO_RESET_PORT GPIOA
//reset Pa15


// SPI MOSI CLK
#define GYRO_SPI				&SPID3

#define GYRO_MOSI        GPIOC_PIN12
#define GYRO_MISO				 GPIOC_PIN11
#define GYRO_CLK         GPIOC_PIN10

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


#define X 0
#define Y 1

extern int16_t prev_ang_vel;
extern int16_t curr_ang_vel;
extern int32_t gyro_angle;
extern int32_t sim_now;
extern int32_t sim_angle;
extern int32_t gyro_comb;
extern int32_t gyro_temp;
extern int16_t real_angle;
extern uint8_t gyro_state;

//public functions
void gyro_init(void);					//init gyro
void gyro_cal(void);					//cal gyro, only for first calibration
void gyro_cal_short(void);		//cal gyro in shorter time, for re-calibration

uint8_t gyro_get_state(void);				//read status for gyro
int16_t gyro_get_vel(void);				//read angular velocity from gyro
uint16_t gyro_get_angle(void);				//read angle from gyro

int16_t gyro_get_off(void);				//read offset(result of callibration) for angular velocity
uint16_t gyro_get_flash(void);				//read number of flash for the rom un gyro
uint16_t gyro_get_power(void);	 			//return milli-volt
uint16_t gyro_get_adc(void);		  		//return milli-volt
uint16_t gyro_get_temp(void); 		   		//return milli-degree
void set_angle(const int16_t angle);
int16_t get_angle(void);
extern int32_t output_angle ;

extern float yaw_pid_output_angle;
extern int32_t require_angle;

void polar_coordinate(int16_t x, int16_t y,int32_t current_angle);

extern float polar_angle ;
extern float polar_distance;

extern float Vx_by_polar;
extern float Vy_by_polar;

extern int32_t angle_in_3600;
extern int32_t angle_updated;

#endif
