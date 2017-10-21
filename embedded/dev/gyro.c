#include "ch.h"
#include "hal.h"
#include "flash.h"
#include "math_misc.h"

#include "adi_gyro.h"

#if !defined(GYRO_ADIS)
#error "Unknown type of Gyro!"
#endif

static GyroStruct gyro;
static lpfilterStruct gyro_lpf;

int16_t gyro_cal_result = 0;

/* Private functions*/
int16_t gyro_get_vel(PGyroStruct pGyro);
uint16_t gyro_get_flash(PGyroStruct pGyro);			//read number of flash for the rom un gyro
uint16_t gyro_get_power(PGyroStruct pGyro);	 	  //return milli-volt
uint16_t gyro_get_adc(PGyroStruct pGyro);		  	//return milli-volt
uint16_t gyro_get_temp(PGyroStruct pGyro); 		  //return milli-degree

PGyroStruct gyro_get(void)
{
  return &gyro;
}

#if defined(GYRO_INTERFACE_SPI)
static gyrodata_t gyro_txrx_spi(SPIDriver* const spid, const gyrodata_t data)
{
	gyrodata_t temp;

  spiAcquireBus(spid);
	spiSelect(spid);

  spiSend(spid, 1, &data);
  spiReceive(spid, 1, &temp);

	spiUnselect(spid);
  spiReleaseBus(spid);

  chThdSleepMicroseconds(10);

  return temp;
}
#endif

static void gyro_write(const PGyroStruct pGyro, const uint8_t addr, const gyrodata_t data)
{
  #if defined(GYRO_ADIS)
    gyrodata_t cmd1, cmd2;
	  uint8_t address = ( ( addr & 0x3F ) | 0x80 );
	  cmd1 = ( address << 8 ) | ( data >> 8 );
    cmd2 = ( (address-1) << 8 ) | ( data & 0x00FF);

    gyro_txrx_spi(pGyro->spid, cmd1 );
	  gyro_txrx_spi(pGyro->spid, cmd2 );
  #endif
}

static uint16_t gyro_read(const PGyroStruct pGyro, const uint8_t addr )
{
  #if defined(GYRO_ADIS)
    uint16_t address = (0x3F & addr) << 8;
    gyro_txrx_spi(pGyro->spid, address);
	  address = gyro_txrx_spi(pGyro->spid, address );
  #endif

	return address;
}

static void gyro_update(PGyroStruct pGyro)
{
	int16_t raw_ang_vel = gyro_get_vel(pGyro);
	if (abs(raw_ang_vel) < GYRO_ANG_VEL_TH)
	  raw_ang_vel = 0;

  float angle, angle_vel, angle_vel_prev;
  angle_vel_prev = pGyro->angle_vel;

  angle_vel = lpfilter_apply(pGyro->lpf, raw_ang_vel/GYRO_SCALE);
  angle = (angle_vel_prev + angle_vel)/2.0f * (GYRO_UPDATE_PERIOD_US / 1000000.0f);

  pGyro->angle_updated = (raw_ang_vel? 1:0);
  pGyro->angle += angle;
  pGyro->angle_vel = angle_vel;
}

void set_angle(PGyroStruct pGyro, const float angle )
{
	pGyro->angle = angle ;
}

float yaw_Kp = 1.5;
float yaw_Ki = 0;
float yaw_Kd = 0;

float yaw_temp_derivative = 0;
float yaw_temp_integral = 0;
float yaw_pre_error= 0;
float yaw_pid_output_angle = 0;

void yaw_axis_pid_cal(int32_t target_angle, int32_t current_angle){

	float error = target_angle - current_angle;
	float Kout = error * yaw_Kp;

	yaw_temp_integral += error;

	float Iout = yaw_temp_integral * yaw_Ki;

	yaw_temp_derivative = error - yaw_pre_error;

	float Dout = yaw_temp_derivative * yaw_Kd;

	yaw_pid_output_angle = Dout + Iout + Kout;

}

#if defined(GYRO_ADIS)
int16_t gyro_get_off(PGyroStruct pGyro)
{
    uint16_t buf = 0;
    int16_t off = 0;
    buf = gyro_read(pGyro, GYRO_OFF);

    if (buf & 0x0800 ) // 0b0000100000000000
        buf |= 0xF000; // 0b1111000000000000
    else
        buf &= 0x0FFF; // 0b0000111111111111;
    off |=buf;
    return off;
}

int16_t gyro_get_vel(PGyroStruct pGyro)
{
    uint16_t buf = 0;
	  int16_t vel = 0;
    buf = gyro_read(pGyro, GYRO_VEL);
    if (buf & 0x2000 ) // 0b0010000000000000)
        buf |= 0xC000;  //0b1100000000000000
    else
        buf &= 0x3FFF; //0b0011111111111111;
    vel |=buf;

    return vel;
}

uint16_t gyro_get_angle(PGyroStruct pGyro)
{
    uint16_t angle = 0;
    angle = gyro_read(pGyro, GYRO_ANGL) & 0x3FFF;   //0b0011111111111111;
    return angle;
}

uint16_t gyro_get_flash(PGyroStruct pGyro)
{
	gyro_read(pGyro, GYRO_FLASH);
	return gyro_read(pGyro, GYRO_FLASH);
}

uint16_t gyro_get_power(PGyroStruct pGyro)
{
	gyro_read(pGyro, GYRO_POWER);
	return 1832*(gyro_read(pGyro, GYRO_POWER) & 0x0FFF);
}

uint16_t gyro_get_adc(PGyroStruct pGyro)
{
	gyro_read(pGyro,GYRO_ADC);
	return 6105*(gyro_read(pGyro, GYRO_ADC) & 0x0FFF);
}

uint16_t gyro_get_temp(PGyroStruct pGyro)
{
	gyro_read(pGyro, GYRO_TEMP);
	return 145*(gyro_read(pGyro, GYRO_TEMP) & 0x0FFF);
}
#endif

#define GYRO_UPDATE_PERIOD  US2ST(GYRO_UPDATE_PERIOD_US)
static THD_WORKING_AREA(gyro_thread_wa, 1024);
static THD_FUNCTION(gyro_thread,p)
{
  PGyroStruct pGyro = (PGyroStruct)p;
  chRegSetThreadName("External Yaw Gyro");

  uint32_t tick = chVTGetSystemTimeX();
  while(true)
  {
    tick += GYRO_UPDATE_PERIOD;
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    if(pGyro->state == INITED)
      gyro_update(pGyro);
  }
}

PGyroStruct gyro_init(void)
{
  PGyroStruct pGyro = &gyro;
  pGyro->lpf = &gyro_lpf;
  memset((void *)pGyro, 0, sizeof(GyroStruct));
  pGyro->angle = 0.0f;
  pGyro->angle_vel = 0.0f;

  //Set low pass filter at cutoff frequency 44Hz
  lpfilter_init(pGyro->lpf, GYRO_UPDATE_FREQ, 44);

  #if defined(GYRO_ADIS)
    pGyro->spid = GYRO_SPI;
    const SPIConfig GyroSPI_cfg =
    {
      NULL,
      GYRO_NSS_port,
      GYRO_NSS_pin,
      SPI_CR1_MSTR | //SPI_Mode_Master
      SPI_CR1_DFF |  //SPI_DataSize_16b
      SPI_CR1_BR_2 | //SPI_BaudRatePrescaler_32
      SPI_CR1_CPHA | //SPI_CPHA_2Edge
      SPI_CR1_CPOL   //SPI_CPOL_High
    };

    pGyro->spid->rxdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
    pGyro->spid->txdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
    spiStart(pGyro->spid, &GyroSPI_cfg);

    //Reset.....
    gyro_write(pGyro, GYRO_COMD,0x0080);		// sofware reset
	  chThdSleepMilliseconds(50);

    //Factory Cal...
    gyro_write(pGyro, GYRO_COMD, 0x0002);

    //Set Filter...
    gyro_write(pGyro, GYRO_SENS,0x0404);		// setting the Dynamic Range 320/sec
    gyro_write(pGyro, GYRO_SMPL,0x0001);		// set Internal Sample Rate 1.953 * ( 0x0001 & 0x2F + 1 ) = 3.906ms
    gyro_write(pGyro, GYRO_COMD,0x0008);		// Auxiliary DAC data latch
	  chThdSleepMilliseconds(100);
  #endif

  chThdCreateStatic(gyro_thread_wa,sizeof(gyro_thread_wa),
          NORMALPRIO + 5,
          gyro_thread, &gyro);

	pGyro->state = INITED;

  return pGyro;
}
