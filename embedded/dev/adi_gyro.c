#include "ch.h"
#include "hal.h"
#include <math.h>

#include "adi_gyro.h"

static const SPIConfig GyroSPI_cfg =
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

int16_t curr_ang_vel = 0;
int16_t prev_ang_vel = 0;
int32_t gyro_angle = 0;
int16_t curr_real_angle = 0;
int16_t prev_real_angle = 0;
int16_t gyro_cal_result = 0;
uint8_t gyro_state = 0;

int32_t sim1_angle = 0;
int32_t sim2_angle = 0;
uint8_t sim_factor = 0;
int32_t sim_before = 0;
int32_t sim_now = 0;
int32_t sim_angle = 0;
int16_t real_angle =0;
int16_t sim_angle_off  =  0;

static uint16_t spi_frame(const uint16_t data)
{
	uint16_t temp;

  spiAcquireBus(GYRO_SPI);
	spiSelect(GYRO_SPI);

  spiSend(GYRO_SPI, 1, &data);
  spiReceive(GYRO_SPI, 1, &temp);

	spiUnselect(GYRO_SPI);
  spiReleaseBus(GYRO_SPI);

  chThdSleepMicroseconds(10);

  return temp;
}

static void adis_write(const uint8_t addr, const uint16_t data)
{
	uint16_t cmd1, cmd2;
	uint8_t address = ( ( addr & 0x3F ) | 0x80 );
	cmd1 = ( address << 8 ) | ( data >> 8 );
	spi_frame( cmd1 );

	cmd2 = ( (address-1) << 8 ) | ( data & 0x00FF);
	spi_frame( cmd2 );
}

static uint16_t adis_read(const uint8_t addr )
{
	uint16_t address = (0x3F & addr);
	address = address << 8;

  spi_frame(address);
	address = spi_frame( address );
	return address;
}

void gyro_cal(void)
{
  int32_t gyro_zero32 = 0;
  int16_t gyro_zero = 0;
  int16_t tmp = 0 , i =0;
  uint16_t buf = 0;

	if (gyro_state == 0)
		return;

  for (i = 0; i < 1024; i++) {
      if (i >= 512) {
          tmp = gyro_get_vel();
          gyro_zero32 += tmp;
      }
			chThdSleepMilliseconds(4);
  }

  gyro_zero32 = gyro_zero32 / 128;
  gyro_zero32 = -gyro_zero32;
  gyro_zero = (int16_t)gyro_zero32;

  buf |= gyro_zero;
	gyro_cal_result = buf;

  adis_write(GYRO_OFF, buf );
  adis_write(GYRO_COMD, 0x0008 );
	chThdSleepMilliseconds(100); //nessasary. otherwise will fly

	gyro_state = 2;
}

void gyro_cal_short(void)
{
  int32_t gyro_zero32 = 0;
  int16_t gyro_zero = 0;
  int16_t tmp = 0 , i =0;
  uint16_t buf = 0;
	int16_t prev_zero = 0;

	if (gyro_state == 0)
		return;

  for (i = 0; i < 256; i++) {
      tmp = gyro_get_vel();
      if (abs(tmp) > 12)
          return;
      gyro_zero32 += tmp;
			chThdSleepMilliseconds(4);
  }

  gyro_zero32 /= 64;
  gyro_zero32 = -gyro_zero32;
  gyro_zero = (int16_t)gyro_zero32;

  if (abs(gyro_zero) > 1) {
      prev_zero = gyro_get_off();
      gyro_zero += prev_zero;
      buf |= gyro_zero;
      adis_write(GYRO_OFF, buf);
      adis_write(GYRO_COMD,0x0008);
			chThdSleepMilliseconds(100);
  }

	gyro_state = 2;
}

int16_t gyro_get_off(void)
{
    uint16_t buf = 0;
    int16_t off = 0;
    buf = adis_read(GYRO_OFF);

    if (buf & 0x0800 ) // 0b0000100000000000
        buf |= 0xF000; // 0b1111000000000000
    else
        buf &= 0x0FFF; // 0b0000111111111111;
    off |=buf;
    return off;
}

int16_t gyro_get_vel(void)
{
    uint16_t buf = 0;
	  int16_t vel = 0;
    buf = adis_read(GYRO_VEL);
    if (buf & 0x2000 ) // 0b0010000000000000)
        buf |= 0xC000;  //0b1100000000000000
    else
        buf &= 0x3FFF; //0b0011111111111111;
    vel |=buf;

    return vel;
}

uint16_t gyro_get_angle(void)
{
    uint16_t angle = 0;
    angle = adis_read(GYRO_ANGL) & 0x3FFF;   //0b0011111111111111;
    return angle;
}

static void gyro_update(void){

	curr_ang_vel = gyro_get_vel();

	if ( curr_ang_vel < GYRO_ANG_VEL_TH && curr_ang_vel > -GYRO_ANG_VEL_TH) {
	    curr_ang_vel = 0;
	    if (sim_factor) {
			sim_factor = 0;
	    }
	}

	if (curr_ang_vel) {
		if (!sim_factor) {		// 1st time
			sim_before = sim_now;
			sim1_angle = curr_ang_vel;
			sim2_angle = 0;
			sim_factor = 3;
		} else {
			sim1_angle += (sim_factor*prev_ang_vel+curr_ang_vel)/3;
			sim_factor = 4-sim_factor;
			sim2_angle += (sim_factor*prev_ang_vel+curr_ang_vel)/3;
		}

		sim_now = sim_before+(sim_factor == 3 ? sim1_angle : sim2_angle);
		sim_angle = sim_now/GYRO_SCALE ;

		sim_angle = sim_angle % 3600;
		if (sim_angle < 0)
			sim_angle +=3600;
		real_angle = sim_angle = (sim_angle!=0)?3600-sim_angle:0;
		real_angle = (real_angle + sim_angle_off ) % 3600;
		if( real_angle < 0 )
			real_angle += 3600;
	}

  prev_ang_vel = curr_ang_vel;
}


int16_t get_angle(void){
	return real_angle;
}


uint16_t gyro_get_flash(void)
{
	adis_read(GYRO_FLASH);
	return adis_read(GYRO_FLASH);
}

uint16_t gyro_get_power(void)
{
	adis_read(GYRO_POWER);
	return 1832*(adis_read(GYRO_POWER) & 0x0FFF);
}

uint16_t gyro_get_adc(void)
{
	adis_read(GYRO_ADC);
	return 6105*(adis_read(GYRO_ADC) & 0x0FFF);
}

uint16_t gyro_get_temp(void)
{
	adis_read(GYRO_TEMP);
	return 145*(adis_read(GYRO_TEMP) & 0x0FFF);
}

void set_angle(const int16_t angle )
{
	sim_angle_off = angle - sim_angle;
	real_angle = angle ;
}

float yaw_Kp = 1.5;
float yaw_Ki = 0;
float yaw_Kd = 0;

float yaw_temp_derivative = 0;
float yaw_temp_integral = 0;
float yaw_pre_error= 0;

float yaw_pid_output_angle = 0;
int32_t angle_updated = 0;


void yaw_axis_pid_cal(int32_t target_angle, int32_t current_angle){

	float error = target_angle - current_angle;

	float Kout = error * yaw_Kp;

	yaw_temp_integral += error;

	float Iout = yaw_temp_integral * yaw_Ki;

	yaw_temp_derivative = error - yaw_pre_error;

	float Dout = yaw_temp_derivative * yaw_Kd;

	yaw_pid_output_angle = Dout + Iout + Kout;

}

int32_t old_yaw = 0;
int16_t yaw_turn_time = 0;
int32_t output_angle = 0;

#define GYRO_UPDATE_PERIOD  MS2ST(1000U/GYRO_UPDATE_FREQ)
static THD_WORKING_AREA(gyro_thread_wa, 1024);
static THD_FUNCTION(gyro_thread,p)
{
  (void)p;
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

    gyro_update();

    if(old_yaw < 200 && real_angle >3400){
      yaw_turn_time --;
    }
    if(old_yaw>3400 && real_angle < 200){
      yaw_turn_time ++;
    }
    old_yaw = real_angle;

    if(yaw_turn_time != 0){
      output_angle = yaw_turn_time *3600 + real_angle;
    }
    else if (yaw_turn_time == 0){
      output_angle = real_angle;
    }
  }
}

void gyro_init(void)
{
  (*GYRO_SPI).rxdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
  (*GYRO_SPI).txdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
  spiStart(GYRO_SPI, &GyroSPI_cfg);

  //Reset.....
  adis_write(GYRO_COMD,0x0080);		// sofware reset
	chThdSleepMilliseconds(50);

  //Factory Cal...
  adis_write(GYRO_COMD, 0x0002);

  //Set Filter...
  adis_write(GYRO_SENS,0x0404);		// setting the Dynamic Range 320/sec
  adis_write(GYRO_SMPL,0x0001);		// set Internal Sample Rate 1.953 * ( 0x0001 & 0x2F + 1 ) = 3.906ms
  adis_write(GYRO_COMD,0x0008);		// Auxiliary DAC data latch
	chThdSleepMilliseconds(100);

  chThdCreateStatic(gyro_thread_wa,sizeof(gyro_thread_wa),
          NORMALPRIO + 5,
          gyro_thread, NULL);

	gyro_state = 1;
}
