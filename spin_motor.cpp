#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>

#include <sys/time.h>
#include <stdint.h>
#include <signal.h>

#include <sys/shm.h>
#include <sys/stat.h>
//#include <curses.h>
//gcc -o cal esc_cal.cpp -lwiringPi -lncurses -lm
#define frequency 25000000.0
#define LED0 0x6			//LED0 start register
#define LED0_ON_L 0x6		//LED0 output and brightness control byte 0
#define LED0_ON_H 0x7		//LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8		//LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9		//LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4	// For the other 15 channels
#define i_lim 50
#define motor_lim_max 1800
#define motor_lim_min 1000

int pwm;

void init_pwm(int pwm)
{
    float freq =400.0*.95;
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    uint8_t prescale = floor(prescaleval+0.5);
    int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
    int sleep	= settings | 0x10;
    int wake 	= settings & 0xef;
    int restart = wake | 0x80;
    wiringPiI2CWriteReg8(pwm, 0x00, sleep);
    wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
    wiringPiI2CWriteReg8(pwm, 0x00, wake);
    delay(10);
    wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
}

void init_motor(int pwm,uint8_t channel)
{
	int on_value=0;

	int time_on_us=900;
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1200;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1000;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

}
void set_PWM(int pwm, uint8_t channel, float time_on_us)
{
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);

}


int main (int argc, char *argv[])
{



		wiringPiSetup () ;


    //setup imu on I2C
  
    pwm=wiringPiI2CSetup (0x40);

    if(pwm==-1)
    {
        printf("cant connect to I2C device %d\n",pwm);
        return -1;
    }
    else
    {
      
      init_pwm(pwm);
      init_motor(pwm,0);
      init_motor(pwm,1);
      init_motor(pwm,2);
      init_motor(pwm,3);
      int motor=0;
      int speed=1000;
        while(1)
        {
		
     
          //		set_PWM(pwm,0,2000);	
          //	printf("plug int esc to pwm port 0 and attach battery, press enter when done\n\r");
          //	getchar();
          //  set_PWM(pwm,0,900);
          printf("set motor\n\r");	
           scanf("%d",&motor);
               printf("set speed\n\r");	
           scanf("%d",&speed);
          
          if(motor==4)
          {
           if(speed>=1000&&speed<=2000)
            {
              set_PWM(pwm,0,speed); 
              set_PWM(pwm,1,speed);
               set_PWM(pwm,2,speed);
                set_PWM(pwm,3,speed);
             
              printf("setting all motor to  speed%d\n\r",speed);	
            }
          
          }
          else
          {
            if(speed>=1000&&speed<=2000)
            {
              set_PWM(pwm,motor,speed);
             
              printf("setting motor %d to  speed%d\n\r",motor,speed);	
            }
          }
          
       

}








  	 }
        return 0;
}
