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
#include <curses.h>
#include "vive.h"

//gcc -o flight_manager flight_manager.cpp -lwiringPi -lncurses -lm

#define frequency  25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define PWM_MAX          1850
#define frequency        25000000.0
#define LED0             0x6
#define LED0_ON_L        0x6
#define LED0_ON_H        0x7
#define LED0_OFF_L       0x8
#define LED0_OFF_H       0x9
#define LED_MULTIPLYER   4

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

//Function Declarations
int setup_imu();
void get_joystick();
void calibrate_imu();
void read_imu();
void update_filter();
void setup_keyboard();
void trap(int signal);
void pulse(int heartbeat);
void safety_check(char c, int h);
void init_pwm();
void init_motor(uint8_t channel);
void set_PWM( uint8_t channel, float time_on_us);
void pid_update();
void vive_control();

//global variables
int imu;
float desired_yaw_rate;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //gyro xyz, accel xyz
float imu_cal[6]; // for finding the bias
long time_curr;
long time_prev;
float time_total = 0;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
float convert_g=1.0;
float A=.01;      //More A favors accelerometer, less favors gyro
int previous_beat = 0;  //Gyro is stable, accelerometer is accurate
int previous_vive_beat = 0; //Heartbeat for vive
long last_beat = 0;
long last_vive_beat = 0;  //Vive
float neutral_power = 1600;
int pause_command = 0;
Position local_p;       //Position struct from vive.h

struct Keyboard {
  int keypress;
  float pitch;
  float roll;
  float yaw;
  float thrust;
  int version;
};

Keyboard* shared_memory;
Keyboard keyboard;

int run_program=1;
int pwm;
float prev_pitch = 0, prev_roll = 0;
float pitch_I = 0, roll_I = 0;
float desired_roll;
float prev_x = 0;
float x_est = 0;
float desired_pitch;
float prev_y = 0;
float y_est = 0;

int main (int argc, char *argv[])
{
    init_shared_memory();
    init_pwm();
    init_motor(0);
    init_motor(1);
    init_motor(2);
    init_motor(3);
    delay(1000);

    setup_imu();
    calibrate_imu();

    setup_keyboard();
    signal(SIGINT, &trap);

    printf("x, y, z, yaw\n");  //Debug print titles

    /*  //Debug File print
    FILE *file_output = fopen("output.csv","w");
    fputs("Output1, Output2\n",file_output);   //Do this in main before the loop
    */

    /*  //Debug File print
    char str[100];
    sprintf(str,"%f, %f\n",value1,value2);
    fputs(str,file_output);  //Do this in the function with relevant data
    */

    while(run_program==1)
    {
      local_p=*position;
      read_imu();
      update_filter();

      keyboard = *shared_memory;
      get_joystick();
      safety_check(keyboard.keypress, keyboard.version);
      if(pause_command == 0)
      {
        vive_control();
        pid_update();
      }
    }

    set_PWM(0,1000);
    set_PWM(1,1000);
    set_PWM(2,1000);
    set_PWM(3,1000);

    /*  //Debug File print
    fclose(file_output);  //Place this in the shutdown path
    */

    return 0;
}

void get_joystick(void)
{
    switch(keyboard.keypress)
    {
        case 32  :                      //Kill
            run_program = 0;
            break;

        case 33  :                      //Pause
            pause_command = 1;
            set_PWM(0,1000);
            set_PWM(1,1000);
            set_PWM(2,1000);
            set_PWM(3,1000);
            break;

        case 34  :                      //Unpause
            pause_command = 0;
            break;

        case 35  :                      //Calibrate
            calibrate_imu();
            break;
    }
}

void vive_control()
{
  int Kpv = 200;
  desired_yaw_rate = -Kpv*(0-local_p.yaw); //(keyboard.yaw-128.0)* max_yaw /112.0;

  float Px = 0.004;
  float Dx = 2.0;
  x_est = x_est*.6 + local_p.x*.4;
  desired_roll = -Px*(x_est - 0) - Dx*(x_est - prev_x);
  prev_x = x_est;


  //float Py = 0.008;
  //float Dy = 1.7;
  float Py = Px;
  float Dy = Dx;
  y_est = y_est*.6 + local_p.y*.4;
  desired_pitch = -Py*(y_est - 0) - Dy*(y_est - prev_y);
  prev_y = y_est;

  //Debug: Are these needed? Concerned they may be preventing recovery
  // if (desired_roll > 3.0) { desired_roll = 3.0; }
  // else if (desired_roll < -3.0) { desired_roll = -3.0; }
  // if (desired_pitch > 3.0) { desired_pitch = 3.0; }
  // else if (desired_pitch < -3.0) { desired_pitch = -3.0; }

  printf("%f\t%f\t%f\t%f\n",roll_angle,desired_roll,pitch_angle,desired_pitch);
}


void pid_update()
{
  float speed0, speed1, speed2, speed3;

  float pitch_error;
  desired_pitch = desired_pitch + 0.5*(keyboard.pitch-128.0)*15.0/112.0;
  pitch_error = desired_pitch - pitch_angle;

  float roll_error;
  desired_roll = desired_roll + 0.5*(keyboard.roll-128.0)*15.0/112.0;
  roll_error = desired_roll - roll_angle;

  int max_yaw = 120;
  //desired_yaw_rate = Kpv*(0-local_p.yaw); //(keyboard.yaw-128.0)* max_yaw /112.0;
  float yaw_rate_error = desired_yaw_rate - imu_data[2];
  //like thrust, but I want to add a value to some motors and subtract that from others.

  float thrust = neutral_power - (keyboard.thrust-128.0)*200.0/112.0;


  float pitch_velocity;
  pitch_velocity = pitch_angle - prev_pitch;

  float roll_velocity;
  roll_velocity = roll_angle - prev_roll;

  float P_pitch,D_pitch,I_pitch,P_roll,D_roll,I_roll, P_yaw;
  P_pitch = 24; //24; //Min: ~11.5  12.5
  D_pitch = 350; //350; //150
  I_pitch = 0.05; //0.05; //.03
  P_roll = 12;
  D_roll = 425;
  I_roll = 0.07;
  P_yaw = 1;

  pitch_I += pitch_error*I_pitch;
  roll_I += roll_error*I_roll;
  float I_max = 150;
  if (pitch_I > I_max) { pitch_I = I_max; }
  if (pitch_I < -I_max) { pitch_I = -I_max;}
  if (roll_I > I_max) { roll_I = I_max; }
  if (roll_I < -I_max) { roll_I = -I_max;}


  //left roll neg, right roll pos, forward pitch neg, backward pitch pos
  //Combined Control
  speed0 = thrust - yaw_rate_error*P_yaw - roll_error*P_roll + roll_velocity*D_roll - roll_I + pitch_error*P_pitch - pitch_velocity*D_pitch + pitch_I;
  speed1 = thrust + yaw_rate_error*P_yaw - roll_error*P_roll + roll_velocity*D_roll - roll_I - pitch_error*P_pitch + pitch_velocity*D_pitch - pitch_I;
  speed2 = thrust + yaw_rate_error*P_yaw + roll_error*P_roll - roll_velocity*D_roll + roll_I + pitch_error*P_pitch - pitch_velocity*D_pitch + pitch_I;
  speed3 = thrust - yaw_rate_error*P_yaw + roll_error*P_roll - roll_velocity*D_roll + roll_I - pitch_error*P_pitch + pitch_velocity*D_pitch - pitch_I;


  //printf("%f,%f\n",desired_yaw_rate,imu_data[2]);   //debug removing this loses stability

  //Debug turn back on
  set_PWM(0,speed0);
  set_PWM(2,speed2);
  set_PWM(1,speed1);
  set_PWM(3,speed3);


  prev_pitch = pitch_angle;
  prev_roll = roll_angle;
}

void calibrate_imu()
{
  imu_cal[0] = 0.0;
  imu_cal[1] = 0.0;
  imu_cal[2] = 0.0;
  imu_cal[3] = 0.0;
  imu_cal[4] = 0.0;
  imu_cal[5] = 0.0;

  /*
  x_gyro_calibration=??
  y_gyro_calibration=??
  z_gyro_calibration=??
  roll_calibration=??
  pitch_calibration=??
  accel_z_calibration=??
  */

  for(int i = 0; i<1000; i++)
  {
    read_imu();
    update_filter();
  }
  x_gyro_calibration = imu_cal[0] / 1000.0;
  y_gyro_calibration = imu_cal[1] / 1000.0;
  z_gyro_calibration = imu_cal[2] / 1000.0;
  accel_z_calibration = imu_cal[3] / 1000.0;

  convert_g = 1 / accel_z_calibration;  //todo validate this and or improve it

  float x_avg = imu_cal[4] / 1000.0;
  float y_avg = imu_cal[5] / 1000.0;

  pitch_calibration = (atan2(y_avg,accel_z_calibration))*180.0/3.1415;
  roll_calibration = (atan2(x_avg,accel_z_calibration))*180.0/3.1415;

printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);


}

void read_imu()
{
  int address=59; //Accel x value
  float ax=0;
  float az=0;
  float ay=0;
  int vh,vl;
  float scale=65.546;

  //read in data
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  //convert 2 complement
  int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[3]=vw*convert_g; //Convert vw from raw values to "g's"


  address=61; //Accel y value
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[4]=vw*convert_g; //Convert vw from raw valeus to "g's"


  address=63; //Accel z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[5]=-vw*convert_g;  //Convert vw from raw values to g's


  address=67; //Gyro x value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[0]=-x_gyro_calibration+vw/scale; //Convert vw from raw values to degrees/second
  imu_data[0]*=-1;

  address=69; //Gyro y value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[1]=-y_gyro_calibration+vw/scale; //Convert vw from raw values to degrees/second

  address=71; //Gyro z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[2]=-z_gyro_calibration+vw/scale; //Convert vw from raw values to degrees/second

//todo refigure calibrate_imu() suchthat it doesn't need to collect this data after it finishes. It's pointless.
  imu_cal[0] += imu_data[0];
  imu_cal[1] += imu_data[1];
  imu_cal[2] += imu_data[2];
  imu_cal[3] += imu_data[5];
  imu_cal[4] += imu_data[3];
  imu_cal[5] += imu_data[4];

}

void update_filter()
{

  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;         //This is only the fractional part of a second since last reading. It wouldn't include whole seconds...
  //Compute time since last execution
  float imu_diff=time_curr-time_prev;

  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;

  //comp. filter for roll, pitch here:
  float pitch_accel = (atan2(imu_data[4],imu_data[5]))*180.0/3.1415 - pitch_calibration;
  float roll_accel = (atan2(imu_data[3],imu_data[5]))*180.0/3.1415 - roll_calibration;

  float pitch_gyro_delta = imu_data[0]*imu_diff;
  float roll_gyro_delta = imu_data[1]*imu_diff;

  roll_angle = roll_accel*A + (1-A)*(roll_gyro_delta + roll_angle);
  pitch_angle = pitch_accel*A + (1-A)*(pitch_gyro_delta + pitch_angle);

}


int setup_imu()
{
  wiringPiSetup ();


  //setup imu on I2C
  imu=wiringPiI2CSetup (0x68) ; //accel/gyro address

  if(imu==-1)
  {
    printf("-----cant connect to I2C device %d --------\n",imu);
    return -1;
  }
  else
  {

    printf("connected to i2c device %d\n",imu);
    printf("imu who am i is %d \n",wiringPiI2CReadReg8(imu,0x75));

    uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS


    //init imu
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x00);
    printf("                    \n\r");
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00);
    wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04
    int c=wiringPiI2CReadReg8(imu,  GYRO_CONFIG);
    wiringPiI2CWriteReg8(imu,  GYRO_CONFIG, c & ~0xE0);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c | Ascale << 3);
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2, c & ~0x0F); //
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2,  c | 0x00);
  }
  return 0;
}

void setup_keyboard()
{
  int segment_id;
  struct shmid_ds shmbuffer;
  int segment_size;
  const int shared_segment_size = 0x6400;
  int smhkey=33222;

  /* Allocate a shared memory segment.  */
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
  /* Attach the shared memory segment.  */
  shared_memory = (Keyboard*) shmat (segment_id, 0, 0);
  printf ("shared memory attached at address %p\n", shared_memory);
  /* Determine the segment's size. */
  shmctl (segment_id, IPC_STAT, &shmbuffer);
  segment_size  =               shmbuffer.shm_segsz;
  printf ("segment size: %d\n", segment_size);
  /* Write a string to the shared memory segment.  */
  //sprintf (shared_memory, "test!!!!.");



}


void trap(int signal)
{
   set_PWM(0,1000);   //todo: Remove these redundant PWM settings?
   set_PWM(1,1000);
   set_PWM(2,1000);
   set_PWM(3,1000);

   printf("ending program\n\r");

   run_program=0;
}


void pulse(int heartbeat)
{
    if(heartbeat != previous_beat)
    {
        last_beat = time_curr;
        previous_beat = heartbeat;
    }

    if((time_curr-last_beat) > .25*1000000000)
    {
        run_program = 0;
        printf("Joystick timeout!");
    }
}

void vive_pulse(int heartbeat)
{

    // printf("beat: %d\n",heartbeat);
    if (time_curr < .1*1000000000)
      last_vive_beat = 0;
    if(heartbeat != previous_vive_beat)
    {
        last_vive_beat = time_curr;
        previous_vive_beat = heartbeat;

    }

    // printf("%ld\n", (time_curr-last_vive_beat));
    if(((time_curr-last_vive_beat) > .5*1000000000)) //todo Increase to .5, fix rollover issue
    {
        run_program = 0;
        printf("Vive timeout!");
    }
}


void safety_check(char keypress, int heartbeat)
{
  //Gyro Rate
  if (imu_data[0] > 300 || imu_data[1] > 300 || imu_data[2] > 300)
  {
    run_program = 0;
    printf("Gyro rate over 300. x: %f y: %f z: %f\n",imu_data[0],imu_data[1],imu_data[2]);
  }
/*
  //Impact
  if (imu_data[3] > 1.8 || imu_data[4] > 1.8 || imu_data[5] > 1.8)
  {
    run_program = 0;

    printf("Impact! Accel > 1.8. x: %f y: %f z: %f\n",imu_data[3],imu_data[4],imu_data[5]);
  }
*/
/*
  //Free Falling
  if (fabs(imu_data[3]) < .25 && fabs(imu_data[4]) < .25 && fabs(imu_data[5]) < .25)
  {
    run_program = 0;
    printf("Free falling! Accel < .25. x: %f y: %f z: %f\n",imu_data[3],imu_data[4],imu_data[5]);
  }
*/
  //Over Roll
  if (roll_angle > 45 || roll_angle < -45)
  {
    run_program = 0;
    printf("Excess Roll! %f\n",roll_angle);
  }

  //Over Pitch
  if (pitch_angle > 45 || pitch_angle < -45)
  {
    run_program = 0;
    printf("Excess pitch! %f\n",pitch_angle);
  }


  //Controller Disconnect
  pulse(heartbeat);

  //Vive Disconnect
  vive_pulse(local_p.version);

}

void init_pwm()
{

    pwm=wiringPiI2CSetup (0x40);
    if(pwm==-1)
    {
      printf("-----cant connect to I2C device %d --------\n",pwm);

    }
    else
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
}

void init_motor(uint8_t channel)
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

void set_PWM( uint8_t channel, float time_on_us)
{
  {
    if(time_on_us>PWM_MAX)
    {
      time_on_us=PWM_MAX;
    }
    else if(time_on_us<1000)
    {
      time_on_us=1000;
    }
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
}
