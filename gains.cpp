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
#define PWM_MAX 1900
#define frequency 25000000.0
#define LED0 0x6
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define LED_MULTIPLYER 4

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
void get_joystick();  //Debug joystick test
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

//global variables
int imu;
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
long last_beat = 0;
float neutral_power = 1550;
int pause_command = 0;
// struct Keyboard {

//   char key_press;

//   int heartbeat;

//   int version;

// };  //Debug Joystick test
struct Keyboard {
  int keypress;
  float pitch;
  float roll;
  float yaw;
  float thrust;
  int version;
};
Keyboard* shared_memory;
int run_program=1;
int pwm;
float prev_pitch = 0, prev_roll = 0;
float pitch_I = 0, roll_I = 0;

int main (int argc, char *argv[])
{//
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

    printf("Desired,Actual\n");  //Debug print titles

    while(run_program==1)
    {
      read_imu();
      update_filter();

      Keyboard keyboard=*shared_memory;
      get_joystick();  //Debug Joystick test
      // printf("%d, %f, %f, %f, %f\n", keyboard.keypress, keyboard.pitch, keyboard.roll, keyboard.yaw, keyboard.thrust); Debug print joystick input
      safety_check(keyboard.keypress, keyboard.version);// keyboard.heartbeat); Debug Joystick Test
      if(pause_command == 0)
        pid_update();
    }

    set_PWM(0,1000);
    set_PWM(1,1000);
    set_PWM(2,1000);
    set_PWM(3,1000);


    return 0;
}

void get_joystick(void)
{
  Keyboard keyboard=*shared_memory;   //Todo Update keyboard to be a global variable updated once a cycle.

  // switch(keyboard.keypress)  //Todo Lookup the syntax for a switch statement
  if(keyboard.keypress != 36)
  {
    if(keyboard.keypress == 32)           //Kill
    {
      run_program = 0;
    }
    else if(keyboard.keypress == 33)      //Pause
    {
    pause_command = 1;
      set_PWM(0,1000);
      set_PWM(1,1000);
      set_PWM(2,1000);
      set_PWM(3,1000);
    }
    else if(keyboard.keypress == 34)      //Unpause
    {
      pause_command = 0;
    }
    else if(keyboard.keypress == 35)      //Calibrate
    {
      calibrate_imu();
    }
  }

}


void pid_update()
{
  //printf("pitch:\t%f\n",pitch_angle);


  Keyboard keyboard=*shared_memory;
  float speed0, speed1, speed2, speed3;

  float pitch_error;
  float desired_pitch = (keyboard.pitch-128.0)*10.0/112.0;
  pitch_error = desired_pitch - pitch_angle;

  float roll_error;
  float desired_roll = (keyboard.roll-128.0)*10.0/112.0;
  roll_error = desired_roll - roll_angle;

  int max_yaw = 120;
  float desired_yaw_rate = (keyboard.yaw-128.0)* max_yaw /112.0;
  float yaw_rate_error = desired_yaw_rate - imu_data[2];
  //like thrust, I want to add a value to some motors, but subtract that from others

  float thrust = neutral_power - (keyboard.thrust-128.0)*200.0/112.0;

  //printf("%f,%f\n", desired_pitch,pitch_angle); //debug

  float pitch_velocity;
  pitch_velocity = pitch_angle - prev_pitch;

  float roll_velocity;
  roll_velocity = roll_angle - prev_roll;

  float P_pitch,D_pitch,I_pitch,P_roll,D_roll,I_roll, P_yaw;
  P_pitch = 12; //24; //Min: ~11.5  12.5
  D_pitch = 425; //350; //150
  I_pitch = 0.09; //0.05; //.03
  P_roll = 12;
  D_roll = 425;
  I_roll = 0.09;//.07
  P_yaw = 1;  //1; todo adjust the joystick range upward to account for low gain

  pitch_I += pitch_error*I_pitch;
  roll_I += roll_error*I_roll;
  float I_max = 150;
  if (pitch_I > I_max) { pitch_I = I_max; }
  if (pitch_I < -I_max) { pitch_I = -I_max;}
  if (roll_I > I_max) { roll_I = I_max; }
  if (roll_I < -I_max) { roll_I = -I_max;}


  //left roll neg, right roll pos, forward pitch neg, backward pitch pos
  //pitch control block
  // speed0 = thrust + pitch_error*P_pitch - pitch_velocity*D_pitch + pitch_I;
  // speed2 = thrust + pitch_error*P_pitch - pitch_velocity*D_pitch + pitch_I;
  // speed1 = thrust - pitch_error*P_pitch + pitch_velocity*D_pitch - pitch_I;
  // speed3 = thrust - pitch_error*P_pitch + pitch_velocity*D_pitch - pitch_I;

  //roll control block
  // speed0 = thrust - roll_error*P_roll + roll_velocity*D_roll - roll_I;
  // speed1 = thrust - roll_error*P_roll + roll_velocity*D_roll - roll_I;
  // speed2 = thrust + roll_error*P_roll - roll_velocity*D_roll + roll_I;
  // speed3 = thrust + roll_error*P_roll - roll_velocity*D_roll + roll_I;

  //Combined Control
  speed0 = thrust - yaw_rate_error*P_yaw - roll_error*P_roll + roll_velocity*D_roll - roll_I + pitch_error*P_pitch - pitch_velocity*D_pitch + pitch_I;
  speed1 = thrust + yaw_rate_error*P_yaw - roll_error*P_roll + roll_velocity*D_roll - roll_I - pitch_error*P_pitch + pitch_velocity*D_pitch - pitch_I;
  speed2 = thrust + yaw_rate_error*P_yaw + roll_error*P_roll - roll_velocity*D_roll + roll_I + pitch_error*P_pitch - pitch_velocity*D_pitch + pitch_I;
  speed3 = thrust - yaw_rate_error*P_yaw + roll_error*P_roll - roll_velocity*D_roll + roll_I - pitch_error*P_pitch + pitch_velocity*D_pitch - pitch_I;

  // speed0 = thrust - yaw_rate_error*P_yaw;
  // speed1 = thrust + yaw_rate_error*P_yaw;
  // speed2 = thrust + yaw_rate_error*P_yaw;
  // speed3 = thrust - yaw_rate_error*P_yaw;





  // printf("%f\n",pitch_angle);
  //printf("Pitch:\t%f\tP:\t%f\tD:\t%f\tI:\t%f\tPWM:\t%f\n",pitch_angle,pitch_error*P,-pitch_velocity*D,pitch_I,speed1);
  printf("%f,%f\n",desired_yaw_rate,imu_data[2]);


  set_PWM(0,speed0);
  set_PWM(2,speed2);
  set_PWM(1,speed1);
  set_PWM(3,speed3);

  prev_pitch = pitch_angle;
  prev_roll = roll_angle;

  //printf("integrator:\r%f\n",pitch_I);

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

  convert_g = 1 / accel_z_calibration;

  float x_avg = imu_cal[4] / 1000.0;
  float y_avg = imu_cal[5] / 1000.0;

  pitch_calibration = (atan2(y_avg,accel_z_calibration))*180.0/3.1415;
  roll_calibration = (atan2(x_avg,accel_z_calibration))*180.0/3.1415;

printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);


}

void read_imu()
{
  int address=59;//todo: set address value for accel x value
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
  imu_data[3]=vw*convert_g;//  todo: convert vw from raw values to "g's"


  address=61;//todo: set address value for accel y value
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[4]=vw*convert_g;//Todo: convert vw from raw valeus to "g's"


  address=63;//todo: set addres value for accel z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[5]=-vw*convert_g;//todo: convert vw from raw values to g's


  address=67;//todo: set addres value for gyro x value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[0]=-x_gyro_calibration+vw/scale;////todo: convert vw from raw values to degrees/second
  imu_data[0]*=-1;

  address=69;//todo: set addres value for gyro y value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[1]=-y_gyro_calibration+vw/scale;////todo: convert vw from raw values to degrees/second
  //imu_data[1]*=-1;

  address=71;////todo: set addres value for gyro z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
 // printf("%f\n",z_gyro_calibration+vw);
  imu_data[2]=-z_gyro_calibration+vw/scale;////todo: convert vw from raw values to degrees/second


  //printf("x accel\t%f\ny accel\t%f\nz accel\t%f\nx gyro\t%f\ny gyro\t%f\nz gyro\t%f\n", imu_data[3], imu_data[4], imu_data[5], imu_data[0], imu_data[1], imu_data[2]);
  imu_cal[0] += imu_data[0];
  imu_cal[1] += imu_data[1];
  imu_cal[2] += imu_data[2];
  imu_cal[3] += imu_data[5];
  imu_cal[4] += imu_data[3];
  imu_cal[5] += imu_data[4];

  //float roll_accel = (atan2(imu_data[4],imu_data[5]))*180.0/3.1415 - roll_calibration;
  //float pitch_accel = (atan2(imu_data[3],imu_data[5]))*180.0/3.1415 - pitch_calibration;
  //printf("z gyro\t%f\n",imu_data[3]);
  //printf("x gryo\t%f\t\ty gyro\t%f\t\tz gyro\t%f\t\troll_angle\t%f\t\tpitch_angle\t%f\n", imu_data[0], imu_data[1], imu_data[3], roll_angle, pitch_angle);

  //printf("%f\t%f\t%f\n",imu_data[3],imu_data[4],imu_data[5]);


}

void update_filter()
{

  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
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

  //printf("roll\t%f\tpitch\t%f\n",roll_angle,pitch_angle);
  //printf("%f, %f, %f, %f, %f, %f\n",roll_angle,roll_accel,imu_data[0],pitch_angle,pitch_accel,imu_data[1]);
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
   set_PWM(0,1000);
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
        printf("keyboard timeout");
    }
}



void safety_check(char keypress, int heartbeat)
{
  if (imu_data[0] > 300 || imu_data[1] > 300 || imu_data[2] > 300)
  {
    run_program = 0;
    set_PWM(0,1000);
    set_PWM(1,1000);
    set_PWM(2,1000);
    set_PWM(3,1000);
    printf("Gyro rate over 300. x: %f y: %f z: %f\n",imu_data[0],imu_data[1],imu_data[2]);
  }
/*
  if (imu_data[3] > 1.8 || imu_data[4] > 1.8 || imu_data[5] > 1.8)
  {
    run_program = 0;

    printf("Impact! Accel > 1.8. x: %f y: %f z: %f\n",imu_data[3],imu_data[4],imu_data[5]);
  }
*/
/*
  if (fabs(imu_data[3]) < .25 && fabs(imu_data[4]) < .25 && fabs(imu_data[5]) < .25)
  {
    run_program = 0;
    printf("Free falling! Accel < .25. x: %f y: %f z: %f\n",imu_data[3],imu_data[4],imu_data[5]);
  }
*/
  if (roll_angle > 45 || roll_angle < -45)
  {
    run_program = 0;
    set_PWM(0,1000);
    set_PWM(1,1000);
    set_PWM(2,1000);
    set_PWM(3,1000);
    printf("Excess Roll! %f\n",roll_angle);
  }

  if (pitch_angle > 45 || pitch_angle < -45)
  {
    run_program = 0;
    printf("Excess pitch! %f\n",pitch_angle);
  }

/*
  pulse(heartbeat);
*/
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
      int sleep = settings | 0x10;
      int wake  = settings & 0xef;
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
//  if(run_program==1)
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
