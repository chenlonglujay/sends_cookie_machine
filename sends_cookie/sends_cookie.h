//--------------------------------------
//hardware:
//MEGA 2560
//CLP MOTOR AND DRIVER
//function
//(1)CW (2)CCW
//use TIMER4  and TIMER5 to detect and create pulse
//CLPMotor driver(Hybird Servo Drive) Pulse needs to set 1600
//divide 1600,the smallest pulse is 8(5steps),so at least run once needs 5 steps
//go_step*pulseChange/10 
//5 * 16 /10=8step

//if there haven't thread,Please follow the steps listed below:
//1.Sketch->Include Library -> Manage Libraries
//2.type thread to search and install it
//--------------------------------------

#ifndef __SENDS_COOKIE_H
#define __SENDS_COOKIE_H

#include <Wire.h>
#include <CLP_MOTOR.h>
#include <Thread.h>
#include <ThreadController.h>

#define log_print 0
#define turn_therad_state 1
#if turn_therad_state
ThreadController controll = ThreadController();
Thread  DCMTR_thread = Thread();
Thread  CLPMTR_thread = Thread();
Thread  limit_sensor_thread = Thread();
#endif

#define limit_sensor 37
#define DC_motor 7

//Button pin  sets interrupt
#define control_BTN 2     

//CLP STEP MOTOR setting
CLPMTR *CLPM_tester = new CLPMTR;
#define testerPUL  23
#define testerDIR  22


//timer setting
unsigned int Timer4CountSet[10];
unsigned int Timer5CountSet[7];
#define timer5set 3

struct  CLPMTR_control {
  bool arrive;
  bool DIR;      //0 CW ,1 CCW
  bool TimerSW;      //pulse high low change
  uint8_t set_speed;
  int pulse_count;
  int set_step;
  enum {CW = 0,CCW};
  bool DIR_state;
  bool motor_run;
}  clp_motor_set;

struct DCMTR_control {
  bool arrive;
  bool DIR;
  bool timer_count;  
} dc_motor_set;


#define goto_eat_cookie_steps 5500  //for CW
#define global_pulse goto_eat_cookie_steps*1.6
 void clp_motor_set_initial(struct  CLPMTR_control *input){    
    input->arrive = true;
    input->DIR =  input->CCW;   
    input->TimerSW = 0;      //pulse high low change
    input->set_speed = 1;
    input->pulse_count = 0;
    input->set_step = global_pulse;
    input->DIR_state = 0;
    input->motor_run =false;
}

struct system_state {
    bool limit_sensor_state;     //true: senses something
    bool prevent_startup_into_ISR;
    bool zero_ok;
} SYS_state;

void systme_parameter_initial(struct system_state *input) {
  input->limit_sensor_state = false;  //true: senses something
  input->prevent_startup_into_ISR = false;
  input->zero_ok = false;
}
#endif
