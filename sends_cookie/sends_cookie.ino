#include "sends_cookie.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println(F("start"));
  CLPMTR_initial();
  Timer4_initial();
  Timer5_initial();
  limit_sensor_initial();
  control_BTN_initial();
  interrupt_initial();
#if turn_therad_state
  thread_initial();
#endif
  delay(1000);     //delay wait for CLP motor and driver
  //Serial.println(F("before to  set zero point"));
  bool state = digitalRead(limit_sensor);   //high :senses something
  if (!state) {
    CLPMTR_JogStepSet(clp_motor_set.DIR, 0) ; //CLPMTR goto zero point
  }
  //Serial.println(F("set zero point ok"));
  //DC_motor
}

void loop() {
#if turn_therad_state
  controll.run();
#else if

#endif
}


void limit_sensor_initial() {
  pinMode(limit_sensor, INPUT);
}

void control_BTN_initial() {
  pinMode(control_BTN, INPUT);
}

#if turn_therad_state
//thread_initial
void  thread_initial() {

  DCMTR_thread.onRun(DCMTR_thread_Callback);
  DCMTR_thread.setInterval(100);
  controll.add(&DCMTR_thread);  // Add DCMTR__thread to the controller

  CLPMTR_thread.onRun(CLPMTR_thread_Callback);
  CLPMTR_thread.setInterval(100);
  controll.add(&CLPMTR_thread);  // Add CLRMTR_thread to the controller

  limit_sensor_thread.onRun(limit_sensor_thread_Callback);
  limit_sensor_thread.setInterval(20);
  controll.add(&limit_sensor_thread);  // Add imit_switch_thread to the controller
}
#endif

void DCMTR_thread_Callback() {

}

void CLPMTR_thread_Callback() {
  if (clp_motor_set.motor_run) {
    Serial.println("motor_run: true");
    CLPMTR_JogStepSet(clp_motor_set.DIR, 0);
    clp_motor_set.motor_run = false;    //clp_motor_set.motor_run that can change state when next press button
  }
}

void limit_sensor_thread_Callback() {
  bool state = digitalRead(limit_sensor);   //high :senses something

  if (state) {
    //went to zero point first time when system power on
    if (!SYS_state.prevent_startup_into_ISR) {
      SYS_state.prevent_startup_into_ISR = true;
      SYS_state.zero_ok  = true;
      clp_motor_set.DIR_state  = !clp_motor_set.DIR_state ;
    }
#if log_print
    Serial.print("limit_senses_state:");
    Serial.println("senses something");
#endif
  } else {
#if log_print
    Serial.print("limit_senses_state:");
    Serial.println("senses nothing");
#endif
  }

  SYS_state.limit_sensor_state = state;
}
//timer set and CLPM StepSet
void CLPMTR_JogStepSet(bool CW_CCW, bool CLPM_arrive) {
#if log_print
  Serial.println(F( "CLPMTR_JogStepSet"));
#endif
  set_motor_DIR(CW_CCW);
  TCNT4 = Timer4CountSet[clp_motor_set.set_speed];
  TCNT5 = Timer5CountSet[timer5set];
  clp_motor_set.arrive = CLPM_arrive;
  TimerStart();
#if log_print
  Serial.println(F( "CLPMTR_JogStepSet over"));
#endif
}

//CLP motor initial
void CLPMTR_initial() {
  CLPM_tester->CLP_MOTOR_Initial(testerPUL, testerDIR);
  CLPM_tester->setCLPMTR_LOW();
  clp_motor_set_initial(&clp_motor_set);
  set_motor_DIR( clp_motor_set.CCW);
}

void interrupt_initial() {
  //digitalWrite(control_BTN, HIGH); //turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(control_BTN), change_dir_move_ISR, FALLING);
}

void change_dir_move_ISR() {
  if (SYS_state.prevent_startup_into_ISR) {
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 200 && SYS_state.zero_ok) {
      //first press button motor needs turn CW direction
      //Serial.println(F( "interrupt"));
      clp_motor_set.DIR_state  = !clp_motor_set.DIR_state ;
      set_motor_DIR(clp_motor_set.DIR_state);
      clp_motor_set.motor_run = true;
    }
    last_interrupt_time = interrupt_time;
  }
}


//timer4 initial
void Timer4_initial() {
  Timer4CountSet[0] = 65531; //0.02ms 20us中斷設定
  Timer4CountSet[1] = 65524; //0.048ms 48us中斷設定
  Timer4CountSet[2] = 65511; //0.1ms 100us中斷設定
  Timer4CountSet[3] = 65473; //0.25ms 250us中斷設定
  Timer4CountSet[4] = 65411; //0.5ms 500us中斷設定
  Timer4CountSet[5] = 65286; //1ms 1000us中斷設定
  Timer4CountSet[6] = 65036; //2ms 2000us中斷設定
  Timer4CountSet[7] = 63036; //10ms 中斷設定
  Timer4CountSet[8] = 53036; //50ms 中斷設定
  Timer4CountSet[9] = 40536; //100ms 中斷設定
  TCCR4A =  0x00;
  TCCR4B =  0X03;          //設定 除頻=3 16Mhz/64=0.25Mhz
  //1/0.25Mhz=4us,每4us計數一次,假設設定為65531,
  //共65536-65531=5,第5次發生計時中斷,共經時間5*4us=20us
  TCCR4C =  0x00;
  TIMSK4 =  0x00;      //timer4 stop
}

//timer5 initial
void Timer5_initial() {
  Timer5CountSet[0] = 65531; //20us中斷設定
  Timer5CountSet[1] = 65521;    //0.06ms 60us中斷設定
  Timer5CountSet[2] = 65511; //0.1ms 100us中斷設定
  Timer5CountSet[3] = 65473;  //0.25ms 250us中斷設定
  Timer5CountSet[4] = 65411;  //0.5ms 500us中斷設定
  Timer5CountSet[5] = 65286;   //1ms 1000us中斷設定
  Timer5CountSet[6] = 65036;   //2ms 2000us中斷設定
  TCCR5A =  0x00;
  TCCR5B =  0X03;          //除頻=5 16Mhz/1024 除頻=3 16Mhz/64
  TCCR5C =  0x00;
  TIMSK5 =  0x00;      //timer5 stop
}


//timer start
void TimerStart() {
  TIMSK4 = 0x01;       //timer4 start
  TIMSK5 = 0x01;       //timer5 start
}

//timer stop
void TimerStop() {
  TIMSK4 = 0x00;     //timer4 stop
  TIMSK5 = 0x00;     //timer5 stop
}

//timer4 interrput ISR
//create Pulse for CLPMOTOR driver
ISR (TIMER4_OVF_vect) {
  TIMSK4 = 0x00;     //timer4 stop
  clp_motor_set.TimerSW = ! clp_motor_set.TimerSW;
  TCNT4 = Timer4CountSet[clp_motor_set.set_speed];
  if (!clp_motor_set.arrive) {
    if (clp_motor_set.TimerSW) {
      CLPM_tester->setCLPMTR_HIGH();
    }  else {
      CLPM_tester->setCLPMTR_LOW();
      clp_motor_set.pulse_count = clp_motor_set.pulse_count + 1;
    }
    TIMSK4 = 0x01;       //timer4 start
  } else {
    CLPM_tester->setCLPMTR_LOW();
    TIMSK4 = 0x00;     //timer4 stop
  }
}

//timer5 interrput ISR
ISR (TIMER5_OVF_vect) {
  TIMSK5 = 0x00;     //timer5 stop
  TCNT5 = Timer5CountSet[timer5set];
  if (clp_motor_set.DIR == clp_motor_set.CW) {
    //CW has no limit sensor to check
    //Serial.println(F( "CW"));
    if (clp_motor_set.pulse_count >= clp_motor_set.set_step)  {
      clp_motor_set.arrive = true;
      clp_motor_set.pulse_count = 0;
    }
  } else if (clp_motor_set.DIR == clp_motor_set.CCW) {
    //CCW has limit sensor to chceck
    //Serial.println(F( "CCW"));
    if (SYS_state.limit_sensor_state) {
      clp_motor_set.arrive = true;
      clp_motor_set.pulse_count = 0;
    }
  }

  if (!clp_motor_set.arrive) {
    TIMSK5 = 0x01;       //timer5 start
  } else {
    TIMSK5 = 0x00;     //timer5 stop
  }
}

void set_motor_DIR(bool DIR) {
  if (DIR == clp_motor_set.CW) {
    clp_motor_set.DIR = CLPM_tester->setCLPMTR_CW();
  } else if (DIR == clp_motor_set.CCW) {
    clp_motor_set.DIR = CLPM_tester->setCLPMTR_CCW();
  }
}

void systme_parameter_initial() {
  SYS_state.limit_sensor_state = false;  //true: senses something
  SYS_state.prevent_startup_into_ISR = false;
  SYS_state.zero_ok = false;
}

