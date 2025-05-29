#include <Arduino.h>
#include <AccelStepper.h>
#include "cmsis_os.h"


#define CLOUDS_MOTION_TIME  30000

#define ONE_TURN_STEPS 2048
#define MAX_SPEED_STEPS_PER_SECOND  400
#define ONE_DEGREECE_STEPS  (ONE_TURN_STEPS/360)

#define MOTION_SENSOR_PIN   PB4


#define STEPPER1_IN1_PIN  PA7
#define STEPPER1_IN2_PIN  PA4
#define STEPPER1_IN3_PIN  PA6
#define STEPPER1_IN4_PIN  PA5

#define STEPPER2_IN1_PIN  PB9
#define STEPPER2_IN2_PIN  PB6
#define STEPPER2_IN3_PIN  PB8
#define STEPPER2_IN4_PIN  PB7

#define STEPPER3_IN1_PIN  PB13
#define STEPPER3_IN2_PIN  PB15
#define STEPPER3_IN3_PIN  PB12
#define STEPPER3_IN4_PIN  PB14

#define STEPPER0_IN1_PIN  PA3
#define STEPPER0_IN2_PIN  PA0
#define STEPPER0_IN3_PIN  PA2
#define STEPPER0_IN4_PIN  PA1


void steppers_lines_off(void){
  digitalWrite(STEPPER1_IN1_PIN, LOW);
  digitalWrite(STEPPER1_IN2_PIN, LOW);
  digitalWrite(STEPPER1_IN3_PIN, LOW);
  digitalWrite(STEPPER1_IN4_PIN, LOW);

  digitalWrite(STEPPER2_IN1_PIN, LOW);
  digitalWrite(STEPPER2_IN2_PIN, LOW);
  digitalWrite(STEPPER2_IN3_PIN, LOW);
  digitalWrite(STEPPER2_IN4_PIN, LOW);

  digitalWrite(STEPPER3_IN1_PIN, LOW);
  digitalWrite(STEPPER3_IN2_PIN, LOW);
  digitalWrite(STEPPER3_IN3_PIN, LOW);
  digitalWrite(STEPPER3_IN4_PIN, LOW);
 
  digitalWrite(STEPPER0_IN1_PIN, LOW);
  digitalWrite(STEPPER0_IN2_PIN, LOW);
  digitalWrite(STEPPER0_IN3_PIN, LOW);
  digitalWrite(STEPPER0_IN4_PIN, LOW);
  
}



volatile uint8_t motion_status = 0;

AccelStepper stepper0(AccelStepper::FULL4WIRE, 
                                        STEPPER0_IN1_PIN, 
                                        STEPPER0_IN3_PIN, 
                                        STEPPER0_IN4_PIN, 
                                        STEPPER0_IN2_PIN);// IN1 IN3 IN4 IN2
AccelStepper stepper1(AccelStepper::FULL4WIRE, 
                                        STEPPER1_IN1_PIN, 
                                        STEPPER1_IN3_PIN, 
                                        STEPPER1_IN4_PIN, 
                                        STEPPER1_IN2_PIN);// IN1 IN3 IN4 IN2
AccelStepper stepper3(AccelStepper::FULL4WIRE, 
                                        STEPPER3_IN1_PIN, 
                                        STEPPER3_IN3_PIN, 
                                        STEPPER3_IN4_PIN, 
                                        STEPPER3_IN2_PIN);// IN1 IN3 IN4 IN2
AccelStepper stepper2(AccelStepper::FULL4WIRE, 
                                        STEPPER2_IN1_PIN, 
                                        STEPPER2_IN3_PIN, 
                                        STEPPER2_IN4_PIN, 
                                        STEPPER2_IN2_PIN);// IN1 IN3 IN4 IN2


#define TASK1_STK_SIZE 512
void task1(void* pdata);
osThreadDef(task1, osPriorityNormal, 1, TASK1_STK_SIZE);

void task1(void* pdata) {
  int count = 1;
  while (1) {

    if(digitalRead(MOTION_SENSOR_PIN)){
        motion_status = 5;
        digitalWrite(LED_BUILTIN, LOW);
        osDelay(CLOUDS_MOTION_TIME);
        while(digitalRead(MOTION_SENSOR_PIN)){
          osDelay(10);
        }
        motion_status = 0;
        steppers_lines_off();
    }
    digitalToggle(LED_BUILTIN);
    osDelay(80);
  }
}




#define TASK2_STK_SIZE 512
void task_steppers_run(void* pdata);
osThreadDef(task_steppers_run, osPriorityNormal, 1, TASK1_STK_SIZE);

void task_steppers_run(void* pdata) {
  while (1) {
    if(motion_status){
      stepper0.run();
      stepper1.run();
      stepper2.run();
      stepper3.run();
    }
    osDelay(1);
  }
}

#define TASK3_STK_SIZE 512
void task_stepper3(void* pdata);
osThreadDef(task_stepper3, osPriorityNormal, 1, TASK3_STK_SIZE);

void task_stepper3(void* pdata) {
  while(1){
    while(stepper3.distanceToGo()) osDelay(1);
    stepper3.move(ONE_TURN_STEPS*3);

    while(stepper3.distanceToGo()) osDelay(1);
    stepper3.move(-ONE_TURN_STEPS*1);
  }
}

#define TASK3_STK_SIZE 512
void task_stepper2(void* pdata);
osThreadDef(task_stepper2, osPriorityNormal, 1, TASK3_STK_SIZE);

void task_stepper2(void* pdata) {

  uint8_t polar;

  while(1){

    while(stepper2.distanceToGo()) osDelay(1);
    polar = random();
    polar %=2;

    stepper2.setAcceleration(random(10, 200));

    if(polar){
      stepper2.move(random(ONE_TURN_STEPS>>1, ONE_TURN_STEPS*3));
    }else{
      stepper2.move(-random(ONE_TURN_STEPS>>1, ONE_TURN_STEPS*4));
    }
  }
}



#define TASK3_STK_SIZE 512
void task_stepper0(void* pdata);
osThreadDef(task_stepper0, osPriorityNormal, 1, TASK3_STK_SIZE);

void task_stepper0(void* pdata) {

  uint8_t polar;

  while(1){

    while(stepper0.distanceToGo()) osDelay(1);
    polar = random();
    polar %=2;

    stepper0.setAcceleration(random(10, 200));

    if(polar){
      stepper0.move(random(ONE_TURN_STEPS>>1, ONE_TURN_STEPS*3));
    }else{
      stepper0.move(-random(ONE_TURN_STEPS>>1, ONE_TURN_STEPS*4));
    }
  }
}



#define TASK3_STK_SIZE 512
void task_stepper1(void* pdata);
osThreadDef(task_stepper1, osPriorityNormal, 1, TASK3_STK_SIZE);

void task_stepper1(void* pdata) {

  uint8_t polar;

  while(1){

    while(stepper1.distanceToGo()) osDelay(1);
    polar = random();
    polar %=2;

    stepper1.setAcceleration(random(10, 200));

    if(polar){
      stepper1.move(random(ONE_TURN_STEPS>>1, ONE_TURN_STEPS*3));
    }else{
      stepper1.move(-random(ONE_TURN_STEPS>>1, ONE_TURN_STEPS*4));
    }
  }
}




void setup() {

    stepper0.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND);
    stepper0.setAcceleration(MAX_SPEED_STEPS_PER_SECOND*10);
    stepper0.moveTo(248);

    stepper1.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND);
    stepper1.setAcceleration(MAX_SPEED_STEPS_PER_SECOND*10);
    stepper1.moveTo(248);


    stepper2.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND); // +-400 max
    stepper2.setAcceleration(100.0);
    stepper2.move(ONE_TURN_STEPS*10);

    stepper3.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND);
    stepper3.setAcceleration(MAX_SPEED_STEPS_PER_SECOND*10);
    stepper3.moveTo(248);


  pinMode(LED_BUILTIN, OUTPUT);
  
  osKernelInitialize();                   // TOS Tiny kernel initialize
  osThreadCreate(osThread(task1), NULL);  // Create task1
  osThreadCreate(osThread(task_steppers_run), NULL);  // Create task1
  osThreadCreate(osThread(task_stepper3), NULL);  // Create task1
  osThreadCreate(osThread(task_stepper2), NULL);  // Create task1
  osThreadCreate(osThread(task_stepper0), NULL);  // Create task1
  osThreadCreate(osThread(task_stepper1), NULL);  // Create task1

  osKernelStart();  // Start TOS Tiny

}


void loop() {

}

