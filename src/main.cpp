#include <Arduino.h>
#include <AccelStepper.h>
#include "cmsis_os.h"



#define ONE_TURN_STEPS 2048
#define MAX_SPEED_STEPS_PER_SECOND  400
#define ONE_DEGREECE_STEPS  (ONE_TURN_STEPS/360)

#define MOTION_SENSOR_PIN   PB4


AccelStepper stepper0(AccelStepper::FULL4WIRE, PA3, PA2, PA1, PA0);// IN1 IN3 IN4 IN2
AccelStepper stepper2(AccelStepper::FULL4WIRE, PA7, PA6, PA5, PA4);// IN1 IN3 IN4 IN2
AccelStepper stepper3(AccelStepper::FULL4WIRE, PB13, PB12, PB14, PB15);// IN1 IN3 IN4 IN2
AccelStepper stepper1(AccelStepper::FULL4WIRE, PB9, PB8, PB7, PB6);// IN1 IN3 IN4 IN2


#define TASK1_STK_SIZE 512
void task1(void* pdata);
osThreadDef(task1, osPriorityNormal, 1, TASK1_STK_SIZE);

void task1(void* pdata) {
  int count = 1;
  while (1) {
    digitalToggle(LED_BUILTIN);
    osDelay(80);
  }
}




#define TASK2_STK_SIZE 512
void task_steppers_run(void* pdata);
osThreadDef(task_steppers_run, osPriorityNormal, 1, TASK1_STK_SIZE);

void task_steppers_run(void* pdata) {
  while (1) {
    stepper0.run();
    stepper1.run();
    stepper2.run();
    stepper3.run();

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

