#include <Arduino.h>
#include <AccelStepper.h>
#include "cmsis_os.h"



#define ONE_TURN_STEPS 2048
#define MAX_SPEED_STEPS_PER_SECOND  400
#define ONE_DEGREECE_STEPS  (ONE_TURN_STEPS/360)

#define SS
#ifdef SS

AccelStepper stepper2(AccelStepper::FULL4WIRE, PB13, PB12, PB14, PB15);// IN1 IN3 IN4 IN2
AccelStepper stepper3(AccelStepper::FULL4WIRE, PB9, PB8, PB7, PB6);// IN1 IN3 IN4 IN2


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
void task2(void* pdata);
osThreadDef(task2, osPriorityNormal, 1, TASK1_STK_SIZE);

void task2(void* pdata) {
  while (1) {
    stepper2.run();
    stepper3.run();

     if (stepper3.distanceToGo() == 0)
	      stepper3.moveTo(-stepper3.currentPosition());
    osDelay(1);
  }
}


void setup() {
    stepper2.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND); // +-400 max
    stepper2.setAcceleration(100.0);
    stepper2.move(ONE_TURN_STEPS*10);

    stepper3.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND);
    stepper3.setAcceleration(MAX_SPEED_STEPS_PER_SECOND*10);
    stepper3.moveTo(248);


  pinMode(LED_BUILTIN, OUTPUT);
  
  osKernelInitialize();                   // TOS Tiny kernel initialize
  osThreadCreate(osThread(task1), NULL);  // Create task1
  osThreadCreate(osThread(task2), NULL);  // Create task1

  osKernelStart();  // Start TOS Tiny


}


void loop() {

  stepper2.run();
  stepper3.run();

     if (stepper3.distanceToGo() == 0)
	stepper3.moveTo(-stepper3.currentPosition());
}


#endif

#ifdef DD
void setup() {
  digitalWrite(PB4, LOW);  // sets the digital pin 13 off
  pinMode(LED_BUILTIN, OUTPUT);

}


void loop() {

  pinMode(PB4, INPUT);    // sets the digital pin 13 as output
  digitalWrite(LED_BUILTIN, LOW);  // sets the digital pin 13 off

  delay(100);            // waits for a second

  pinMode(PB4, OUTPUT);    // sets the digital pin 13 as output
  digitalWrite(LED_BUILTIN, HIGH);  // sets the digital pin 13 off

  delay(100);            // waits for a second

}

#endif

