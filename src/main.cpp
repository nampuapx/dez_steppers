#include <Arduino.h>
#include <AccelStepper.h>
#include "cmsis_os.h"



#define ONE_TURN_STEPS 2048
#define MAX_SPEED_STEPS_PER_SECOND  400
#define ONE_DEGREECE_STEPS  (ONE_TURN_STEPS/360)

#define MOTION_SENSOR_PIN   PB4

uint8_t status = 0;

AccelStepper stepper3(AccelStepper::FULL4WIRE, PA11, PB3, PA15, PA12);// IN1 IN3 IN4 IN2
//AccelStepper stepper2(AccelStepper::FULL4WIRE, PB9, PB8, PB7, PB6);// IN1 IN3 IN4 IN2





#define TASK1_STK_SIZE 512
void task1(void* pdata);
osThreadDef(task1, osPriorityNormal, 1, TASK1_STK_SIZE);

void task1(void* pdata) {
  int count = 1;
  while (1) {
    if(digitalRead(MOTION_SENSOR_PIN)){
        digitalWrite(LED_BUILTIN, LOW);
        status = 5;
            stepper3.move(24800);

        osDelay(5000);
        
        digitalWrite(LED_BUILTIN, HIGH);
        status = 0;

        while(digitalRead(MOTION_SENSOR_PIN)){
          osDelay(10);
        }
    //}else{
        //status = 0;

    }

    osDelay(30);
  }
}

#define TASK2_STK_SIZE 512
void task2(void* pdata);
osThreadDef(task2, osPriorityNormal, 1, TASK1_STK_SIZE);

void task2(void* pdata) {
  while (1) {
    osDelay(1);
    if(!status)continue;

    ///stepper2.run();
    stepper3.run();

     //if (stepper3.distanceToGo() == 0)
	   //   stepper3.moveTo(-stepper3.currentPosition());
  }
}


void setup() {
    //stepper2.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND); // +-400 max
    //stepper2.setAcceleration(100.0);
    //stepper2.move(ONE_TURN_STEPS*10);

    stepper3.setMaxSpeed(MAX_SPEED_STEPS_PER_SECOND);
    stepper3.setAcceleration(MAX_SPEED_STEPS_PER_SECOND*10);
    stepper3.move(24800);


  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MOTION_SENSOR_PIN, INPUT);


  osKernelInitialize();                   // TOS Tiny kernel initialize
  osThreadCreate(osThread(task1), NULL);  // Create task1
  osThreadCreate(osThread(task2), NULL);  // Create task1

  osKernelStart();  // Start TOS Tiny


}


void loop() {

  //stepper2.run();
  //stepper3.run();

    // if (stepper3.distanceToGo() == 0)
	//stepper3.moveTo(-stepper3.currentPosition());
}




