#include <Arduino.h>
#include <AccelStepper.h>

#define SS
#ifdef SS

//#include "GyverStepper.h"
// подключим три мотора
// у первого и второго управление EN не подключаем
//GStepper<STEPPER4WIRE> stepper1(2048, PB13, PB12, PB14, PB15);
//GStepper<STEPPER4WIRE> stepper2(2048, PB4, PB3, PA15, PA12);
//GStepper<STEPPER4WIRE> stepper3(2048, PB9, PB8, PB7, PB6);
AccelStepper stepper2(AccelStepper::FULL4WIRE, PB13, PB12, PB14, PB15);// IN1 IN3 IN4 IN2
AccelStepper stepper3(AccelStepper::FULL4WIRE, PB9, PB8, PB7, PB6);// IN1 IN3 IN4 IN2




void setup() {
      stepper2.setMaxSpeed(300.0);
    stepper2.setAcceleration(100.0);
    stepper2.moveTo(2048);

    stepper3.setMaxSpeed(100.0);
    stepper3.setAcceleration(10.0);
    stepper3.moveTo(2048);
  // мотор 1 просто вращается
  //stepper1.setRunMode(KEEP_SPEED);
  //stepper1.setSpeed(30);
  //stepper1.enable();
  // мотор 2 будет делать sweep по проверке tick
  // stepper2.setRunMode(FOLLOW_POS);
  // stepper2.setMaxSpeed(100);
  // stepper2.setAcceleration(300);
  // // мотор 3 будет перемещаться на случайную позицию
  // stepper3.setRunMode(FOLLOW_POS);
  // stepper3.setMaxSpeed(1000);
  // stepper3.setAcceleration(300);
  // stepper3.autoPower(true);
  // stepper3.enable();
}


void loop() {

  stepper2.run();
   stepper3.run();
 
  // первый мотор
  //stepper1.tick();
  
  // второй крутим туды-сюды (-1000, 1000)
  // if (!stepper2.tick()) {
  //   static bool dir;
  //   dir = !dir;
  //   stepper2.setTarget(dir ? -1000 : 1000);
  // }
  // // третий по таймеру
  // // будет отключаться при остановке
  // stepper3.tick();
  // static uint32_t tmr;
  // if (millis() - tmr > 5000) {   // каждые 5 секунд
  //   tmr = millis();
  //   stepper3.setTarget(random(0, 2000));  // рандом 0-2000
  // }
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

