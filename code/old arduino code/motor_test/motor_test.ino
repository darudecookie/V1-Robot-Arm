#include <AccelStepper.h>

int micro = 256;
int steps_rev = micro * 200;

float rotat = .5;

//steps/rev, step, dir
AccelStepper axis1(AccelStepper::DRIVER, 1, 2);
AccelStepper axis2(AccelStepper::DRIVER, 4, 5);
AccelStepper axis3(AccelStepper::DRIVER, 7, 8);
AccelStepper axis4(AccelStepper::DRIVER, 10, 11);


void setup() {
  axis2.setMaxSpeed(16*200*.15);
  axis2.setAcceleration(16*200*.05);

  axis4.setMaxSpeed(200 * 3);
  axis4.setAcceleration(100 * 1.5);

  Serial.begin(9600);
  Serial.println("");
  Serial.println("start!");

  // put your setup code here, to run once:
}

void loop() {
  //axis4.move(256 * 200 * 2);
  //axis2.move(400);

  //while (axis2.distanceToGo() != 0 || axis4.distanceToGo() != 0) {
    //axis2.run();
    //axis4.run();
  //}


  delay(500);
  //axis2.move(-400);
  axis4.move(256 * 200 * 2);

  while (axis4.distanceToGo() != 0) {
    //axis2.run();
    axis4.run();
  }


  delay(2000);
}
