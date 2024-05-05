#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

String inputString = "";
int A5_Angle = 90;
int A6_Angle = 90;
int A5_Angle_old = 90;
int A6_Angle_old = 90;

float vert = 14;
bool Zeroing;

AccelStepper *axisArray[4];
int currentA[4] = { 0 };
int newA[4];

const int MotorPins[4][2] = {
  { 23, 25 },
  { 27, 29 },
  { 31, 33 },
  { 35, 37 }
};
/*
const int MotorPins[4][2] = {
  { 22, 24 }, { 28, 30 }, { 36, 34 }, { 42, 40 }
};
*/

const long speed_accel_microsteps_reduc[4][4] = {
  { 2 * 2000, 2 * 550, 16, 20 },
  { 80000, 4 * 5000, 16, -15 },
  { 3 * 50000, 75000, 8, 15 },
  { 2 * 15000, 2 * 15000, 8, 3 }
};

Servo Axis5;
Servo Axis6;

void setup() {

  Serial.begin(9600);
  Serial.println("");
  Serial.println("begin setup!");
  for (int x = 0; x < 4; x++) {
    axisArray[x] = new AccelStepper(AccelStepper::DRIVER, MotorPins[x][0], MotorPins[x][1]);
    axisArray[x]->setMaxSpeed(speed_accel_microsteps_reduc[x][0]);
    axisArray[x]->setAcceleration(speed_accel_microsteps_reduc[x][1]);
  }

  Axis5.attach(29);
  Axis6.attach(25);

  Serial.println("end setup!");
}

void loop() {
  // if (analogRead(A0) < 600) {
  if (0 > 600) {
    Zeroer();
  } else {
    if (Serial.available()) {
      char receivedChar = Serial.read();

      if (receivedChar != '\n') {
        inputString += receivedChar;
      } else {
        Mover();
        inputString = "";  // Clear the input string
      }
    }
  }
}

void Mover() {
  if (Zeroing) {
    for (int x; x < 4; x++) {
      currentA[x] = 0;
      newA[x] = 0;
    }
  } else {

    int parsedNumbers = sscanf(inputString.c_str(), "%d,%d,%d,%d,%d,%d", &newA[0], &newA[1], &newA[2], &newA[3], &A5_Angle, &A6_Angle);
    if (parsedNumbers == 6) {
      Serial.println("data success");
      A5_Angle = Servo_checker(A5_Angle, "Axis 5");
      A6_Angle = Servo_checker(A6_Angle, "Axis 6");
    } else {
      Serial.println("data error! ");
      for (int x; x < 4; x++) {
        newA[x] = currentA[x];
        A5_Angle = A5_Angle_old;
        A6_Angle = A6_Angle_old;
      }
    }

    for (int x = 0; x < 4; x++) {

      float steps_to_move = -(currentA[x] - newA[x]) * (speed_accel_microsteps_reduc[x][2]);
      steps_to_move = (steps_to_move * 200) / 360;
      steps_to_move = steps_to_move * float(speed_accel_microsteps_reduc[x][3]);

      axisArray[x]->move(steps_to_move);
      currentA[x] = newA[x];
    }
    A5_Angle_old = A5_Angle;
    A6_Angle_old = A6_Angle;

    if ((currentA[1] < -20) || (currentA[1] > 200)) {
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Serial.println("AXIS 1: OUT OF ANGLE ERROR");
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

      while (analogRead(A1) < 300) {
        delay(100);
      }
    }

    Axis5.write(A5_Angle);
    Axis6.write(A6_Angle);
    while (axisArray[0]->distanceToGo() != 0 || axisArray[1]->distanceToGo() != 0 || axisArray[2]->distanceToGo() != 0 || axisArray[3]->distanceToGo() != 0) {
      axisArray[0]->run();
      axisArray[1]->run();
      axisArray[2]->run();
      axisArray[3]->run();
    }
  }

  Zeroing = false;
}

void Zeroer() {

  long prevzero;
  long currentzero = millis();

  if (currentzero - prevzero > 100 && analogRead(A0) > 600) {
    prevzero = currentzero;
    Zeroing = true;
    Serial.println("Arm zero'd! ");
    Mover();
  }
}

int Servo_checker(int x, String y) {

  bool error = false;
  x = 90 - x;

  if (x > 180) {

    x = 180;
    error = true;
  } else if (x < 0) {
    x = 0;
    error = true;
  }
  if (error) {
    Serial.println("");
    Serial.print(y);
    Serial.print(" servo");
    Serial.print(" out of bounds - moving to closes angle in bounds (0 or 180 deg) ");
  }

  return x;
}

void fk_math(int angles[4]) {

  float radians[4];

  float absA3 = angles[2] - (180 - angles[1]);
  absA3 *= (M_PI / 180);

  for (int x = 0; x < 4; x++) {
    radians[x] = angles[x] * (M_PI / 180);
  }

  float arm_len_xy = 255.5 * cos(angles[0]) + 117.54 * cos(absA3);

  float end_x = cos(angles[0]) * arm_len_xy + 112.15 * cos(angles[1] + M_PI / 2);
  float end_y = sin(angles[0]) * arm_len_xy + 112.15 * sin(angles[0] + M_PI / 2);
  float end_z = vert + 217.55 + 255.5 * sin(angles[1]) + 117.54 * sin(absA3);

  Serial.println("");
  Serial.print("moving to: ");
  Serial.print(end_x);
  Serial.print(", ");
  Serial.print(end_y);
  Serial.print(", ");
  Serial.print(end_z);
  Serial.println("");
}