#include <Stepper.h>
#include <I2C>
#include <MPU9250>



int newA1=0;
int newA2=0;
int newA3=0;
int newA4=0;

int oldA1=0;
int oldA2=0;
int oldA3=0;
int oldA4=0;




bool newData = false;
char receivedData[50];
int dataIndex = 0;

int steps_rev=16*200;
Stepper axis1(steps_rev,1,2);
Stepper axis2(steps_rev,4,5);
Stepper axis3(steps_rev,8,7);
Stepper axis4(steps_rev,11,10);


void setup() {
  Serial.begin(9600);
  Serial.println("start!");

  axis1.setSpeed(10);
  axis2.setSpeed(10);
  axis3.setSpeed(10);
  axis4.setSpeed(10);

}

void loop() {
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    
    if (receivedChar == ',') {
      receivedData[dataIndex] = '\0'; // Null-terminate the string
      dataIndex = 0;
      newData = true;
    } else {
      receivedData[dataIndex] = receivedChar;
      dataIndex++;
      if (dataIndex >= 50) {
        dataIndex = 49; // Prevent buffer overflow
      }
    }
  }
  
  if (newData) {
    // Use atoi to convert the received data into integers
    char *token = strtok(receivedData, ",");
    if (token != NULL) {
      newA4 = atoi(token);
      token = strtok(NULL, ",");
      Serial.println(newA4);
    }
    if (token != NULL) {
      newA3 = atoi(token);
      token = strtok(NULL, ",");
    }
    if (token != NULL) {
      newA2 = atoi(token);
      token = strtok(NULL, ",");
    }
    if (token != NULL) {
      newA1 = atoi(token);
    }

    //if(analogRead(A0)>50){
    //  newA1=newA2=newA3=newA4=0;
    //}
    



    newData = false;
  }

    Serial.println(newA1);
    Serial.print(newA2);
    Serial.print(newA3);
    Serial.print(newA4);

    axis1.step(newA1);
    axis2.step(newA2);
    axis3.step(newA3);
    axis4.step(newA4);


    newA1=0;
    newA2=0;
    newA3=0;
    newA4=0;

  delay(1000);
  axis2.step(00);
  axis3.step(1000);
  axis4.step(20000);


}