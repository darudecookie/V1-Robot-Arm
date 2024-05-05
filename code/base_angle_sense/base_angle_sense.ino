#include <Wire.h>

const int MPU_addr = 0x68;
float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

int minVal = 265;
int maxVal = 402;

double x;
double y;
double z;

int i = 0;
float zero = 0;
bool zeroed = false;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);


  Serial.begin(19200);
  Serial.println("end setup");
}


long start = millis();

float oldx = 0;
float oldy = 0;
float oldz = 0;

void loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  float xAng = map(AcX, minVal, maxVal, -90, 90);
  float yAng = map(AcY, minVal, maxVal, -90, 90);
  float zAng = map(AcZ, minVal, maxVal, -90, 90);

  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);
  Serial.println(", ");

  Serial.print(x - oldx);
  Serial.print(", ");
  Serial.print(y - oldy);
  Serial.print(", ");
  Serial.print(z - oldz);
  oldx = x;
  oldy = y;
  oldz = z;

  delay(1000);
}
