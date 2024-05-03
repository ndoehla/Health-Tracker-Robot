//Natalie Doehla, Student ID: 261029530

#include <Arduino_LSM6DS3.h>
#include <MadgwickAHRS.h>
#include <Math.h>
#include <string.h>

Madgwick filter;
const float sensorRate = 104.00;
int stepCount = 0;
const float xStepThreshold = 0.6; // change to 0.5 for recurse
const float zStepThreshold = 0.9;  // change to 0.9 for recurse
const float yStepThreshold = 0.0;
const float Lgain = 0.2;
const String smoothType = "none"; //change to "recurse" to use recursive function
const int windowSize = 10;
int curIndex = 0;
float accelerationX[windowSize];
float accelerationY[windowSize];
float accelerationZ[windowSize];
bool stepper = false;
float prevSmoothedX = 0.0;
float prevSmoothedY = 0.0;
float prevSmoothedZ = 0.0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);
  if(!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while(1);
  }
  filter.begin(sensorRate);
  pinMode(LED_BUILTIN, OUTPUT);
}

float recursiveFilterX(float dataPoint) {
  float smoothedPoint = prevSmoothedX + Lgain * (dataPoint - prevSmoothedX);
  prevSmoothedX = smoothedPoint;
  return smoothedPoint;

}

float recursiveFilterY(float dataPoint) {
  float smoothedPoint = prevSmoothedY + Lgain * (dataPoint - prevSmoothedY);
  prevSmoothedY = smoothedPoint;
  return smoothedPoint;

}

float recursiveFilterZ(float dataPoint) {
  float smoothedPoint = prevSmoothedZ + Lgain * (dataPoint - prevSmoothedZ);
  prevSmoothedZ = smoothedPoint;
  return smoothedPoint;

}

float updateAverages(float x, float y, float z) {
  accelerationX[curIndex] = x;
  accelerationY[curIndex] = y;
  accelerationZ[curIndex] = z;
  curIndex = (curIndex+1)%windowSize;
}

float calculateMovingAverageX() {
  float sumData = 0;
  for(int i = 0; i<windowSize; i++) {
    sumData += accelerationX[i];
  }
  sumData = sumData / windowSize;
  return sumData;
}

float calculateMovingAverageY() {
  float sumData = 0;
  for(int i = 0; i<windowSize; i++) {
    sumData += accelerationY[i];
  }
  sumData = sumData / windowSize;
  return sumData;
}

float calculateMovingAverageZ() {
  float sumData = 0;
  for(int i = 0; i<windowSize; i++) {
    sumData += accelerationZ[i];
  }
  sumData = sumData / windowSize;
  return sumData;
}



void loop() {
  // put your main code here, to run repeatedly:

  float xDirec, yDirec, zDirec, gx, gy, gz, smoothX, smoothY, smoothZ;
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(xDirec, yDirec, zDirec);
    IMU.readGyroscope(gx, gy, gz);
    filter.updateIMU(gx, gy, gz, xDirec, yDirec, zDirec);
    updateAverages(xDirec, yDirec, zDirec);

    smoothX = calculateMovingAverageX();
    smoothY = calculateMovingAverageY();
    smoothZ = calculateMovingAverageZ();

    if (smoothType.compareTo("none") != 0) {
      smoothX = recursiveFilterX(xDirec);
      smoothY = recursiveFilterY(yDirec);
      smoothZ = recursiveFilterZ(zDirec);
    }

    if(!stepper && smoothX < xStepThreshold && smoothZ >= zStepThreshold) {
      stepper = true;
      Serial.println("starting a step!");
      digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)                  
      stepCount ++;
     Serial.println(stepCount);
   }
    else if(stepper && smoothX >= xStepThreshold && smoothZ < zStepThreshold) {
      stepper = false;
      digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
      Serial.println("done stepping!!");
    }

    //uncomment the following code for serial simulation with graph

  // Serial.print("x-acceleration:");
  //  Serial.print(smoothX);
   //Serial.print(",");
   // Serial.print("y-acceleration:");
   // Serial.print(smoothY);
  //  Serial.print(",");
  //  Serial.print("z-acceleration:");
  //Serial.println(smoothZ);
    
  }


  delay(80);

}
