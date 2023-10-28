/**
 * This example turns the ESP32 into a Bluetooth LE Absolute Mouse that clicks
 * the center of the screen every 2 seconds.
 */
#include <BleAbsMouse.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <string.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
BleAbsMouse bleAbsMouse("Air Touch", "Norange", 100);
double baudRate = 115200;
int state=0; // 0: default; 1: calibration corner1; 2: calibration corner2; 3: cursor calibration
bool prevBtnVal = 0;

double vel_x = 0;
double vel_y = 0;
double vel_z = 0;

double oldD_x =0;
double oldD_y =0;
double oldD_z =0;

double endEffector_x = 0;
double endEffector_y = 0;
double endEffector_z = 0;

long elapsedTime;
long lastCheckPoint=0;

double oldVel_x, oldVel_y, oldVel_z =0;

bool mouseIsEnabled(){
  return digitalRead(19)==HIGH;
}

double magnitude(double x, double y, double z){
  return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}

double dot(double x1, double y1, double z1, double x2, double y2, double z2){
  return (x1*x2)+(y1*y2)+(z1*z2);
}

void projAonU(double &new_x, double &new_y, double &new_z, double xa, double ya, double za, double xu, double yu, double zu){
  double projMag = dot(xa, ya, za, xu, yu, zu); // no need to divide by ||u|| assuming it's a unit vector
  new_x = xu*projMag;
  new_y = yu*projMag;
  new_z = zu*projMag;
}

void toUnitVect(double &new_x, double &new_y, double &new_z, double x, double y, double z){
  new_x = x/sqrt(pow(x,2)+pow(y,2)+pow(z,2));
  new_y = y/sqrt(pow(x,2)+pow(y,2)+pow(z,2));
  new_z = z/sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  bleAbsMouse.begin();
  pinMode(18, OUTPUT);
  pinMode(14, OUTPUT);
  digitalWrite(18, HIGH);
  digitalWrite(14, HIGH);
  pinMode(27, INPUT);
  pinMode(19, INPUT);
  Wire.begin(); 
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel_value = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  if(mouseIsEnabled() && bleAbsMouse.isConnected()){
    
    //Serial.println("Click");
    //bleAbsMouse.click(5000, 5000);
    elapsedTime = (micros()-lastCheckPoint);
    lastCheckPoint = micros(); // update last check point  

    vel_x = oldVel_x + elapsedTime*accel_value.x();
    vel_y = oldVel_y + elapsedTime*accel_value.y();
    vel_z = oldVel_z + elapsedTime*accel_value.z();
    Serial.println(accel_value.y());

    //update oldVel variables
    oldVel_x = vel_x;
    oldVel_y = vel_y;
    oldVel_z = vel_z;

    endEffector_x = oldD_x + elapsedTime*vel_x;
    endEffector_y = oldD_y + elapsedTime*vel_y;
    endEffector_z = oldD_z + elapsedTime*vel_z;

    //update oldD variables
    oldD_x = endEffector_x;
    oldD_y = endEffector_y;
    oldD_z = endEffector_z;

    if(prevBtnVal==0 && digitalRead(27)==HIGH){ // if calibration button was pushed
      if(state==1){
        endEffector_x = 0;
        endEffector_y = 0;
        endEffector_z = 0;
      }
      else if(state==2){
        //something
      }
      else if(state==3){
        //something
      }
      else if(state==4){
        //something
      }
      state++;
      prevBtnVal=1;
      if(state==5){
        state=0;
      }
    }
    if(digitalRead(27)==LOW){
      prevBtnVal=0;
    }
    Serial.println(state);
    Serial.println(String(endEffector_x)+" | "+String(endEffector_y)+" | "+String(endEffector_z));
    //delay(2000);
  }
}
