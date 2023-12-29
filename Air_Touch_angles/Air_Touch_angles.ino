/**
 * This example turns the ESP32 into a Bluetooth LE Absolute Mouse that clicks
 * the center of the screen every 2 seconds.
 */

 //max x value is 359 deg
 //max y value is 89 deg
 //max z value is 179 deg

//#include <BleAbsMouse.h>
#include <BleMouse.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <string.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
BleMouse bleMouse("Air Touch", "Norange", 100);
double baudRate = 115200;
int state=0; // 0: default; 1: calibration corner1; 2: calibration corner2; 3: cursor calibration
bool prevBtnVal = 0;

float xRes = 3840;
float yRes = 2160;

float xMultiplier = xRes/359;
float yMultiplier = yRes/179;

float angle_x_origin=0;
float angle_y_origin=0;
float angle_z_origin=0;

float angle_x_end=359;
float angle_y_end=179;
float angle_z_end=0;

float angle_x = 0;
float angle_y = 0;
float angle_z = 0;

float relevantAngle_x=0;
float relevantAngle_y=0;

int currentPixel_x = 0;
int currentPixel_y = 0;

long lastCheckPoint=0;

bool mouseIsEnabled(){
  return digitalRead(19)==HIGH;
}

bool btnIsPressed(){
  return (digitalRead(27)==LOW);
}

void updatePos(){
  if(angle_x>=angle_x_origin && angle_y>=angle_y_origin && angle_x<=angle_x_end && angle_y<=angle_y_end){
    bleMouse.move(int(relevantAngle_x*xMultiplier)-currentPixel_x,int(relevantAngle_y*yMultiplier)-currentPixel_y);
    currentPixel_x+=(relevantAngle_x*xMultiplier);
    currentPixel_y+=(relevantAngle_y*yMultiplier);
    if(currentPixel_x<0){
      currentPixel_x=0;
    }
    if(currentPixel_y<0){
      currentPixel_y=0;
    }
    if(currentPixel_x>xRes){
      currentPixel_x=xRes;
    }
    if(currentPixel_y>yRes){
      currentPixel_y=yRes;
    }
  }
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
  bleMouse.begin();
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
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
  imu::Vector<3> accel_value = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  if(mouseIsEnabled() && bleMouse.isConnected()){
    
    //Serial.println("Click");
    //bleMouse.click(5000, 5000);
    
    angle_x = euler.x();
    angle_y = 90+euler.y();
    angle_z = euler.z();

    relevantAngle_x = angle_x-(angle_x_origin+(abs(angle_x_end-angle_x_origin)/2));
    relevantAngle_y = angle_y-(angle_y_origin+(abs(angle_y_end-angle_y_origin)/2));


    if(prevBtnVal==0 && btnIsPressed()){ // if calibration button was pushed
      if(state==1){
        angle_x_origin = angle_x;
        angle_y_origin = angle_y;
        angle_z_origin = angle_z;
        bleMouse.move(-currentPixel_x,-currentPixel_y);
        currentPixel_x=0;
        currentPixel_y=0;
      }
      else if(state==2){
        angle_x_end=angle_x;
        angle_y_end=angle_y;
        bleMouse.move(xRes-currentPixel_x,yRes-currentPixel_y);
        currentPixel_x=xRes;
        currentPixel_y=yRes;
        xMultiplier=xRes/abs(angle_x_end-angle_x_origin);
        yMultiplier=yRes/abs(angle_y_end-angle_y_origin);
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
    if(!btnIsPressed()){
      prevBtnVal=0;
    }
    Serial.println(state);
    Serial.println(String(angle_x)+" | "+String(angle_y)+" | "+String(angle_z));
    Serial.println(String(relevantAngle_x)+" | "+String(relevantAngle_y));
    Serial.println("Multipliers: "+String(xMultiplier)+" | "+String(yMultiplier));
    Serial.println("currentPixel: "+String(currentPixel_x)+" | "+String(currentPixel_y));
    updatePos();
    delay(10);
  }
}
