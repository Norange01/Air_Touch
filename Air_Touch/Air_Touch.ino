/**
 * This example turns the ESP32 into a Bluetooth LE Absolute Mouse that clicks
 * the center of the screen every 2 seconds.
 */
#include <BleAbsMouse.h>
#include <math.h>

BleAbsMouse bleAbsMouse;
int state=0; // 0: default; 1: calibration corner1; 2: calibration corner2; 3: cursor calibration
bool prevBtnState = 0;

double endEffector_x = 0;
double endEffector_y = 0;
double endEffector_z = 0;

double origin_x = 0;
double origin_y = 0;
double origin_z = 0;

bool mouseIsEnabled(){
  return digitalRead(19)==1;
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
  pinMode(18, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
}

void loop() {
  if(mouseIsEnabled()){
    if(bleAbsMouse.isConnected()) {
      //Serial.println("Click");
      //bleAbsMouse.click(5000, 5000);
      
    }
    if(prevBtnState==0 && digitalRead(27)==1){
      state++;
      prevBtnState=1;
      if(state==5){
        state=0;
      }
    }
    else if(digitalRead(27)==0){
      prevBtnState=0;
    }
    Serial.println(state);
    //delay(2000);
  }
}
