/**
 * This example turns the ESP32 into a Bluetooth LE Absolute Mouse that clicks
 * the center of the screen every 2 seconds.
 */
#include <BleAbsMouse.h>

BleAbsMouse bleAbsMouse;
int state=0; // 0: default; 1: calibration corner1; 2: calibration corner2; 3: cursor calibration
bool prevBtnState = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  bleAbsMouse.begin();
  pinMode(18, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
}

void loop() {
  if(bleAbsMouse.isConnected()) {
    if(digitalRead(19)==1){
      Serial.println("Click");
      //bleAbsMouse.click(5000, 5000);
    }
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
  delay(2000);
}
