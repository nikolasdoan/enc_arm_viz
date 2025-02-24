#include"TLE5012.h"

unsigned char ssc_delay = 0; // SSC clock frequency adjust, 0 = fastest, 255 = slowest
TLE5012 enc(7, 6, 8, ssc_delay);

void setup() {
  Serial.begin(115200);
  enc.begin();
}

void loop(){
  uint16_t encAng = enc.readAngle();
  if(encAng == enc.invalidAngle)
  {
    Serial.println("Invalid Angle");
    return false;
  }
  else
  {
    Serial.println(encAng);
  }
  delay(500);
}

