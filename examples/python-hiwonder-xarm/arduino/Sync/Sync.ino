#include <SoftwareSerial.h>
#include "HWSerialModuleControl.h"
 
#define BTH_RX 11
#define BTH_TX 12

#define SERVO_SERIAL_RX 10
#define SERVO_SERIAL_TX 3
#define SERVO_SERIAL_RX_CON 4
#define SERVO_SERIAL_TX_CON 2

#define LED         6
#define KEY1        8
#define KEY2        7
#define LED_ON      0
#define LED_OFF     1

#define   SERVO_TYPE_PWM      0
#define   SERVO_TYPE_BUS      1

#define MAX_ATTEMPTS 20

SoftwareSerial ServoSerial(SERVO_SERIAL_RX, SERVO_SERIAL_TX);
HWSerialModuleControl sync(ServoSerial,SERVO_SERIAL_RX_CON,SERVO_SERIAL_TX_CON,1);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  sync.OnInit();
}

void loop() {
  static int16_t pos[6], posClean[6];
  bool ok = false;
  uint8_t attempts = 0;
  for (uint8_t i = 0; i < 6; i++)
    posClean[i] = -1;
  while (!ok && attempts < MAX_ATTEMPTS) {
    sync.getSyncPositionAll(pos);
    for(uint8_t i=0; i<=5;i++){
      if (pos[i] >= 0)
        posClean[i] = pos[i];
    }
    ok = true;
    for (uint8_t i = 0; i < 6; i++)
      if (posClean[i] < 0)
        ok = false;
    attempts++;
  }
  Serial.write((uint8_t*) posClean, 6 * sizeof(int16_t));
  // for(uint8_t i=1; i<=6;i++){
  //   Serial.print(i);
  //   Serial.print(":");
  //   Serial.print(posClean[i]);
  //   Serial.print("\t");
  // }
  Serial.print("\n");
}

int16_t PosConvert(int16_t pos, uint8_t servo_type = SERVO_TYPE_BUS, int16_t middle_pos = 500, bool flip = false, float rate = 1.0, uint16_t offset = 0)
{
  float p = middle_pos + (pos - middle_pos)*rate + offset;
  int16_t pp = int16_t(p);
  if(servo_type == SERVO_TYPE_BUS){
    if(pp<0)pp = 0;
    else if(pp>1000)pp = 1000;
    if(flip)
      return 1000 - int16_t(pp);
    return int16_t(pp);
  }else if(servo_type == SERVO_TYPE_PWM){
    if(pp<400)pp = 400;
    else if(pp>2600)pp = 2600;
    if(flip)
      return 3000 - int16_t(pp);
    return int16_t(pp);
  }
}
