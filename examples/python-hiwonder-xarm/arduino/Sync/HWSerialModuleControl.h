/******************************************************
* FileName:      HWSerialModuleControl.h
* Company:     Hiwonder
* Date:          2020/05/15  11:12
 *Last Modification Date: 202005151938
* www.hiwonder.com
*****************************************************/


#ifndef HWSERIALMODULECONTROL_H
#define HWSERIALMODULECONTROL_H
#include "LobotSerialServoControl.h"

#define HIWONDER_SERVO_IS_MODULE_ID       0xFC//当舵机的ID号为本值时，视为模块类的固定ID

#define SYN_ID_WRITE        1//同步器的一些指令
#define SYN_ID_READ         2
#define SYN_POS_READ        3


class HWSerialModuleControl: public LobotSerialServoControl
{
  public:
    HWSerialModuleControl(SoftwareSerial &A, uint8_t moduleID):LobotSerialServoControl(A),moduleID(moduleID){};
    HWSerialModuleControl(SoftwareSerial &A,int receiveEnablePin, int transmitEnablePin, uint8_t moduleID):LobotSerialServoControl(A,receiveEnablePin,transmitEnablePin),moduleID(moduleID){};
    HWSerialModuleControl(HardwareSerial &A, uint8_t moduleID):LobotSerialServoControl(A),moduleID(moduleID){};
    HWSerialModuleControl(HardwareSerial &A,int receiveEnablePin, int transmitEnablePin, uint8_t moduleID):LobotSerialServoControl(A,receiveEnablePin,transmitEnablePin),moduleID(moduleID){};

    int16_t getSyncPosition(uint8_t id);//此id区别于 moduleID，是同步器模块之间的id
    void getSyncPositionAll(int16_t *pos);
    void setSyncID(uint8_t id);
    int getSyncID(void);//id值为广播指令0xFE，一次只能连接1个获取ID
    
  private:
    uint8_t moduleID;//区别于 #define HIWONDER_SERVO_IS_MODULE_ID       0xFC
                      //同步器这类的moduleID为1
};


#endif
