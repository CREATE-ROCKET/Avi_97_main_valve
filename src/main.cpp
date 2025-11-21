// 1がmain,2がF/D
#include <Arduino.h>
#include <IcsHardSerialClass.h>
#include "CANCREATE.h"
#define SERIAL_DEBUG

#define SendingServoAngleId 0x401
#define ReceivingServoAngleId 0x300
#define ReceivingServoAngleRequestId 0x301
#define ValveOpenId 0x10b
#define RXFD 19
#define TXFD 18
#define LED 32
#define EMG 16
#define CAN_TX 15
#define CAN_RX 13

constexpr byte EN_PIN = 17; // 基板21
constexpr long BAUDRATE = 115200;
constexpr int TIMEOUT = 1000;                                  // 通信できてないか確認用にわざと遅めに設定
IcsHardSerialClass krs(&Serial2, EN_PIN, BAUDRATE, TIMEOUT);   // インスタンス＋ENピン(17番ピン)およびUARTの指定

float kakudo_o = 58;  // open 角度
float kakudo_c = -77; // close 角度
float kakudo_r = 0;   // recieve 角度
int pos_o = kakudo_o * 8000 / 270 + 7000;
int pos_c = kakudo_c * 8000 / 270 + 7000;
int pos_r = 0;
int maeposr = pos_r; /*以下でバウンス*/
float currentPos = 0;
float Pos = 0;

long count = 0;
int count_EMG = 0;
bool flag_EMG = 0;
int count_free = 0;
bool flag_free = 0;

CAN_CREATE CAN(true);
void setup()
{
  Serial.begin(115200);
  // 100 kbpsでCANを動作させる
  if (CAN.begin(100E3, CAN_RX, CAN_TX))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
  Serial.println("I am a CAN sender");
  pinMode(LED, OUTPUT);
  pinMode(EMG, INPUT);
  // サーボモータの通信初期設定
  Serial2.begin(115200, SERIAL_8N1, RXFD, TXFD);
  krs.begin(); // サーボモータの通信初期設定
  digitalWrite(LED, HIGH);
  krs.setFree(0);
}

void getandsendPos()
{
  currentPos = krs.getPos(0);
  Pos = (currentPos - 7000) / 8000 * 270;
  uint8_t rdata[4];
  rdata[0] = Pos;
  rdata[1] = abs(kakudo_r);
  rdata[2] = 16;
  rdata[3] = 13;
  if (CAN.sendData(SendingServoAngleId, rdata, 4))
  {
    Serial.println("failed to send CAN data");
  }
}

void loop()
{
  if (!flag_EMG)
  {
    if (CAN.available())
    {
      can_return_t message;
      if (!CAN.readWithDetail(&message))
      {
        switch (message.id)
        {
        case ReceivingServoAngleId:
          kakudo_r = message.data[0] - 128;
          pos_r = kakudo_r * 8000 / 270 + 7000;
          if (kakudo_r > 64)
          {
            digitalWrite(LED, HIGH);
          }
          else
          {
            digitalWrite(LED, LOW);
          }
          if (message.id == ReceivingServoAngleRequestId)
          {
          }
          Serial.println("m");
          break;
        case ReceivingServoAngleRequestId:
          getandsendPos();
          break;
        case ValveOpenId:
          krs.setPos(0, pos_r);
          break;
        }
        if (maeposr != pos_r)
        {
          krs.setPos(0, pos_r); // 位置指令 任意
          maeposr = pos_r;
          count_free = 0;
          flag_free = 1;
        }

      }
    }

    if (flag_free)
    {
      count_free++;
    }
    if (count_free > 1000)
    {
      krs.setFree(0);
      flag_free = 0;
    }
    if (digitalRead(EMG) == HIGH)
    {
      count_EMG++;
      if (count_EMG > 3000) // ダンプ試験はここを変える
      {
        flag_EMG = 1;
      }
    }
    else
    {
      count_EMG = 0;
    }
    if (count % 100 == 1)
    {
      getandsendPos();
    }
    digitalWrite(LED, digitalRead(LED) ^ 1);
  }
  else
  {
    Serial.println("EMG detected, stopping servo.");
    if (digitalRead(EMG) == LOW)
    {
      count_EMG = 0;
      flag_EMG = 0;
    }
  }
  ++count;
  delay(50);
#ifdef SERIAL_DEBUG
  if (Serial.available())
  {
    int input = Serial.read();
    Serial.print("Input: ");
    Serial.println(input);
    if (input == 'o')
    {
      Serial.println("Open position selected");
      Serial.println(krs.setPos(0, pos_o));
      Serial.println(krs.getPos(0));
      kakudo_r = kakudo_o;
      maeposr = pos_o;
    }
    else if (input == 'c')
    {
      krs.setPos(0, pos_c);
      kakudo_r = kakudo_c;
      maeposr = pos_c;
    }
  }
  delay(10);
#endif
}
