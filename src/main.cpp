// Main Valve Control
#include <Arduino.h>
#include <IcsHardSerialClass.h>
#include "CANCREATE.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#define SERIAL_DEBUG

// --- CAN ID Definitions ---
#define CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL 0x102
#define CAN_ID_RECV_MAIN_VALVE_ANGLE 0x105
#define ValveOpenId 0x10b

#define RX_MAIN_VALVE 22
#define TX_MAIN_VALVE 21
#define LED 32 // s3にはない
#define EMG 14
#define CAN_TX 15
#define CAN_RX 13
// 論理icは5V駆動
constexpr byte EN_PIN = 12; // 基板21
constexpr long BAUDRATE = 115200;
constexpr int TIMEOUT = 1000;                                // 通信できてないか確認用にわざと遅めに設定 (ms)
IcsHardSerialClass krs(&Serial2, EN_PIN, BAUDRATE, TIMEOUT); // インスタンス＋ENピン(17番ピン)およびUARTの指定

// 可動範囲は3500～11500
constexpr int openAngle = 135;
constexpr int closeAngle = -9;
constexpr int openPosition = openAngle * 8000 / 270 + 7000;
constexpr int closePosition = closeAngle * 8000 / 270 + 7000;
int targetAngle = 0;
int currentTargetPosition = 0;
int lastSentPosition = currentTargetPosition;
float currentPosition = 0;
float currentAngle = 0;
int pendingTargetPosition = currentTargetPosition;



enum SystemState
{
  NORMAL,
  EMG_ACTIVE
};
SystemState currentState = NORMAL;

SemaphoreHandle_t positionMutex;

long count = 0;
int count_EMG = 0;
int count_free = 0;
bool flag_free = 0;

CAN_CREATE CAN(true);

void canTask(void *pvParameters);

void setup()
{
  Serial.begin(115200);
  // 100 kbpsでCANを動作させる
  if (CAN.begin(100E3, CAN_RX, CAN_TX))
  {
    // Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
  Serial.println("I am a CAN sender");
  // pinMode(LED, OUTPUT);
  pinMode(EMG, INPUT);
  // サーボモータの通信初期設定
  Serial2.begin(115200, SERIAL_8N1, RX_MAIN_VALVE, TX_MAIN_VALVE);
  krs.begin(); // サーボモータの通信初期設定
  digitalWrite(LED, HIGH);
  krs.setFree(0);

  positionMutex = xSemaphoreCreateMutex();

  xTaskCreateUniversal(
      canTask,
      "CAN_Task",
      4096,
      NULL,
      1,
      NULL,
      0);
}

void getandsendPos()
{
  currentPosition = krs.getPos(0);
  currentAngle = (currentPosition - 7000) / 8000 * 270;
  uint8_t rdata[4];
  memcpy(rdata, &currentAngle, sizeof(float));
  CAN.sendData(CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL, rdata, 4);
}

void canTask(void *pvParameters)
{
  while (1)
  {
    if (CAN.available())
    {
      can_return_t message;
      if (!CAN.readWithDetail(&message))
      {
        switch (message.id)
        {
        case CAN_ID_RECV_MAIN_VALVE_ANGLE:
          xSemaphoreTake(positionMutex, portMAX_DELAY);
          targetAngle = message.data[0] - 120; /*assume message.data[0] == 255(open) or 111(close) */
          pendingTargetPosition = targetAngle * 8000 / 270 + 7000;

          xSemaphoreGive(positionMutex);

          if (targetAngle > 64)
          {
            digitalWrite(LED, HIGH); // openrequest
          }
          else
          {
            digitalWrite(LED, LOW); // closerequest
          }
          break;
        case ValveOpenId:
          xSemaphoreTake(positionMutex, portMAX_DELAY);
          krs.setPos(0, openPosition);
          xSemaphoreGive(positionMutex);
          break;
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void loop()
{
  switch (currentState)
  {
  case NORMAL:
    xSemaphoreTake(positionMutex, portMAX_DELAY);
    if (lastSentPosition != pendingTargetPosition)
    {
      currentTargetPosition = pendingTargetPosition;
      krs.setPos(0, currentTargetPosition); // 位置指令 任意
      lastSentPosition = currentTargetPosition;
      count_free = 0;
      flag_free = 1;
    }
    xSemaphoreGive(positionMutex);

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
        currentState = EMG_ACTIVE;
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
    break;
  case EMG_ACTIVE:
    Serial.println("EMG detected, stopping servo.");
    if (digitalRead(EMG) == LOW)
    {
      count_EMG = 0;
      currentState = NORMAL;
    }
    break;
  }
  ++count;
  delay(500);
#ifdef SERIAL_DEBUG
  if (Serial.available())
  {
    int input = Serial.read();
    Serial.print("Input: ");
    Serial.println(input);

    xSemaphoreTake(positionMutex, portMAX_DELAY);
    if (input == 'o')
    {
      Serial.println("Open position requested");
      pendingTargetPosition = openPosition;

    }
    else if (input == 'c')
    {
      Serial.println("Close position requested");
      pendingTargetPosition = closePosition;

    }
    xSemaphoreGive(positionMutex);
  }
  delay(10);
#endif
}
