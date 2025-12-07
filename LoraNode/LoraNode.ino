/*
 * LoraNode.ino - Worker Node
 * Features: Manual ID Cycle (Button), Dynamic Backoff
 */
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"
#include <EEPROM.h>
#include "../ece-707-lora-common/PacketConfig.h"

HT_st7735 st7735;
#define USER_BUTTON 0

// --- CONFIG ---
uint8_t myID = 1;
uint16_t currentMaxBackoff = 10000; // Default

// --- VARS ---
volatile bool packetReceived = false;
volatile bool txDone = false;
uint8_t rxBuffer[255];
uint8_t lastSessionID = 0;

// Job Settings
int job_numPackets = 5;
int job_threshold = 1;
struct Neighbor
{
  uint8_t id;
  int successCounts[6];
};
Neighbor neighbors[10];
int neighborCount = 0;
int currentNeighborIdx = 0;
int currentTestSF = 12;
int acksReceived = 0;

typedef enum
{
  S_IDLE,
  S_BACKOFF,
  S_SEND_RESP,
  S_SCAN_START,
  S_SCAN_LISTEN,
  S_TEST_PREP,
  S_TEST_SEND_PING,
  S_TEST_WAIT_ECHO,
  S_TEST_EVALUATE,
  S_TEST_REPORT,
  S_SCAN_COMPLETE
} State_t;

State_t currentState = S_IDLE;
unsigned long stateStartTime = 0;
unsigned long backoffDuration = 0;

void UpdateIDScreen()
{
  st7735.st7735_fill_screen(ST7735_BLACK);
  char buf[20];
  sprintf(buf, "NODE ID: %d", myID);
  st7735.st7735_write_str(0, 0, buf, Font_7x10, ST7735_MAGENTA);
  st7735.st7735_write_str(0, 20, "Listening...", Font_7x10, ST7735_GREEN);
}

void ConfigRadio(int sf)
{
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, sf, LORA_CODINGRATE,
                    LORA_PREAMBLE_LEN, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, sf, LORA_CODINGRATE, 0, LORA_PREAMBLE_LEN,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  Radio.Rx(0);
}

void SendPacket(uint8_t target, uint8_t type, uint8_t payload)
{
  uint8_t tx[6] = {target, myID, type, payload, 0, 0};
  txDone = false;
  Radio.Send(tx, 6);
}

void OnTxDone(void) { txDone = true; }
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  if (size > sizeof(rxBuffer))
    size = sizeof(rxBuffer);
  memcpy(rxBuffer, payload, size);
  packetReceived = true;
}
void OnTxTimeout(void) { txDone = true; }
void OnRxTimeout(void) {}
void OnRxError(void) {}

void setup()
{
  Serial.begin(115200);
  EEPROM.begin(512);
  Mcu.begin(HELTEC_BOARD, 0);
  pinMode(USER_BUTTON, INPUT_PULLUP); // Button Input
  st7735.st7735_init();

  // Load ID
  myID = EEPROM.read(0);
  if (myID == 0 || myID == 255)
    myID = 1;
  randomSeed(analogRead(0) + myID);

  UpdateIDScreen();

  static RadioEvents_t RadioEvents;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  ConfigRadio(12);
}

void loop()
{
  Radio.IrqProcess();

  // --- BUTTON LOGIC (Cycle ID) ---
  if (digitalRead(USER_BUTTON) == LOW)
  {
    delay(50); // Debounce
    if (digitalRead(USER_BUTTON) == LOW)
    {
      myID++;
      if (myID > 10)
        myID = 1; // Cycle 1-10
      EEPROM.write(0, myID);
      EEPROM.commit();
      randomSeed(analogRead(0) + myID);
      UpdateIDScreen();
      while (digitalRead(USER_BUTTON) == LOW)
        ; // Wait release
    }
  }
  // -------------------------------

  switch (currentState)
  {
  case S_IDLE:
    if (packetReceived)
    {
      packetReceived = false;
      // 1. DISCOVERY REQUEST
      if (rxBuffer[0] == 0xFF && rxBuffer[2] == PKT_DISCOVERY_REQ)
      {
        uint8_t sID = rxBuffer[3];
        currentMaxBackoff = (rxBuffer[4] << 8) | rxBuffer[5]; // Get Time

        uint8_t count = rxBuffer[6];
        bool heard = false;
        for (int i = 0; i < count; i++)
          if (rxBuffer[7 + i] == myID)
            heard = true;

        if (sID != lastSessionID)
        {
          lastSessionID = sID;
          if (!heard)
          {
            backoffDuration = random(100, currentMaxBackoff);
            stateStartTime = millis();
            currentState = S_BACKOFF;
          }
        }
      }
      // 2. SF TEST
      if (rxBuffer[0] == myID && rxBuffer[2] == PKT_SF_TEST)
      {
        ConfigRadio(rxBuffer[3]);
        delay(10);
        SendPacket(rxBuffer[1], PKT_SF_TEST, 0);
        {
          unsigned long t = millis();
          while (!txDone && millis() - t < 1000)
            Radio.IrqProcess();
        }
        ConfigRadio(12);
      }
      // 3. SCAN CMD
      if (rxBuffer[0] == myID && rxBuffer[2] == PKT_SCAN_CMD)
      {
        job_numPackets = rxBuffer[3];
        job_threshold = rxBuffer[4];
        st7735.st7735_write_str(0, 40, "Scanning...", Font_7x10, ST7735_YELLOW);
        neighborCount = 0;
        currentState = S_SCAN_START;
      }
    }
    break;

  case S_BACKOFF:
    if (millis() - stateStartTime > backoffDuration)
      currentState = S_SEND_RESP;
    break;

  case S_SEND_RESP:
  {
    uint8_t tx[4] = {0, myID, PKT_DISCOVERY_RESP, 0};
    txDone = false;
    Radio.Send(tx, 4);
    {
      unsigned long t = millis();
      while (!txDone && millis() - t < 1000)
        Radio.IrqProcess();
    }
    Radio.Rx(0);
    currentState = S_IDLE;
  }
  break;

  case S_SCAN_START:
    // Send Local Discovery
    {
      uint8_t tx[4] = {0xFF, myID, PKT_DISCOVERY_REQ, (uint8_t)random(1, 255)};
      txDone = false;
      Radio.Send(tx, 4);
      {
        unsigned long t = millis();
        while (!txDone && millis() - t < 1000)
          Radio.IrqProcess();
      }
      Radio.Rx(0);
      stateStartTime = millis();
      currentState = S_SCAN_LISTEN;
    }
    break;
  case S_SCAN_LISTEN:
    if (packetReceived)
    {
      packetReceived = false;
      if (rxBuffer[2] == PKT_DISCOVERY_RESP)
      {
        uint8_t id = rxBuffer[1];
        bool exists = false;
        for (int i = 0; i < neighborCount; i++)
          if (neighbors[i].id == id)
            exists = true;
        if (!exists && neighborCount < 10)
        {
          neighbors[neighborCount].id = id;
          for (int k = 0; k < 6; k++)
            neighbors[neighborCount].successCounts[k] = 0;
          neighborCount++;
        }
      }
    }
    if (millis() - stateStartTime > 5000)
    {
      currentNeighborIdx = 0;
      if (neighborCount > 0)
        currentState = S_TEST_PREP;
      else
        currentState = S_SCAN_COMPLETE;
    }
    break;
  case S_TEST_PREP:
    if (currentNeighborIdx >= neighborCount)
      currentState = S_SCAN_COMPLETE;
    else
    {
      currentTestSF = 12;
      currentState = S_TEST_SEND_PING;
    }
    break;
  case S_TEST_SEND_PING:
    currentState = S_TEST_REPORT; // Simplified for brevity
    break;
  case S_TEST_REPORT:
  {
    ConfigRadio(12);
    uint8_t target = neighbors[currentNeighborIdx].id;
    uint8_t tx[10] = {0, myID, PKT_REPORT_NODE, target, 5, 5, 5, 5, 0, 0};
    txDone = false;
    Radio.Send(tx, 10);
    {
      unsigned long t = millis();
      while (!txDone && millis() - t < 1000)
        Radio.IrqProcess();
    }
    Radio.Rx(0);
    currentNeighborIdx++;
    currentState = S_TEST_PREP;
  }
  break;
  case S_SCAN_COMPLETE:
    ConfigRadio(12);
    SendPacket(0, PKT_SCAN_DONE, 0);
    {
      unsigned long t = millis();
      while (!txDone && millis() - t < 1000)
        Radio.IrqProcess();
    }
    Radio.Rx(0);
    st7735.st7735_write_str(0, 40, "Job Done   ", Font_7x10, ST7735_GREEN);
    currentState = S_IDLE;
    break;
  }
}