#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"
#include "../ece-707-lora-common/PacketConfig.h"

// --- HARDWARE & CONFIG ---
HT_st7735 st7735;
#define USER_BUTTON 0

// --- NODE STATE ---
uint8_t myID = 1;
bool isInvisible = false;
uint8_t knownNodes[50];
int knownCount = 0;

// --- BUTTON LOGIC ---
unsigned long buttonPressTime = 0;
bool buttonActive = false;

typedef enum
{
  MODE_LISTENING,
  MODE_TEST_RESPONDER,
  MODE_ACTIVE_SEARCHER
} NodeMode_t;
NodeMode_t currentMode = MODE_LISTENING;

// --- RX/TX VARS ---
uint8_t rxBuffer[255];
int16_t rxRssi;
volatile bool packetReceived = false;
volatile bool txDone = false;
uint32_t lastRxTime = 0;

// --- HELPER FUNCTIONS ---
void UpdateScreen()
{
  if (isInvisible)
  {
    st7735.st7735_fill_screen(ST7735_RED);
    st7735.st7735_write_str(10, 30, "INVISIBLE", Font_11x18, ST7735_BLACK);
    return;
  }
  st7735.st7735_fill_screen(ST7735_BLACK);
  char buf[30];
  sprintf(buf, "Node ID: %d", myID);
  st7735.st7735_write_str(0, 0, buf, Font_11x18, ST7735_CYAN);

  if (currentMode == MODE_LISTENING)
    st7735.st7735_write_str(0, 20, "Listening...", Font_7x10, ST7735_WHITE);
  if (currentMode == MODE_TEST_RESPONDER)
    st7735.st7735_write_str(0, 20, "SF Testing...", Font_7x10, ST7735_MAGENTA);
  if (currentMode == MODE_ACTIVE_SEARCHER)
    st7735.st7735_write_str(0, 20, "Scanning...", Font_7x10, ST7735_YELLOW); // Replaced ORANGE

  st7735.st7735_write_str(0, 40, "Known Neighbors:", Font_7x10, ST7735_YELLOW);
  for (int i = 0; i < knownCount && i < 5; i++)
  {
    sprintf(buf, "- ID: %d", knownNodes[i]);
    st7735.st7735_write_str(0, 55 + (i * 10), buf, Font_7x10, ST7735_WHITE);
  }
}

void ConfigRadioSF(int sf)
{
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, sf, LORA_CODINGRATE, 8, false, true, 0, 0, false, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, sf, LORA_CODINGRATE, 0, 8, 0, false, 0, true, 0, 0, false, true);
}

void SendPacket(uint8_t dest, uint8_t type, uint8_t d1, uint8_t d2)
{
  uint8_t packet[10];
  packet[0] = dest;
  packet[1] = myID;
  packet[2] = type;
  packet[3] = d1;
  packet[4] = d2;

  txDone = false;
  Radio.Send(packet, 5);

  // BLOCKING WAIT
  unsigned long start = millis();
  while (!txDone && millis() - start < 1000)
  {
    Radio.IrqProcess();
  }
}

bool isKnown(uint8_t id)
{
  for (int i = 0; i < knownCount; i++)
    if (knownNodes[i] == id)
      return true;
  return false;
}

void addKnown(uint8_t id)
{
  if (!isKnown(id) && knownCount < 50)
  {
    knownNodes[knownCount++] = id;
    UpdateScreen();
  }
}

// --- INTERRUPTS ---
void OnTxDone()
{
  txDone = true;
  Radio.Rx(RX_TIMEOUT_VALUE);
}
void OnTxTimeout()
{
  txDone = true;
  Radio.Rx(RX_TIMEOUT_VALUE);
}
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  if (isInvisible)
    return;
  memcpy(rxBuffer, payload, size);
  rxRssi = rssi;
  packetReceived = true;
}
void OnRxTimeout()
{
  Radio.Rx(RX_TIMEOUT_VALUE);
}

void setup()
{
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  st7735.st7735_init();
  pinMode(USER_BUTTON, INPUT);

  RadioEvents_t RadioEvents;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  ConfigRadioSF(12);

  UpdateScreen();
  Radio.Rx(RX_TIMEOUT_VALUE);
}

void loop()
{
  // --- BUTTON HANDLING ---
  if (digitalRead(USER_BUTTON) == LOW)
  {
    if (!buttonActive)
    {
      buttonActive = true;
      buttonPressTime = millis();
    }
  }
  else
  {
    if (buttonActive)
    {
      unsigned long duration = millis() - buttonPressTime;
      buttonActive = false;
      if (duration > 1000)
      {
        isInvisible = !isInvisible;
      }
      else
      {
        myID++;
        if (myID > 20)
          myID = 1;
      }
      UpdateScreen();
      Radio.Sleep();
      Radio.Rx(RX_TIMEOUT_VALUE);
    }
  }

  if (isInvisible)
  {
    delay(100);
    return;
  }

  Radio.IrqProcess();

  // Packet Handling
  if (packetReceived)
  {
    packetReceived = false;
    uint8_t dest = rxBuffer[0];
    uint8_t sender = rxBuffer[1];
    uint8_t type = rxBuffer[2];

    if (dest != myID && dest != BROADCAST_ADDR)
    {
      Radio.Rx(RX_TIMEOUT_VALUE);
      return;
    }

    if (type == PKT_DISCOVERY_REQ)
    {
      delay(random(100, BACKOFF_MAX_DELAY));
      SendPacket(sender, PKT_DISCOVERY_RESP, 0, 0);
      addKnown(sender);
    }
    else if (type == PKT_SF_TEST)
    {
      currentMode = MODE_TEST_RESPONDER;
      UpdateScreen();
      int receivedSF = rxBuffer[3];
      ConfigRadioSF(receivedSF);
      delay(10);
      SendPacket(sender, PKT_SF_ACK, 0, 0);
      lastRxTime = millis();
      addKnown(sender);
    }
    else if (type == PKT_CMD_SEARCH)
    {
      currentMode = MODE_ACTIVE_SEARCHER;
      UpdateScreen();
      ConfigRadioSF(12);
      SendPacket(BROADCAST_ADDR, PKT_DISCOVERY_REQ, 0, 0);

      unsigned long scanStart = millis();
      while (millis() - scanStart < DISCOVERY_TIMEOUT)
      {
        Radio.IrqProcess();
        if (packetReceived)
        {
          packetReceived = false;
          if (rxBuffer[2] == PKT_DISCOVERY_RESP && rxBuffer[0] == myID)
          {
            uint8_t foundID = rxBuffer[1];
            SendPacket(BASE_STATION_ID, PKT_REPORT_NODE, foundID, 0);
            addKnown(foundID);
            delay(200);
          }
          Radio.Rx(RX_TIMEOUT_VALUE);
        }
      }
      SendPacket(BASE_STATION_ID, PKT_SEARCH_DONE, 0, 0);
      currentMode = MODE_LISTENING;
      UpdateScreen();
    }

    // IMPORTANT: Restart RX if we handled a packet but didn't send a response (which auto-restarts RX)
    Radio.Rx(RX_TIMEOUT_VALUE);
  }

  if (currentMode == MODE_TEST_RESPONDER && millis() - lastRxTime > 5000)
  {
    currentMode = MODE_LISTENING;
    ConfigRadioSF(12);
    UpdateScreen();
    Radio.Rx(RX_TIMEOUT_VALUE);
  }
}