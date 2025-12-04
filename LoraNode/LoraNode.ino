#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"
#include "../ece-707-lora-common/PacketConfig.h"

// --- HARDWARE & CONFIG ---
HT_st7735 st7735;
#define USER_BUTTON 0

// --- NODE STATE ---
uint8_t myID = 1; // Default ID, changeable via button
bool isInvisible = false;
bool connectedToNetwork = false;
uint8_t knownNodes[50];
int knownCount = 0;

// --- BUTTON LOGIC ---
unsigned long buttonPressTime = 0;
bool buttonActive = false;

// --- STATE MACHINE ---
typedef enum { 
  MODE_LISTENING, 
  MODE_TEST_RESPONDER, 
  MODE_ACTIVE_SEARCHER 
} NodeMode_t;

NodeMode_t currentMode = MODE_LISTENING;

// --- SF TEST VARS ---
uint32_t lastRxTime = 0;

// --- RX BUFFERS ---
uint8_t rxBuffer[255];
int16_t rxRssi;
bool packetReceived = false;

// --- HELPER FUNCTIONS ---
void UpdateScreen() {
  if(isInvisible) {
     st7735.st7735_fill_screen(ST7735_RED);
     st7735.st7735_write_str(10, 30, "INVISIBLE", Font_11x18, ST7735_BLACK);
     return;
  }

  st7735.st7735_fill_screen(ST7735_BLACK);
  char buf[30];
  sprintf(buf, "Node ID: %d", myID);
  st7735.st7735_write_str(0, 0, buf, Font_11x18, ST7735_CYAN);
  
  if(currentMode == MODE_LISTENING) st7735.st7735_write_str(0, 20, "Listening...", Font_7x10, ST7735_WHITE);
  if(currentMode == MODE_TEST_RESPONDER) st7735.st7735_write_str(0, 20, "SF Testing...", Font_7x10, ST7735_MAGENTA);
  if(currentMode == MODE_ACTIVE_SEARCHER) st7735.st7735_write_str(0, 20, "Scanning...", Font_7x10, ST7735_ORANGE);
  
  st7735.st7735_write_str(0, 40, "Known Neighbors:", Font_7x10, ST7735_YELLOW);
  for(int i=0; i<knownCount && i<5; i++) {
     sprintf(buf, "- ID: %d", knownNodes[i]);
     st7735.st7735_write_str(0, 55 + (i*10), buf, Font_7x10, ST7735_WHITE);
  }
}

void ConfigRadioSF(int sf) {
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, sf, LORA_CODINGRATE, 8, false, true, 0, 0, false, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, sf, LORA_CODINGRATE, 0, 8, 0, false, 0, true, 0, 0, false, true);
}

void SendPacket(uint8_t dest, uint8_t type, uint8_t d1, uint8_t d2) {
  uint8_t packet[10];
  packet[0] = dest;
  packet[1] = myID;
  packet[2] = type;
  packet[3] = d1;
  packet[4] = d2;
  Radio.Send(packet, 5);
}

bool isKnown(uint8_t id) {
  for(int i=0; i<knownCount; i++) if(knownNodes[i] == id) return true;
  return false;
}

void addKnown(uint8_t id) {
  if(!isKnown(id) && knownCount < 50) {
    knownNodes[knownCount++] = id;
    UpdateScreen();
  }
}

// --- INTERRUPTS ---
void OnTxDone() { Radio.Rx(RX_TIMEOUT_VALUE); }
void OnTxTimeout() { Radio.Rx(RX_TIMEOUT_VALUE); }
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  if(isInvisible) return; // Ignore everything
  memcpy(rxBuffer, payload, size);
  rxRssi = rssi;
  packetReceived = true;
}

void setup() {
  Serial.begin(115200);
  Mcu.begin();
  st7735.st7735_init();
  pinMode(USER_BUTTON, INPUT);
  
  RadioEvents_t RadioEvents;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  ConfigRadioSF(12);
  
  UpdateScreen();
  Radio.Rx(RX_TIMEOUT_VALUE);
}

void loop() {
  // --- BUTTON HANDLING ---
  if (digitalRead(USER_BUTTON) == LOW) { // Pressed
    if (!buttonActive) {
      buttonActive = true;
      buttonPressTime = millis();
    }
  } else { // Released
    if (buttonActive) {
      unsigned long duration = millis() - buttonPressTime;
      buttonActive = false;
      
      if (duration > 1000) {
        // Long Press: Toggle Invisible
        isInvisible = !isInvisible;
      } else {
        // Short Press: Increment ID
        myID++;
        if(myID > 20) myID = 1; // Wrap around
      }
      UpdateScreen();
      // Reset Radio to clear buffers
      Radio.Sleep();
      Radio.Rx(RX_TIMEOUT_VALUE);
    }
  }

  // --- LOGIC ---
  if(isInvisible) {
    delay(100);
    return;
  }

  Radio.IrqProcess(); // Must call often

  // Packet Handling
  if (packetReceived) {
    packetReceived = false;
    uint8_t dest = rxBuffer[0];
    uint8_t sender = rxBuffer[1];
    uint8_t type = rxBuffer[2];

    // Filter: Is it for me or Broadcast?
    if (dest != myID && dest != BROADCAST_ADDR) {
      Radio.Rx(RX_TIMEOUT_VALUE);
      return; 
    }

    // --- 1. DISCOVERY REQUEST ---
    if (type == PKT_DISCOVERY_REQ) {
       // Only reply if I haven't connected to this specific sender before?
       // Logic: Always reply to discovery, let the Base decide if I'm new.
       // Random Backoff
       delay(random(100, BACKOFF_MAX_DELAY));
       SendPacket(sender, PKT_DISCOVERY_RESP, 0, 0);
       addKnown(sender); // I know about this sender now
    }
    
    // --- 2. SF TEST (Passive) ---
    else if (type == PKT_SF_TEST) {
       currentMode = MODE_TEST_RESPONDER;
       UpdateScreen();
       int receivedSF = rxBuffer[3];
       ConfigRadioSF(receivedSF); // Match sender's SF
       delay(10);
       SendPacket(sender, PKT_SF_ACK, 0, 0); // Immediate ACK
       lastRxTime = millis();
       addKnown(sender);
    }

    // --- 3. COMMAND SEARCH (Active BFS) ---
    else if (type == PKT_CMD_SEARCH) {
       // Base Station told me to find MY neighbors
       currentMode = MODE_ACTIVE_SEARCHER;
       UpdateScreen();
       
       // Stop listening, start scanning
       ConfigRadioSF(12);
       SendPacket(BROADCAST_ADDR, PKT_DISCOVERY_REQ, 0, 0);
       
       // Wait for responses (similar to Base Station logic, but simpler)
       unsigned long scanStart = millis();
       while(millis() - scanStart < DISCOVERY_TIMEOUT) {
          Radio.IrqProcess();
          // We need to poll checking for interrupt flag here manually or use a flag
          // But since we are in a blocking loop, we need to be careful.
          // For simplicity, we assume `packetReceived` flag updates via interrupt in background
          if(packetReceived) {
             packetReceived = false;
             if(rxBuffer[2] == PKT_DISCOVERY_RESP && rxBuffer[0] == myID) {
                uint8_t foundID = rxBuffer[1];
                // Report to Base immediately
                SendPacket(BASE_STATION_ID, PKT_REPORT_NODE, foundID, 0);
                addKnown(foundID);
                delay(200); // Give Base time to process
             }
             Radio.Rx(RX_TIMEOUT_VALUE);
          }
       }
       
       // Report Done
       SendPacket(BASE_STATION_ID, PKT_SEARCH_DONE, 0, 0);
       currentMode = MODE_LISTENING;
       UpdateScreen();
    }
    
    // Default: Keep listening
    Radio.Rx(RX_TIMEOUT_VALUE);
  }
  
  // Timeout Reset for Test Mode
  if (currentMode == MODE_TEST_RESPONDER && millis() - lastRxTime > 5000) {
     currentMode = MODE_LISTENING;
     ConfigRadioSF(12); // Reset to default
     UpdateScreen();
     Radio.Rx(RX_TIMEOUT_VALUE);
  }
}