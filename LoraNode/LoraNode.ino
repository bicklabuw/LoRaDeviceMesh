/*
 * LoraNode.ino - Client Node (Discovery + Echo + Button Control)
 * FEATURES: 
 * - Stable Radio Logic (Non-blocking)
 * - Short Press: Change ID
 * - Long Press: Toggle Visibility
 */
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"
#include "../ece-707-lora-common/PacketConfig.h" 

// --- CONFIG ---
uint8_t myID = 1; // Default start ID
bool isInvisible = false;

// --- HARDWARE ---
HT_st7735 st7735;
#define USER_BUTTON 0 // GPIO 0 is the "PRG" button on most Heltec boards

// --- BUTTON VARIABLES ---
unsigned long buttonPressTime = 0;
bool buttonActive = false;
bool updateScreen = true;

// --- RADIO VARIABLES ---
volatile bool packetReceived = false;
volatile bool txDone = false;
uint8_t rxBuffer[255];
uint8_t rxSize = 0;
int16_t lastRssi = 0;

// --- STATE MACHINE ---
typedef enum {
    S_IDLE,
    S_BACKOFF,
    S_SEND_RESP,
    S_PERFORM_SCAN,
    S_REPORT_NEIGHBOR,
    S_REPORT_DONE,
    S_SEND_ECHO
} State_t;

State_t currentState = S_IDLE;
unsigned long stateStartTime = 0;
unsigned long backoffDuration = 0;

// Scan/Echo Variables
int echoSF = 12;
uint8_t foundNeighbors[10];
int neighborCount = 0;
int reportIndex = 0;

// --- HELPER: UPDATE SCREEN ---
void UpdateScreen() {
    st7735.st7735_fill_screen(ST7735_BLACK);
    
    // Header
    char buf[20];
    sprintf(buf, "NODE ID: %d", myID);
    st7735.st7735_write_str(0, 0, buf, Font_7x10, ST7735_CYAN);
    
    // Status
    if (isInvisible) {
        st7735.st7735_write_str(0, 20, "STATUS: HIDDEN", Font_7x10, ST7735_MAGENTA);
        st7735.st7735_write_str(0, 40, "(Ignor. Discovery)", Font_7x10, ST7735_WHITE);
    } else {
        st7735.st7735_write_str(0, 20, "STATUS: VISIBLE", Font_7x10, ST7735_GREEN);
        st7735.st7735_write_str(0, 40, "Listening...", Font_7x10, ST7735_WHITE);
    }
}

// --- RADIO CONFIG ---
void ConfigRadio(int sf) {
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, sf, LORA_CODINGRATE,
                      LORA_PREAMBLE_LEN, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, sf, LORA_CODINGRATE, 0, LORA_PREAMBLE_LEN,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
    Radio.Rx(0);
}

// --- INTERRUPTS ---
void OnTxDone(void) { txDone = true; Radio.Rx(0); }
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    if (size > sizeof(rxBuffer)) size = sizeof(rxBuffer);
    memcpy(rxBuffer, payload, size);
    rxSize = size;
    lastRssi = rssi;
    packetReceived = true;
}
void OnTxTimeout(void) { Radio.Rx(0); }
void OnRxTimeout(void) { }
void OnRxError(void) { }

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, 0);
    
    pinMode(USER_BUTTON, INPUT); // Button is active LOW

    st7735.st7735_init();
    UpdateScreen();

    static RadioEvents_t RadioEvents;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    
    ConfigRadio(12); // Default Listener
}

void loop() {
    Radio.IrqProcess();

    // --- BUTTON LOGIC ---
    // Read button (Active LOW: 0=Pressed, 1=Released)
    if (digitalRead(USER_BUTTON) == LOW) {
        if (!buttonActive) {
            buttonActive = true;
            buttonPressTime = millis();
        }
    } else {
        if (buttonActive) {
            buttonActive = false;
            unsigned long duration = millis() - buttonPressTime;
            
            if (duration > 1000) {
                // LONG PRESS: Toggle Invisible
                isInvisible = !isInvisible;
            } else if (duration > 50) {
                // SHORT PRESS: Change ID
                myID++;
                if (myID > 10) myID = 1; // Cycle 1-10
            }
            UpdateScreen();
        }
    }

    // --- STATE MACHINE ---
    switch (currentState) {
        case S_IDLE:
            if (packetReceived) {
                packetReceived = false;
                
                // CASE 1: Discovery Request
                if ((rxBuffer[0] == 0xFF || rxBuffer[0] == myID) && rxBuffer[2] == PKT_DISCOVERY_REQ) {
                    // ONLY REPLY IF VISIBLE
                    if (!isInvisible) {
                        st7735.st7735_write_str(0, 60, "Discovery Req! ", Font_7x10, ST7735_YELLOW);
                        backoffDuration = random(100, BACKOFF_MAX_DELAY);
                        stateStartTime = millis();
                        currentState = S_BACKOFF;
                    } else {
                        // Log ignored packet?
                        Serial.println("Ignored Discovery (Invisible)");
                    }
                }

                // CASE 2: Remote Scan Command
                else if (rxBuffer[0] == myID && rxBuffer[2] == PKT_SCAN_CMD) {
                    st7735.st7735_write_str(0, 60, "Scanning Area...", Font_7x10, ST7735_MAGENTA);
                    neighborCount = 0;
                    
                    // Broadcast scan
                    uint8_t tx[5] = {0xFF, myID, PKT_DISCOVERY_REQ, 0, 0};
                    ConfigRadio(12);
                    txDone = false;
                    Radio.Send(tx, 5);
                    stateStartTime = millis();
                    currentState = S_PERFORM_SCAN;
                }
                
                // CASE 3: SF Link Test
                else if (rxBuffer[0] == myID && rxBuffer[2] == PKT_SF_TEST) {
                    echoSF = rxBuffer[3]; 
                    st7735.st7735_write_str(0, 60, "Echoing Test... ", Font_7x10, ST7735_YELLOW);
                    currentState = S_SEND_ECHO;
                }
            }
            break;

        case S_BACKOFF:
            if (millis() - stateStartTime > backoffDuration) currentState = S_SEND_RESP;
            break;

        case S_SEND_RESP:
            {
                st7735.st7735_write_str(0, 60, "Sending Hello...", Font_7x10, ST7735_GREEN);
                uint8_t tx[5] = {0, myID, PKT_DISCOVERY_RESP, 0, 0};
                ConfigRadio(12);
                txDone = false;
                Radio.Send(tx, 5);
                unsigned long t = millis();
                while(!txDone && millis()-t < 1000) Radio.IrqProcess();
                
                currentState = S_IDLE;
                st7735.st7735_write_str(0, 60, "               ", Font_7x10, ST7735_BLACK);
            }
            break;
            
        case S_SEND_ECHO:
            {
                ConfigRadio(echoSF);
                delay(10);
                uint8_t tx[5] = {0, myID, PKT_SF_TEST, echoSF, 0};
                txDone = false;
                Radio.Send(tx, 5);
                
                unsigned long t = millis();
                while(!txDone && millis()-t < 1000) Radio.IrqProcess();
                
                ConfigRadio(12); // Revert to listener
                currentState = S_IDLE;
                st7735.st7735_write_str(0, 60, "               ", Font_7x10, ST7735_BLACK);
            }
            break;

        case S_PERFORM_SCAN:
            if (packetReceived) {
                packetReceived = false;
                if (rxBuffer[2] == PKT_DISCOVERY_RESP && neighborCount < 10) {
                    foundNeighbors[neighborCount++] = rxBuffer[1];
                }
            }
            if (millis() - stateStartTime > DISCOVERY_TIMEOUT) {
                reportIndex = 0;
                currentState = S_REPORT_NEIGHBOR;
            }
            break;

        case S_REPORT_NEIGHBOR:
            if (reportIndex < neighborCount) {
                ConfigRadio(12);
                uint8_t tx[5] = {0, myID, PKT_REPORT_NODE, foundNeighbors[reportIndex], 0};
                txDone = false;
                Radio.Send(tx, 5);
                unsigned long t = millis();
                while(!txDone && millis()-t < 1000) Radio.IrqProcess();
                reportIndex++;
                delay(200);
            } else {
                currentState = S_REPORT_DONE;
            }
            break;

        case S_REPORT_DONE:
            {
                ConfigRadio(12);
                uint8_t tx[5] = {0, myID, PKT_SCAN_DONE, 0, 0};
                Radio.Send(tx, 5);
                currentState = S_IDLE;
                UpdateScreen(); // Clear status msg
            }
            break;
    }
}